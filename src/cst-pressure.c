/*************************************************************************
Title:    AIRBRAKE - air-brake system simulation for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Tim Benson <blw@east-slope.com>
File:     cst-pressure.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2019 Michael Petersen & Nathan Holmes
    Copyright (C) 2026 Tim Benson

    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <stdlib.h>

#include "cst-pressure.h"

// ~1 s pulse (10 Hz tick) on the BRK SET function at the start of each brake-pipe reduction.
#define VENT_PULSE_TICKS  10

// Minimum service reduction: a 26L can't make a smaller one - the first service handle position IS
// this reduction (quick service has to fire on every car, and brake-pipe gradient means the rear of
// a long train needs it too). The lever's computed reduction is floored to this once applied.
#define AIRBRAKE_MIN_REDUCTION_PSI  7

// Lever % at which the graduated service ramp reaches a full-service reduction (FULLSVC). Above this,
// up to AIRBRAKE_EMERG_PCNT, the handle laps at full service - more travel does nothing.
#define AIRBRAKE_FULLSVC_PCNT  70

// Lever % at/above which an emergency application dumps the brake pipe to 0 - only when the throttle's
// BRK ESTP option is on (emergencyBrakeEnabled). Hysteretic: releases 5 % below.
#define AIRBRAKE_EMERG_PCNT  95

// Service / emergency brake-pipe vent rates, PSI/s. Used to be BRAKE CFG items (VENT/EMRG VNT) -
// hardcoded at their long-standing defaults since they were never usefully varied per-loco.
#define AIRBRAKE_VENT_RATE_PSI_S        6
#define AIRBRAKE_EMERG_VENT_RATE_PSI_S 25

// Fixed crawl rate added to the tapered brake-pipe recharge (milliPSI per 10 Hz tick = 0.1 PSI/s),
// so the exponential's asymptotic tail still completes without stalling on integer division. Small
// and fixed (not scaled by CHARGE) so the last PSI genuinely creeps in - matches the original ISE
// sim's `+ 10`.
#define AIRBRAKE_RECHARGE_TAIL_MPSI  10

// Threshold (% of the MR governor band, MR OUT - MR IN) that a recharge's accumulated reservoir draw
// must reach to count as a "deep"/synchronising event (COMPRSR) rather than routine (COMPRSR2) - see
// COMPMODE / airCompressorReleaseRun(). 100% would be the worst-case guarantee that any loco, whatever
// its starting phase, gets pulled all the way to cut-in; tuned lower so a real full-service release
// reliably clears it too. Bench-calibrated model shape, like AIRBRAKE_MIN_REDUCTION_PSI/
// AIRBRAKE_FULLSVC_PCNT - not exposed as an AIRBRAKE CFG item.
#define AIRBRAKE_SYNC_BAND_PCT  80

// --- per-profile config (AIRBRAKE CFG), raw stored bytes; loaded by readConfig() in mrbw-cst.c ---
static uint8_t airbrakeCfg[AIRBRAKE_COUNT] =
{
	[AIRBRAKE_CHARGED]     = AIRBRAKE_CHARGED_DEFAULT,
	[AIRBRAKE_MR_CUTIN]    = AIRBRAKE_MR_CUTIN_DEFAULT,
	[AIRBRAKE_MR_CUTOUT]   = AIRBRAKE_MR_CUTOUT_DEFAULT,
	[AIRBRAKE_CHARGE_RATE] = AIRBRAKE_CHARGE_RATE_DEFAULT,
	[AIRBRAKE_LEAK_RATE]   = AIRBRAKE_LEAK_RATE_DEFAULT,
	[AIRBRAKE_PUMP_RATE]   = AIRBRAKE_PUMP_RATE_DEFAULT,
	[AIRBRAKE_DISPLAY]     = AIRBRAKE_DISPLAY_DEFAULT,
	[AIRBRAKE_COMP_MODE]   = AIRBRAKE_COMP_MODE_DEFAULT,
	[AIRBRAKE_MR_LOAD]     = AIRBRAKE_MR_LOAD_DEFAULT,
};

uint8_t airbrakeGet(uint8_t item)
{
	return (item < AIRBRAKE_COUNT) ? airbrakeCfg[item] : 0;
}

void airbrakeSet(uint8_t item, uint8_t value)
{
	if(item < AIRBRAKE_COUNT)
		airbrakeCfg[item] = value;
}

// --- live model state (main-loop context only - no ISR access, so no ATOMIC_BLOCK) ---
static uint32_t bpMilliPsi = (uint32_t)AIRBRAKE_CHARGED_DEFAULT * 1000UL;    // brake pipe
static uint32_t mrMilliPsi = (uint32_t)AIRBRAKE_MR_CUTOUT_DEFAULT * 1000UL;  // main reservoir
static uint8_t  emergencyActive = 0;  // hysteretic emergency-application latch (BRK ESTP option only)
static uint8_t  appliedLatch = 0;     // 0 = released -> BRAKE_REL_FN asserted
static uint8_t  wasVenting = 0;
static uint8_t  wasRecharging = 0;    // detects a new recharge's start edge, for the instant credit below
static uint8_t  ventPulseTimer = 0;   // BRK SET one-shot
static uint8_t  compressorOn = 0;
static uint32_t syncDrawMilliPsi = 0;       // pending consist-sync credit - see updateBrake10Hz()
static uint8_t  compressorRunIsRelease = 0; // latched at each compressor start edge, held for the run

void initAirBrake(void)
{
	uint32_t charged = (uint32_t)airbrakeCfg[AIRBRAKE_CHARGED] * 1000UL;
	uint8_t r = (uint8_t)rand();

	// Seed charged, with a small "gauge isn't dead on the mark" jitter. Seed the reservoir a little
	// below cut-out so it reads as settled rather than exactly at the governor threshold.
	uint32_t mrCutOut = (uint32_t)airbrakeCfg[AIRBRAKE_MR_CUTOUT] * 1000UL;
	bpMilliPsi = (charged > 1750) ? (charged - (uint32_t)(r & 0x07) * 250UL) : charged;
	mrMilliPsi = (mrCutOut > 2000) ? (mrCutOut - 1000UL - (uint32_t)((r >> 3) & 0x07) * 200UL) : mrCutOut;
	emergencyActive = 0;
	appliedLatch = 0;
	wasVenting = 0;
	wasRecharging = 0;
	ventPulseTimer = 0;
	compressorOn = 0;
	syncDrawMilliPsi = 0;
	compressorRunIsRelease = 0;
}

// Runs once per 10 Hz tick from the main loop, every tick regardless of the PREFS AIRBRAKE bit
// (the bit only gates whether the outputs reach the DCC packet).
//   independentBrakeAtRest = mrbw-cst.c's own per-BRK-TYPE "is the independent brake genuinely at
//     rest" check (brakeState for Standard/Step, currentStackBand for Stack, a raw lever-percent
//     fallback for Standard/Pulse where no reusable state exists) - see CLAUDE.md "Brake logic".
//     Replaces a separate AIRBRAKE-only APPLY threshold so the automatic-brake pipe's own
//     apply/release point tracks each mode's real rest boundary instead of an unrelated flat %.
//   emergencyBrakeEnabled = the throttle's BRK ESTP option is on. Only then does slamming the lever
//     to AIRBRAKE_EMERG_PCNT dump the pipe to 0; otherwise the handle just laps at full service.
//     It's a pure option flag (not the ESTOP_BRAKE state), so it still models on the AIRBRAKE screen.
// The ProtoThrottle's one brake lever IS the automatic (train) brake this models: a pipe recharge
// always draws the reservoir at MR_LOAD, and every reduction fires the BRK SET hiss.
void updateBrake10Hz(uint8_t leverPcnt, uint8_t independentBrakeAtRest, uint8_t emergencyBrakeEnabled)
{
	// Config -> working units, all in milliPSI per 100 ms tick. The fast pipe rates (VENT/EMRG) are
	// hardcoded (* 1000 / 10 == * 100); the slower rates (CHARGE/LEAK/PUMP) are still per-profile,
	// stored as PSI/min: * 1000 / 60 / 10 == * 100 / 60.
	uint32_t charged   = (uint32_t)airbrakeCfg[AIRBRAKE_CHARGED]     * 1000UL;
	// FULLSVC is no longer a stored item - derived fresh from CHARGED every tick (2/7 of it, rounded
	// UP to the nearest PSI via the standard ceil-div identity) so it always tracks BP CHARGE rather
	// than risking the two drifting apart. At the default CHARGED=90 this gives 26 PSI, matching the
	// old stored default exactly.
	uint32_t fullSvc   = (((uint32_t)airbrakeCfg[AIRBRAKE_CHARGED] * 2UL + 6UL) / 7UL) * 1000UL;
	uint32_t mrCutIn   = (uint32_t)airbrakeCfg[AIRBRAKE_MR_CUTIN]    * 1000UL;
	uint32_t mrCutOut  = (uint32_t)airbrakeCfg[AIRBRAKE_MR_CUTOUT]   * 1000UL;
	uint32_t ventRate  = (uint32_t)AIRBRAKE_VENT_RATE_PSI_S       * 100UL;
	uint32_t emergRate = (uint32_t)AIRBRAKE_EMERG_VENT_RATE_PSI_S * 100UL;
	uint32_t chargeRate= (uint32_t)airbrakeCfg[AIRBRAKE_CHARGE_RATE] * 100UL / 60UL;
	uint32_t leakRate  = (uint32_t)airbrakeCfg[AIRBRAKE_LEAK_RATE]   * 100UL / 60UL;
	uint32_t pumpRate  = (uint32_t)airbrakeCfg[AIRBRAKE_PUMP_RATE]   * 100UL / 60UL;
	uint32_t mrLoadPct = airbrakeCfg[AIRBRAKE_MR_LOAD];   // reservoir draw per PSI of pipe recharge, %

	// The compressor must out-pace the leak or it can never refill (mr never reaches mrCutOut and
	// the governor never cuts out - compressor stuck on). Floor the net fill at 10 mPSI/tick so a
	// PUMP <= LEAK misconfiguration still cycles (a long but finite on-phase) instead of sticking.
	if(pumpRate < leakRate + 10UL)
		pumpRate = leakRate + 10UL;

	// Guarantee a non-inverted, non-empty governor band. MR LOW >= MR HIGH (a plausible fat-finger on
	// two adjacent menu items - no on-device or import ordering guard) otherwise makes the two
	// governor comparisons below fight every tick: the compressor toggles at ~1-2 Hz forever,
	// stuttering COMPRESSOR_FN and sending a status packet on each edge. Also stops (mrCutOut -
	// mrCutIn) underflowing in the COMPRSR classifier further down.
	if(mrCutOut < mrCutIn + 1000UL)
		mrCutOut = mrCutIn + 1000UL;

	// Emergency application: hysteretic latch on lever percent, gated by the BRK ESTP option. Without
	// that option a 26L automatic brake has no emergency zone - the handle just laps at full service.
	if(emergencyBrakeEnabled && (leverPcnt >= AIRBRAKE_EMERG_PCNT))
		emergencyActive = 1;
	else if(!emergencyBrakeEnabled || (leverPcnt + 5 <= AIRBRAKE_EMERG_PCNT))
		emergencyActive = 0;

	// Brake-pipe pressure the handle is asking for:
	//  - at rest (independentBrakeAtRest) -> fully charged
	//  - service zone                -> at least a minimum reduction (a 26L can't make a smaller one),
	//                                   graduated up to a full-service reduction (FULLSVC) by
	//                                   AIRBRAKE_FULLSVC_PCNT lever
	//  - full service .. emergency   -> laps at full service; more handle travel does nothing (each
	//                                   car's aux reservoir has already equalized with its brake cyl)
	//  - emergency (latched)         -> dump the pipe to 0 (vented at the EMRG rate, below)
	uint32_t leverTarget;
	if(!independentBrakeAtRest)
	{
		// The ramp's own zero-point is a fixed 10% (matching the raw fallback independentBrakeAtRest
		// uses for Standard/Pulse); Step/Stack's real rest boundary sits at or above that (20%/17-25%),
		// so this guard is defensive rather than load-bearing at today's thresholds - it only matters
		// if some mode's rest boundary is ever configured/changed to sit below the ramp's anchor,
		// where leverPcnt-10 would otherwise underflow before the floor below catches it.
		uint32_t reduction = (leverPcnt > 10)
			? (uint32_t)(leverPcnt - 10) * fullSvc / (AIRBRAKE_FULLSVC_PCNT - 10)
			: 0;
		uint32_t minRed = (uint32_t)AIRBRAKE_MIN_REDUCTION_PSI * 1000UL;
		if(reduction < minRed)
			reduction = minRed;
		if(reduction > fullSvc)   // clamp after the floor so a degenerate FULLSVC < minRed still resolves
			reduction = fullSvc;
		leverTarget = (reduction < charged) ? (charged - reduction) : 0;
	}
	else
	{
		leverTarget = charged;
	}
	if(emergencyActive)
		leverTarget = 0;

	// Brake pipe: vent DOWN toward the target on a reduction and hold there; recharge only once the
	// independent brake is genuinely at rest. Not a continuous function of lever position - easing
	// the lever back while still applied just holds.
	uint32_t mrDemand = 0;
	uint8_t venting = 0;
	uint8_t recharging = 0;
	if(!independentBrakeAtRest && (bpMilliPsi > leverTarget))
	{
		uint32_t rate = emergencyActive ? emergRate : ventRate;
		uint32_t gap = bpMilliPsi - leverTarget;
		bpMilliPsi -= (gap < rate) ? gap : rate;
		venting = 1;
	}
	else if(independentBrakeAtRest && (bpMilliPsi < charged))
	{
		// Tapered (first-order) recharge toward `charged`, like the original ISE sim: fill per tick
		// = gap / K + a small fixed crawl. K is set so the initial rate from a full-service-sized
		// gap equals the configured CHARGE, so a bigger gap (emergency, from ~0) recharges faster at
		// first and everything slows as the pipe fills. The crawl (AIRBRAKE_RECHARGE_TAIL_MPSI) is
		// fixed, not scaled by CHARGE, so the exponential runs almost to the top and the last PSI
		// genuinely creeps in rather than the tail turning into a brisk linear ramp.
		uint32_t gap = charged - bpMilliPsi;
		// COMPRSR/COMPRSR2 classification: this release's full, deterministic reservoir draw is known
		// right now, before any of it has actually landed - credit it once, on the first tick of this
		// recharge, rather than gradually as mrDemand (below) happens to deliver it. See
		// updateBrake10Hz()'s header comment and the governor block below for the rest of the model.
		if(!wasRecharging)
		{
			syncDrawMilliPsi += gap * mrLoadPct / 100UL;
			// Anything at/beyond the classification threshold already means "deep"; cap at 2x the full
			// governor band so a pathological config where the compressor can never cut out (its
			// stop-edge reset never fires) can't accumulate this toward a uint32_t wrap over a long run.
			uint32_t syncCap = (mrCutOut - mrCutIn) * 2UL;
			if(syncDrawMilliPsi > syncCap)
				syncDrawMilliPsi = syncCap;
		}
		uint32_t rechargeK = (chargeRate > 0) ? (fullSvc / chargeRate) : 1;
		if(rechargeK < 1)
			rechargeK = 1;
		uint32_t fill = gap / rechargeK + AIRBRAKE_RECHARGE_TAIL_MPSI;
		if(fill > gap)
			fill = gap;
		bpMilliPsi += fill;
		// The recharge is the reservoir's transient load (whole trainline + car reservoirs recharge
		// off the main reservoir). Total draw over a full recharge is gap0 * MR_LOAD% regardless of
		// the taper - it just front-loads the compressor demand right after release.
		mrDemand = fill * mrLoadPct / 100UL;
		recharging = 1;
	}

	// Applied latch: 1 once a vent has started, back to 0 only once the independent brake is
	// genuinely at rest again. BRAKE_REL_FN is asserted while the latch is 0; its ON edge (the full
	// release) is the brake-release sound. The brake-set sound is BRAKE_SET_FN's pulse, below.
	if(venting && !appliedLatch)
		appliedLatch = 1;
	if(independentBrakeAtRest)
		appliedLatch = 0;

	// BRK SET pulse: the trainline exhaust hiss, one ~1 s pulse at the start of every venting
	// episode - the initial reduction and each further one.
	if(venting && !wasVenting)
		ventPulseTimer = VENT_PULSE_TICKS;
	wasVenting = venting;
	wasRecharging = recharging;
	if(ventPulseTimer)
		ventPulseTimer--;

	// Main reservoir + compressor governor. The base leak makes the compressor cycle when idle;
	// a release piles mrDemand on top so it kicks in harder - decoupled from the exact pipe value.
	// The pump is NOT clamped to mrCutOut: clamping to exactly the cut-out would let the next tick's
	// leak drop mr just below it, so "mr >= mrCutOut" (checked after the leak) would never fire and
	// the compressor would never stop. Instead it overshoots by up to one tick's pump, which the
	// next check catches.
	uint8_t compressorWas = compressorOn;
	uint32_t drain = leakRate + mrDemand;
	mrMilliPsi = (mrMilliPsi > drain) ? (mrMilliPsi - drain) : 0;
	if(mrMilliPsi <= mrCutIn)
		compressorOn = 1;
	if(mrMilliPsi >= mrCutOut)
		compressorOn = 0;
	if(compressorOn)
		mrMilliPsi += pumpRate;

	// COMPRSR/COMPRSR2 classification (COMPMODE = CONSIST only - see mrbw-cst.c). syncDrawMilliPsi is
	// "pending consist-sync credit": each release credits its own full, deterministic draw once, on
	// the first tick of its recharge (above) - not gradually as mrDemand happens to deliver it, so
	// the credit doesn't depend on MR's level or on how long delivery takes. It decays at LEAK, but
	// only in a genuine quiet gap (recharging false) - never while a recharge is actively delivering,
	// so a long single recharge's credit survives intact regardless of when the compressor notices
	// it. A run is "deep" once the pending credit reaches AIRBRAKE_SYNC_BAND_PCT of the governor
	// band; below that, indistinguishable from ordinary asynchronous idle cycling.
	if(!recharging)
		syncDrawMilliPsi = (syncDrawMilliPsi > leakRate) ? (syncDrawMilliPsi - leakRate) : 0;
	if(compressorOn && !compressorWas)
		compressorRunIsRelease = (syncDrawMilliPsi >= (mrCutOut - mrCutIn) * AIRBRAKE_SYNC_BAND_PCT / 100UL);
	if(!compressorOn && compressorWas)
		syncDrawMilliPsi = 0;
}

uint8_t airBrakeReleased(void)
{
	return !appliedLatch;
}

uint8_t airBrakeSetPulse(void)
{
	return (ventPulseTimer > 0);
}

uint8_t airCompressorOn(void)
{
	return compressorOn;
}

uint8_t airCompressorReleaseRun(void)
{
	return compressorRunIsRelease;
}

uint8_t airCompressorPendingRelease(void)
{
	uint32_t mrCutIn  = (uint32_t)airbrakeCfg[AIRBRAKE_MR_CUTIN]  * 1000UL;
	uint32_t mrCutOut = (uint32_t)airbrakeCfg[AIRBRAKE_MR_CUTOUT] * 1000UL;
	if(mrCutOut < mrCutIn + 1000UL)   // same non-inverted-band guard updateBrake10Hz() applies
		mrCutOut = mrCutIn + 1000UL;
	return (syncDrawMilliPsi >= (mrCutOut - mrCutIn) * AIRBRAKE_SYNC_BAND_PCT / 100UL);
}

uint8_t airEmergencyActive(void)
{
	return emergencyActive;
}

// The two readouts round to the nearest whole PSI rather than truncating - the model never lets the
// reservoir overshoot the governor cut-out by more than a fraction of a PSI, so a truncating display
// only ever shows MR HIGH for the sliver of time mrMilliPsi is in [cutout, cutout+1). Rounding widens
// that to +/-0.5 PSI, so MR HIGH (and MR LOW at the bottom of the cycle) visibly dwell like a real
// gauge resting at its switch points. Display-only - the governor and every other model decision read
// mrMilliPsi/bpMilliPsi directly in milliPSI.
uint8_t airBrakePipePsi(void)
{
	return (uint8_t)((bpMilliPsi + 500UL) / 1000UL);
}

uint8_t airMainResPsi(void)
{
	uint32_t psi = (mrMilliPsi + 500UL) / 1000UL;
	return (psi > 255) ? 255 : (uint8_t)psi;
}
