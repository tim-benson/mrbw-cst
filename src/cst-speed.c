/*************************************************************************
Title:    Scale-speed simulation for Control Stand Throttle
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-speed.c
License:  GNU General Public License v3

LICENSE:
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

#include "lcd.h"
#include "cst-speed.h"

// 8.8 fixed-point simulated speed step, 0 .. (126<<8). Written by updateSpeed10Hz() and read by
// printSpeed(), both from the main loop (updateSpeed10Hz() is gated by a flag TIMER0_COMPA_vect sets
// every 100ms - see speed10HzTick in mrbw-cst.c), so no cross-context guard is needed.
static uint16_t simSpeedStepQ8 = 0;

// Start Delay state: gates move-off from a full stop (with no brake applied) until the prime-mover
// spool-up delay (decoder CV167 - see updateSpeed10Hz()) elapses. Re-arms only once simSpeedStepQ8
// actually returns to a full stop - mirrors the real decoder, which only spools down/up around genuine
// stops, not every momentary zero-crossing of the commanded target. CV167*250 maxes at 63750, fits
// uint16_t comfortably.
static uint16_t startDelayRemainingMs = 0;
static uint8_t  delayArmed = 1;

// The smooth cubic ramp that replaces ACCPCT's old instant jump (see updateSpeed10Hz() for the
// mechanism, cst-speed.h for why this shape was chosen). rampActive/rampTickIndex track an in-progress
// ramp; rampP/rampQ/rampT/rampR0 are the cubic's precomputed terms, captured once when the ramp begins and
// held fixed for its duration. rampS is the ramp's own endpoint (always the 15mph-equivalent - see
// cst-speed.h). rampP/rampQ are int64_t (not int32_t) because SPEED_ACCEL_TARGET's rampR0 correction term
// can push them well past int32_t range at extreme CV/ACCTGT combinations - confirmed by pre-flash
// simulation across the practical parameter range (worst case ~37.5 billion vs int32_t's ~2.1 billion
// ceiling) before this was ever flashed.
static uint8_t  rampActive = 0;
static uint16_t rampTickIndex = 0;
static int64_t  rampP = 0;
static int64_t  rampQ = 0;
static uint16_t rampT = 0;
static uint16_t rampS = 0;
static int32_t  rampR0 = 0;

// Per-profile config (SPEED CFG), raw stored bytes indexed by the SPEED_ITEM_* enum
// (cst-speed.h). Entered on-device via SPEED_CONFIG_SCREEN, populated by mrbw-cst.c's
// readConfig() through speedSet() - same indexed-accessor-over-a-static-array idiom as
// cst-pressure.c's airbrakeCfg[]. Read directly as speedCfg[SPEED_ITEM_x] throughout the
// model below.
static uint8_t speedCfg[SPEED_ITEM_COUNT] =
{
	[SPEED_ITEM_ACCEL]            = MOMENTUM_ACCEL_CV3_DEFAULT,
	[SPEED_ITEM_DECEL]            = MOMENTUM_DECEL_CV4_DEFAULT,
	[SPEED_ITEM_BRAKE1]           = MOMENTUM_BRAKE1_CV179_DEFAULT,
	[SPEED_ITEM_BRAKE2]           = MOMENTUM_BRAKE2_CV180_DEFAULT,
	[SPEED_ITEM_BRAKE3]           = MOMENTUM_BRAKE3_CV181_DEFAULT,
	[SPEED_ITEM_START_DELAY]      = MOMENTUM_START_DELAY_DEFAULT,
	[SPEED_ITEM_MAX_MPH]          = SPEED_MAX_MPH_DEFAULT,
	[SPEED_ITEM_UNIT]             = SPEED_UNIT_KMH_DEFAULT,
	[SPEED_ITEM_HOLD_FN]          = SPEED_HOLD_WATCH_FN_DEFAULT,
	[SPEED_ITEM_STOP_FN]          = SPEED_STOP_WATCH_FN_DEFAULT,
	[SPEED_ITEM_OPLOAD]           = SPEED_OPLOAD_DEFAULT,
	[SPEED_ITEM_OPLOAD_FN]        = SPEED_OPLOAD_FN_DEFAULT,
	[SPEED_ITEM_PRLOAD]           = SPEED_PRLOAD_DEFAULT,
	[SPEED_ITEM_PRLOAD_FN]        = SPEED_PRLOAD_FN_DEFAULT,
	[SPEED_ITEM_TYPE]             = SPEED_TYPE_DEFAULT,
	[SPEED_ITEM_ACCEL_PCT]        = SPEED_ACCEL_PCT_DEFAULT,
	[SPEED_ITEM_ACCEL_TARGET]     = SPEED_ACCEL_TARGET_DEFAULT,
	[SPEED_ITEM_DECEL_PCT]        = SPEED_DECEL_PCT_DEFAULT,
	[SPEED_ITEM_DECEL_THRESHOLD]  = SPEED_DECEL_THRESHOLD_DEFAULT,
};

// Drive Hold falling-edge detection - see updateSpeed10Hz()'s top-of-function comment for why this only
// matters at the moment Hold releases, not while it's engaged or held.
static uint8_t previousHoldActive = 0;

// Steady-state-lag state - see updateSpeed10Hz() for the mechanism. steadyStopExtraMs is the extra decel
// time "locked in" for a deceleration (coast or brake) - captured once per deceleration and held steady
// until the loco settles or starts accelerating again. 0 if not applicable. Was formerly gated to only
// "genuine steady state" decelerations (WINDUP separately covered interrupted-acceleration decel) - WINDUP
// was removed after extensive hardware testing found this same correction, applied unconditionally to any
// deceleration, already covers the interrupted-acceleration case just as well - see CLAUDE.md.
static uint32_t steadyStopExtraMs  = 0;

// Tracks brakeActive/brakeSum from the previous tick, checked once at the top of updateSpeed10Hz() (before
// any of its early-return branches) so a transition into or out of braking - or a change in brake
// *strength* while still braking (e.g. STACK mode's lever sweeping through bands on its way to a stronger
// step, which it always does - it can't skip) - always resets steadyStopExtraMs. Without tracking brakeSum
// specifically, a correction captured against a weak step's much larger brakeBaseTicks would be held
// steady and wrongly applied against a later, much smaller brakeBaseTicks once a stronger step is reached -
// found on hardware causing the display to reach 0mph 10+ seconds early. See updateSpeed10Hz() for why a
// reset matters at all (steadyStopExtraMs is captured against a different reference, brakeTicksToCross()
// vs ticksToCross(DECEL), depending on which regime is active - and brakeTicksToCross() itself changes
// with brakeSum).
static uint8_t  wasBraking         = 0;
static uint8_t  lastBrakeSum       = 0;

// computeDelta()'s Bresenham-style remainder accumulator - see that function for why this exists.
static uint16_t lastTicksForDelta  = 0;
static uint16_t deltaRemainder     = 0;

static uint16_t speedMultiplierConst(void)
{
	return (SPEED_TYPE_V4V5MULT == speedCfg[SPEED_ITEM_TYPE]) ? SPEED_MULTIPLIER_V4V5MULT : SPEED_MULTIPLIER_V5DCC;
}

// Optional/Primary Load CVs (CV103/CV104): scales a base CV3/CV4 value by loadValue/128 before it's
// used, per the ESU manual's "Acceleration time = CV3 * (load value / 128)" formula. Widened to
// uint16_t since cv*loadValue can exceed 255 (e.g. CV=255, loadValue=255 -> 508) - only the raw CV
// inputs are 0-255, not the resulting scaled time, so this deliberately isn't clamped back to 8 bits.
static uint16_t applyLoad(uint8_t cv, uint8_t loadValue)
{
	return (uint16_t)(((uint32_t)cv * loadValue) / 128);
}

static uint16_t ticksToCross(uint16_t cv)
{
	// ticks = cv * multiplier * 10 ticks/sec = cv * multiplierConst / 100
	return (uint16_t)(((uint32_t)cv * speedMultiplierConst()) / 100);
}

// Computes 126<<8 / ticks with a Bresenham/DDA-style running remainder, so the long-run average delta
// converges to the true fractional value instead of being capped at whatever a single tick's integer
// truncation happens to give. Matters whenever ticks is large enough that delta's integer part is small
// (slow ACCEL/DECEL/brake rates) - plain per-tick truncation can then make small changes to ticks (e.g. a
// steady-state-lag correction) produce literally zero change in behavior, since the
// truncated quotient doesn't move until ticks crosses the next whole-quotient boundary. Resets
// automatically whenever ticks itself changes (a new accel/decel/brake regime, or a live CV change) - no
// explicit reset call is needed anywhere else in updateSpeed10Hz().
static uint16_t computeDelta(uint16_t ticks)
{
	if (0 == ticks)
		return 0xFFFF;
	if (ticks != lastTicksForDelta)
	{
		deltaRemainder = 0;
		lastTicksForDelta = ticks;
	}
	uint16_t deltaBase = (uint16_t)(((uint32_t)126 << 8) / ticks);
	uint16_t remainderPerTick = (uint16_t)(((uint32_t)126 << 8) % ticks);
	deltaRemainder += remainderPerTick;
	if (deltaRemainder >= ticks)
	{
		deltaRemainder -= ticks;
		return deltaBase + 1;
	}
	return deltaBase;
}

// Evaluates the standing-start cubic ramp position at tick tau (elapsed ticks since the ramp began),
// given the precomputed terms P/Q/T/r0 - see updateSpeed10Hz() and cst-speed.h's SPEED_ACCEL_TARGET comment
// for the derivation. position(tau) = tau^2*(P*tau+Q)/T^3 + r0*tau, expanded from the cubic Hermite form
// aτ³+bτ²+cτ (position(0)=0, rate(0)=r0, position(T)=S, rate(T)=plain ACCEL rate) to keep every
// intermediate an integer - P/Q here already have SPEED_ACCEL_TARGET's r0 correction folded in (see
// updateSpeed10Hz()), so this function just needs r0 again for the separate linear term. r0=0 (the
// original, still-supported case) makes this identical to the plain rate(0)=0 formula. Needs int64_t
// throughout: the unreduced numerator and the P/Q terms themselves both reach tens of billions across the
// practical ACCEL/SPEED_ACCEL_TARGET range, well past int32_t (confirmed by pre-flash simulation before
// this was ever flashed) but comfortably inside int64_t's range at AVR-tick timescales (~1-15s of 100ms
// ticks), where the extra software-emulated arithmetic cost is irrelevant.
static uint16_t cubicRampPosition(uint16_t tau, int64_t P, int64_t Q, uint16_t T, uint16_t S, int32_t r0)
{
	int64_t num = (int64_t)tau * (int64_t)tau * (P * (int64_t)tau + Q);
	int64_t denom = (int64_t)T * (int64_t)T * (int64_t)T;
	int64_t pos = (num / denom) + (int64_t)r0 * (int64_t)tau;
	if (pos < 0)
		return 0;
	if (pos > (int64_t)S)
		return S;
	return (uint16_t)pos;
}

// Solves for the cubic's initial slope r0 (Q8-steps/tick) such that the ramp reaches desiredPos exactly at
// tick tauTarget - see cst-speed.h's SPEED_ACCEL_TARGET comment for the full derivation. The ramp's position
// is linear in r0 for any fixed tau (position(tau) = basePos(tau) + r0*tau*(tau-T)^2/T^2, where basePos is
// the r0=0/P0,Q0 formula), so this is one division. Ceiling (not truncating) division is used so the ramp
// reaches the target at or before tauTarget, never a tick late - confirmed necessary by pre-flash
// simulation, plain truncation left it consistently a tick short. Returns 0 (falls back to the unmodified
// rate(0)=0 curve - identical to today's behavior) whenever the target can't be met by speeding up alone:
// tauTarget is 0 or >= T (degenerate), or the natural r0=0 curve already reaches desiredPos at or before
// tauTarget (the configured target asks for *more* delay than the ramp naturally has - a nonzero starting
// rate can only shorten the climb, never lengthen it).
static int32_t solveRampR0(uint16_t tauTarget, int64_t P0, int64_t Q0, uint16_t T, uint16_t S, uint16_t desiredPos)
{
	if (0 == tauTarget || tauTarget >= T)
		return 0;
	uint16_t basePos = cubicRampPosition(tauTarget, P0, Q0, T, S, 0);
	if (basePos >= desiredPos)
		return 0;
	int64_t diff = (int64_t)desiredPos - (int64_t)basePos;
	int64_t tauDiff = (int64_t)tauTarget - (int64_t)T;
	int64_t fNum = (int64_t)tauTarget * tauDiff * tauDiff;
	int64_t numer = diff * (int64_t)T * (int64_t)T;
	int64_t r0 = (numer + fNum - 1) / fNum;  // ceiling division - fNum and numer are always > 0 here
	return (r0 > 0) ? (int32_t)r0 : 0;
}

// Evaluates the ramp's position at tau (elapsed ticks since the ramp began) - the single point both the
// ramp's first tick and its "continuing" ticks evaluate through, so they can't drift out of sync.
static uint16_t rampPositionAt(uint16_t tau)
{
	return cubicRampPosition(tau, rampP, rampQ, rampT, rampS, rampR0);
}

static uint16_t brakeTicksToCross(uint8_t cvBrakeSum, uint16_t cv4)
{
	// stopSeconds = (255-cvBrakeSum)/255 * cv4*multiplier
	//   => ticks = (255-cvBrakeSum)*cv4*multiplierConst / (255*100)
	return (uint16_t)(((uint32_t)(255 - cvBrakeSum) * cv4 * speedMultiplierConst()) / (255UL * 100UL));
}

// Snap to a dead stop and drop every in-progress motion state - including an active standing-start
// ramp - so that releasing an e-stop or STOPFN resumes from a genuine standing start (Start Delay,
// then a fresh ramp from 0), not mid-ramp. Shared by updateSpeed10Hz()'s emergencyActive and
// watchedFunctionActive branches so the two can't drift apart. rampP/rampQ/rampT/rampS/rampR0 aren't
// cleared here - they're recomputed from scratch whenever a new ramp begins.
static void snapToStop(void)
{
	simSpeedStepQ8 = 0;
	startDelayRemainingMs = 0;
	delayArmed = 1;
	rampActive = 0;
	rampTickIndex = 0;
}

// Called once per 100ms tick from TIMER0_COMPA_vect. brake1/2/3Active are this pass's
// BRAKE_CONTROL/BK2_CONTROL/BK3_CONTROL bits from mrbw-cst.c's controls byte - passed in rather
// than read via extern so this file doesn't need to know mrbw-cst.c's bit layout. emergencyActive
// mirrors THROTTLE_STATUS_EMERGENCY (the throttle's own built-in e-stop); watchedFunctionActive/
// oploadActive/prloadActive/holdActive are true when the user's configured "watch" DCC functions
// (STOPFN/OPLOADFN/PRLOADFN/HOLDFN) are currently part of the outgoing functionMask, regardless of
// which physical control put them there - same mechanism for all four.
void updateSpeed10Hz(uint8_t commandedSpeedStep, uint8_t brake1Active, uint8_t brake2Active,
                         uint8_t brake3Active, uint8_t emergencyActive, uint8_t watchedFunctionActive,
                         uint8_t oploadActive, uint8_t prloadActive, uint8_t holdActive)
{
	// Drive Hold, falling edge only: if the target is already non-zero the instant Hold releases
	// (having been "revved" during the hold period), CV167's mechanical spool-up is considered
	// already done for this specific start, so skip just that contribution below - CV167 models
	// sound/mechanical revving, which has happened during the hold. Purely local/transient (not
	// persisted across ticks): computed fresh here and consumed by the Start Delay block later in
	// this same call, on this same tick.
	uint8_t skipStartDelayCV = (!holdActive && previousHoldActive && (0 != commandedSpeedStep)) ? 1 : 0;
	previousHoldActive = holdActive;

	if (emergencyActive)
	{
		// Highest priority - overrides even Hold. Snaps to zero and holds there for as long as it
		// stays active (re-checked fresh every tick). Drops any in-progress ramp/Start-Delay the same
		// way a genuine stop-with-no-demand already does, so normal operation resumes - from a fresh
		// standing start - the instant it clears.
		snapToStop();
		return;
	}

	if (holdActive)
	{
		// Freeze everything exactly as-is - brake/load/stop-function effects (and the stop-delay/
		// start-delay countdowns below) are all ignored while Drive Hold is active on the real
		// decoder. Nothing here is touched at all while frozen, so this resumes exactly where it
		// left off - or, if the requested speed changed during the freeze, picks up that new state
		// fresh - the instant Hold releases, with no extra tracking needed either way.
		return;
	}

	if (watchedFunctionActive)
	{
		// STOPFN - one of the "stop functions" Hold itself ignores, so only relevant once Hold
		// isn't active. Same snap-to-zero (and same ramp/Start-Delay reset) as emergencyActive above.
		snapToStop();
		return;
	}

	uint16_t targetQ8 = (uint16_t)commandedSpeedStep << 8;
	uint16_t current = simSpeedStepQ8;

	// Optional/Primary Load (CV103/CV104): scales CV3/CV4 while its watched function is active, 128 =
	// neutral. Primary Load wins if both are active simultaneously, matching the real decoder.
	uint8_t loadValue = prloadActive ? speedCfg[SPEED_ITEM_PRLOAD] : (oploadActive ? speedCfg[SPEED_ITEM_OPLOAD] : 128);

	// Brake1/2/3 CVs sum when stacked (capped at 255 = near-instant stop), rather than "fastest wins".
	uint16_t brakeSumRaw = 0;
	if (brake1Active) brakeSumRaw += speedCfg[SPEED_ITEM_BRAKE1];
	if (brake2Active) brakeSumRaw += speedCfg[SPEED_ITEM_BRAKE2];
	if (brake3Active) brakeSumRaw += speedCfg[SPEED_ITEM_BRAKE3];
	uint8_t brakeSum    = (brakeSumRaw > 255) ? 255 : (uint8_t)brakeSumRaw;
	uint8_t brakeActive = brake1Active || brake2Active || brake3Active;

	// Reset steadyStopExtraMs on any brake-regime transition (either direction) or any change in brake
	// *strength* while still braking - it's captured against brakeTicksToCross() (which itself depends on
	// brakeSum) while braking and ticksToCross(DECEL) otherwise, so carrying a value over across either
	// kind of transition would apply a correction computed against the wrong reference. The brakeSum
	// comparison specifically matters because STACK mode's lever always sweeps through every band on its
	// way to a stronger step (it can't skip) - without it, a correction captured against an early, weak
	// step's much larger brakeBaseTicks would be held steady and wrongly applied once a much smaller
	// brakeBaseTicks is reached. Checked here, unconditionally, before any of the branches below, so a
	// transition happening while sitting at a full stop (e.g. brake released mid-Start-Delay-spool) is
	// still caught correctly.
	if ((brakeActive != wasBraking) || (brakeActive && (brakeSum != lastBrakeSum)))
		steadyStopExtraMs = 0;
	wasBraking = brakeActive;
	if (brakeActive)
		lastBrakeSum = brakeSum;
	if (brakeActive)
		rampActive = 0;  // brake always wins immediately, regardless of the ramp's sub-phase

	if (rampActive)
	{
		// Continuing a ramp started on an earlier tick, checked ahead of (and independent of) the
		// current==0 test below - plain integer-truncation of the cubic's early ticks (more likely at a
		// small/zero SPEED_ACCEL_TARGET-solved r0) can legitimately leave current sitting at exactly 0 for
		// more than one tick, and current==0 must not be mistaken for "not yet started" while a ramp is
		// actually in progress, or execution would silently fall through to the unmodified plain-rate
		// accelerating logic below instead of continuing the ramp. rampS is re-clamped against the live
		// target every tick, so a target change (or drop) mid-ramp is handled the same way the rest of
		// this file already handles live target changes - rampP/rampQ/rampR0/rampT stay fixed (computed
		// against the original target), but the result is still clamped to liveRampTarget, so a target
		// drop mid-ramp still terminates cleanly.
		delayArmed = 0;  // ramping - not idle
		uint16_t liveRampTarget = (rampS > targetQ8) ? targetQ8 : rampS;
		if (current >= liveRampTarget || rampTickIndex >= rampT)
		{
			rampActive = 0;
		}
		else
		{
			rampTickIndex++;
			uint16_t pos = rampPositionAt(rampTickIndex);
			current = (pos >= liveRampTarget) ? liveRampTarget : pos;
			simSpeedStepQ8 = current;
			if (current < liveRampTarget && rampTickIndex < rampT)
				return;
			rampActive = 0;
		}
	}
	else if (0 == current)
	{
		if (brakeActive || 0 == targetQ8)
		{
			// Can't move off with a brake applied - and with no demand, just stay armed and wait.
			delayArmed = 1;
			startDelayRemainingMs = 0;
			return;
		}
		if (delayArmed)
		{
			if (0 == startDelayRemainingMs)
				// CV167 x 0.25s, in ms (skipped if just "revved" via Drive Hold - see
				// skipStartDelayCV above).
				startDelayRemainingMs = skipStartDelayCV ? 0 : (uint16_t)speedCfg[SPEED_ITEM_START_DELAY] * 250;
			if (startDelayRemainingMs > 100)
			{
				startDelayRemainingMs -= 100;
				return;  // still spooling up - hold at zero
			}
			startDelayRemainingMs = 0;
			delayArmed = 0;  // delay consumed - begin the smooth cubic ramp below

			// Smooth cubic ease-in from a genuine standing start, replacing ACCPCT's old instant jump.
			// The ramp always converges at 15mph (rampS is always the 15mph-equivalent, never the
			// commanded target) so that every standing start gets the identical onset shape -
			// position(tau) = tau^2*(P*tau+Q)/T^3 + r0*tau (cubicRampPosition() above), chosen so
			// position(0)=0, position(rampT)=rampS, rate(rampT)=plain ACCEL rate (no kink handing off to
			// normal climbing) - see cst-speed.h's SPEED_ACCEL_TARGET comment for the full derivation of
			// r0 and why rate(0)=r0 (not necessarily 0) still preserves both of those endpoint conditions
			// exactly. A commanded target below 15mph clamps the ramp's *output* to that target
			// (liveRampTarget below, same clamp the "continuing an active ramp" branch already applies
			// every subsequent tick) rather than building a different, shorter-duration ramp - otherwise a
			// low notch would reach its target via a different early slope than a high notch, making
			// time-to-first-visible-mph depend on the commanded speed, which defeats the point of a
			// uniform onset paid back by 15mph.
			uint16_t accelTicksNow = ticksToCross(applyLoad(speedCfg[SPEED_ITEM_ACCEL], loadValue));
			uint16_t v = (accelTicksNow > 0) ? (uint16_t)(((uint32_t)126 << 8) / accelTicksNow) : 0xFFFF;
			uint32_t s15Q8 = ((uint32_t)15 * 126 * 256) / speedCfg[SPEED_ITEM_MAX_MPH];
			rampS = (uint16_t)s15Q8;

			// ACCPCT's leadTime budget - the ramp's *total* duration to 15mph, unaffected by
			// SPEED_ACCEL_TARGET (see below) - so this stays exactly the already hardware-calibrated value.
			uint32_t headstartQ8 = ((uint32_t)speedCfg[SPEED_ITEM_ACCEL_PCT] * (126UL << 8)) / 255;
			uint16_t plainTicksToS = (uint16_t)(((uint32_t)rampS * accelTicksNow) / (126UL << 8));
			uint16_t headstartTicks = (uint16_t)(((uint32_t)headstartQ8 * accelTicksNow) / (126UL << 8));
			rampT = (plainTicksToS > headstartTicks) ? plainTicksToS - headstartTicks : 1;

			// SPEED_ACCEL_TARGET: target ticks-to-1mph, solved for the initial ramp slope r0 that hits it
			// exactly (solveRampR0() above) - see cst-speed.h for the full derivation and the graceful
			// fallback (r0=0, today's unmodified curve) when the target can't be reached by speeding up.
			int64_t P0 = (int64_t)v * (int64_t)rampT - 2 * (int64_t)rampS;
			int64_t Q0 = 3 * (int64_t)rampS * (int64_t)rampT - (int64_t)v * (int64_t)rampT * (int64_t)rampT;
			uint16_t desiredPos = (uint16_t)(((uint32_t)126 << 8) / (2 * (uint32_t)speedCfg[SPEED_ITEM_MAX_MPH]));  // ~0.5mph
			rampR0 = solveRampR0(speedCfg[SPEED_ITEM_ACCEL_TARGET], P0, Q0, rampT, rampS, desiredPos);
			rampP = P0 + (int64_t)rampR0 * (int64_t)rampT;
			rampQ = Q0 - 2 * (int64_t)rampR0 * (int64_t)rampT * (int64_t)rampT;

			if (rampS > 0)
			{
				uint16_t liveRampTarget = (rampS > targetQ8) ? targetQ8 : rampS;
				rampTickIndex = 1;
				uint16_t pos = rampPositionAt(rampTickIndex);
				current = (pos >= liveRampTarget) ? liveRampTarget : pos;
				simSpeedStepQ8 = current;
				if (current < liveRampTarget && rampTickIndex < rampT)
				{
					rampActive = 1;
					return;
				}
			}
			// else: rampS is 0 - current stays 0, falls through to the normal accelerating logic below.
		}
	}
	else
	{
		delayArmed = 0;  // moving under a normal (non-ramp) climb/decel - re-arm only once back at a
		                  // genuine stop. rampActive is already false here (handled above), so there's
		                  // nothing else to do in this branch.
	}

	uint16_t ticks;
	uint16_t delta;

	if (brakeActive)
	{
		// Brake overrides throttle demand entirely - always heads toward a full stop while held. The
		// steady-state-lag approximation (DECTHR/DECPCT) applies to any braking deceleration, not just
		// from a genuine steady state - reusing the same formula and tunables as the non-brake case, just
		// referencing brakeTicksToCross() instead of ticksToCross(DECEL). (Used to be scoped to
		// genuine-steady-state-then-brake only, mirroring a separate WINDUP mechanism for
		// interrupted-acceleration-then-brake - WINDUP was removed after hardware testing found this same
		// correction, applied unconditionally, already covers that case just as well - see CLAUDE.md.)
		if (0 == current)
			return;

		uint16_t brakeBaseTicks = brakeTicksToCross(brakeSum, applyLoad(speedCfg[SPEED_ITEM_DECEL], loadValue));

		if (0 == steadyStopExtraMs)
		{
			// Identical formula to the non-brake steady-state-lag capture below, referencing
			// brakeBaseTicks instead of ticksToCross(DECEL).
			uint16_t thresholdQ8 = (uint16_t)speedCfg[SPEED_ITEM_DECEL_THRESHOLD] << 8;
			uint16_t excessQ8 = (current > thresholdQ8) ? (current - thresholdQ8) : 0;
			uint32_t excessTicks = ((uint32_t)brakeBaseTicks * excessQ8) / current;

			steadyStopExtraMs = (excessTicks * 100 * speedCfg[SPEED_ITEM_DECEL_PCT]) / 255;
		}

		uint16_t reduceTicks = (uint16_t)(steadyStopExtraMs / 100);
		ticks = (reduceTicks >= brakeBaseTicks) ? 1 : brakeBaseTicks - reduceTicks;
		delta = computeDelta(ticks);
		simSpeedStepQ8 = (delta >= current) ? 0 : current - delta;
		return;
	}

	if (current == targetQ8)
	{
		steadyStopExtraMs = 0;
		return;
	}

	// Steady-state lag: approximates real decoder BEMF-regulation PID wind-up on deceleration - an
	// explicitly linear first approximation (see cst-speed.h). Captured once at the transition into
	// decelerating and held steady until the climb/decel ends (settles or reverses). Applies to *any*
	// deceleration, not just from a genuine steady state - see the comment on WINDUP's removal in
	// CLAUDE.md for why the earlier interrupted-acceleration-only distinction was dropped.
	uint8_t accelerating = (targetQ8 > current);
	uint16_t accelTicks = ticksToCross(applyLoad(speedCfg[SPEED_ITEM_ACCEL], loadValue));

	if (accelerating)
	{
		steadyStopExtraMs = 0;
	}
	else if (0 == steadyStopExtraMs)
	{
		// current already reflects the steady speed at this instant - simSpeedStepQ8 hasn't been
		// updated yet this tick. Scaled by current (the actual distance this decel run will cover,
		// since the target here is always 0) rather than the full 126-step range - current is
		// guaranteed nonzero in this branch (unreachable with current==0, which would force
		// accelerating==true instead).
		uint16_t thresholdQ8 = (uint16_t)speedCfg[SPEED_ITEM_DECEL_THRESHOLD] << 8;
		uint16_t excessQ8 = (current > thresholdQ8) ? (current - thresholdQ8) : 0;
		uint16_t decelTicksFull = ticksToCross(applyLoad(speedCfg[SPEED_ITEM_DECEL], loadValue));
		uint32_t excessTicks = ((uint32_t)decelTicksFull * excessQ8) / current;

		steadyStopExtraMs = (excessTicks * 100 * speedCfg[SPEED_ITEM_DECEL_PCT]) / 255;
	}

	if (accelerating)
	{
		ticks = accelTicks;
	}
	else
	{
		// Real decoder stops faster than plain DECEL predicts - speed the simulation up to match
		// (subtract ticks), floored at 1 so it can't underflow to 0 or negative if the correction
		// would otherwise exceed the whole decel time.
		uint16_t decelBaseTicks = ticksToCross(applyLoad(speedCfg[SPEED_ITEM_DECEL], loadValue));
		uint16_t reduceTicks = (uint16_t)(steadyStopExtraMs / 100);
		ticks = (reduceTicks >= decelBaseTicks) ? 1 : decelBaseTicks - reduceTicks;
	}
	delta = computeDelta(ticks);  // CV==0: snap instantly

	if (accelerating)
		simSpeedStepQ8 = (current + delta > targetQ8) ? targetQ8 : current + delta;
	else
		simSpeedStepQ8 = (delta >= current || current - delta < targetQ8) ? targetQ8 : current - delta;
}

void resetSpeed(void)
{
	simSpeedStepQ8 = 0;
	startDelayRemainingMs = 0;
	delayArmed = 1;
	rampActive = 0;
	rampTickIndex = 0;
	rampP = 0;
	rampQ = 0;
	rampT = 0;
	rampS = 0;
	rampR0 = 0;
	previousHoldActive = 0;
	steadyStopExtraMs = 0;
	wasBraking = 0;
	lastBrakeSum = 0;
	lastTicksForDelta = 0;
	deltaRemainder = 0;
}

void printSpeed(void)
{
	uint16_t speedQ8 = simSpeedStepQ8;   // same (main-loop) context as the writer, updateSpeed10Hz()

	// Convert the configured max once, rather than converting the already-rounded displayed value,
	// to avoid compounding rounding error.
	uint32_t targetMax = speedCfg[SPEED_ITEM_MAX_MPH];
	if(SPEED_UNIT_KMH == speedCfg[SPEED_ITEM_UNIT])
		targetMax = ((uint32_t)speedCfg[SPEED_ITEM_MAX_MPH] * 1609 + 500) / 1000;  // mph -> km/h, rounded

	uint32_t speedVal = ((uint32_t)speedQ8 * targetMax + ((126UL << 8) / 2)) / (126UL << 8);

	// MAIN_SCREEN's field is a fixed 6 characters (mrbw-cst.c:1623-1628), immediately followed by the
	// UP/DOWN function glyph in the next column. speedVal can never exceed targetMax (speedQ8 tops out
	// exactly at 126<<8), so branching the field width on the *configured* max - not the live value -
	// guarantees a correct, non-truncating width that stays stable as the throttle accelerates, rather
	// than flipping padded/unpadded mid-session.
	if(targetMax > 99)
	{
		printDec3Dig((uint16_t)speedVal);
		lcd_puts((SPEED_UNIT_KMH == speedCfg[SPEED_ITEM_UNIT]) ? "KMH" : "MPH");
	}
	else
	{
		printDec2Dig((uint16_t)speedVal);
		lcd_puts((SPEED_UNIT_KMH == speedCfg[SPEED_ITEM_UNIT]) ? "KMH " : "MPH ");
	}
}

uint8_t speedGet(uint8_t item)
{
	return (item < SPEED_ITEM_COUNT) ? speedCfg[item] : 0;
}

void speedSet(uint8_t item, uint8_t value)
{
	if(item < SPEED_ITEM_COUNT)
		speedCfg[item] = value;
}
