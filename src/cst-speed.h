/*************************************************************************
Title:    Scale-speed simulation for Control Stand Throttle
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-speed.h
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

#ifndef _CST_SPEED_H_
#define _CST_SPEED_H_

// --- SPEED CFG per-profile config items ---
// speedGet(item) / speedSet(item, val) index a single static array by these, the same
// indexed-accessor-over-a-static-array idiom as cst-pressure.c's airbrakeCfg[]. This is the
// storage order and is fixed; the SPEED_CONFIG_SCREEN display order is a separate concern
// driven by the per-TYPE descriptor in cst-speed.c (see speedItemAt()). The four correction
// tunables (ACCEL_PCT..DECEL_THRESHOLD) are hidden unless ADV FUNC is on.
enum
{
	SPEED_ITEM_ACCEL = 0,       // ACCEL   - mirrors decoder CV3
	SPEED_ITEM_DECEL,           // DECEL   - mirrors decoder CV4
	SPEED_ITEM_BRAKE1,          // BRK1    - mirrors decoder CV179
	SPEED_ITEM_BRAKE2,          // BRK2    - mirrors decoder CV180
	SPEED_ITEM_BRAKE3,          // BRK3    - mirrors decoder CV181
	SPEED_ITEM_START_DELAY,     // DELAY   - mirrors decoder CV167
	SPEED_ITEM_MAX_MPH,         // MAXSPEED
	SPEED_ITEM_UNIT,            // UNIT    - SPEED_UNIT_MPH/_KMH
	SPEED_ITEM_HOLD_FN,         // HOLDFN  - watched DCC fn 0-28, 255=OFF
	SPEED_ITEM_STOP_FN,         // STOPFN  - watched DCC fn 0-28, 255=OFF
	SPEED_ITEM_OPLOAD,          // OPLOAD  - mirrors decoder CV103
	SPEED_ITEM_OPLOAD_FN,       // OPLOADFN - watched DCC fn 0-28, 255=OFF
	SPEED_ITEM_PRLOAD,          // PRLOAD  - mirrors decoder CV104
	SPEED_ITEM_PRLOAD_FN,       // PRLOADFN - watched DCC fn 0-28, 255=OFF
	SPEED_ITEM_TYPE,            // TYPE    - SPEED_TYPE_V5DCC/_V5MULT/_V4
	SPEED_ITEM_ACCEL_PCT,       // ACCPCT  - standing-start head-start, ADV FUNC only
	SPEED_ITEM_ACCEL_TARGET,    // ACCTGT  - target ticks-to-1mph, ADV FUNC only
	SPEED_ITEM_DECEL_PCT,       // DECPCT  - steady-state decel-lag strength, ADV FUNC only
	SPEED_ITEM_DECEL_THRESHOLD, // DECTHR  - decel-lag speed threshold, ADV FUNC only
	SPEED_ITEM_ACCEL_ADJ,       // ACCELADJ  - mirrors decoder CV23 (added to CV3), V5 only
	SPEED_ITEM_DECEL_ADJ,       // DECELADJ  - mirrors decoder CV24 (added to CV4), V5 only
	SPEED_ITEM_COUNT
};

#define MOMENTUM_ACCEL_CV3_DEFAULT      60
#define MOMENTUM_DECEL_CV4_DEFAULT     230
#define MOMENTUM_BRAKE1_CV179_DEFAULT  130
#define MOMENTUM_BRAKE2_CV180_DEFAULT   70
#define MOMENTUM_BRAKE3_CV181_DEFAULT  100
#define MOMENTUM_START_DELAY_DEFAULT    13
#define SPEED_MAX_MPH_DEFAULT           50

#define SPEED_UNIT_MPH                   0
#define SPEED_UNIT_KMH                   1
#define SPEED_UNIT_KMH_DEFAULT           SPEED_UNIT_MPH

// Watched DCC function number (0-28) for the speed-display stop trigger; 255 = OFF/disabled. 255 is
// also EEPROM's natural erased-byte value, so an unprogrammed/upgraded chip safely defaults to OFF.
#define SPEED_STOP_WATCH_FN_OFF        255
#define SPEED_STOP_WATCH_FN_DEFAULT    SPEED_STOP_WATCH_FN_OFF

// Decoder families. The momentum multiplier (x1000) feeds ticksToCross()/brakeTicksToCross(): 896 is
// NMRA S9.2.2's 0.896, specific to ESU LokSound 5 DCC (non-MultiProtocol); 250 is the general-case
// 0.25 the ESU manual gives for CV3, used by LokPilot/LokSound V4 and by LokSound/LokPilot V5
// MultiProtocol. SPEED_TYPE_V5MULT keeps the value (1) that was SPEED_TYPE_V4V5MULT - the split is a
// rename plus a genuinely new SPEED_TYPE_V4 (2), so a stored TYPE of 1 keeps its exact behaviour.
// V4 shares the V5 MultiProtocol model and multiplier and differs only in which parameters it exposes:
// it drops BRK2/BRK3 and the load CVs (a V4 decoder has no CV180/CV181/CV103/CV104), which
// speedResetModel()/speedApplyTypeInert() force inert, so the model math needs no per-type branch.
#define SPEED_TYPE_V5DCC                 0
#define SPEED_TYPE_V5MULT               1
#define SPEED_TYPE_V4                    2
#define SPEED_TYPE_DEFAULT               SPEED_TYPE_V5DCC
#define SPEED_MULTIPLIER_V5DCC          896
#define SPEED_MULTIPLIER_V5MULT         250   // and SPEED_TYPE_V4

// Optional/Primary Load CVs (decoder CV103/CV104 mirrors) - a 0-255 value, 128 = neutral, that scales
// both CV3 (accel) and CV4 (decel/brake) while its watched DCC function is active - see cst-speed.c's
// applyLoad(). 128 is also a safe readByteOrDefault() fallback distinct from EEPROM's 0xFF erased value.
#define SPEED_OPLOAD_DEFAULT            128
#define SPEED_PRLOAD_DEFAULT            128

// Watched DCC function numbers (0-28) that activate OPLOAD/PRLOAD, same OFF/255 encoding and "watch
// whatever's in the outgoing functionMask" mechanism as SPEED_STOP_WATCH_FN above - not a Functions-enum
// entry, since this observes an already-configured function rather than owning a control itself.
#define SPEED_OPLOAD_FN_DEFAULT         SPEED_STOP_WATCH_FN_OFF
#define SPEED_PRLOAD_FN_DEFAULT         SPEED_STOP_WATCH_FN_OFF

// ESU Drive Hold: watched DCC function (0-28, 255=OFF) that freezes the speed simulation while active -
// same "watch whatever's in the outgoing functionMask" mechanism as the fields above, but defaults to
// F09 rather than OFF, since Drive Hold is meant to work out of the box.
#define SPEED_HOLD_WATCH_FN_DEFAULT     9

// Steady-state deceleration lag: a PID wind-up approximation (explicitly a linear first approximation,
// not a precise model, since the real decoder's control-loop gains aren't known) for decelerating from a
// genuine steady state. Even with zero acceleration history, the real locomotive was found to consistently
// stop *before* the display reaches 0mph (the display lags), growing with steady-state speed above a
// threshold - i.e. the real decoder decelerates faster than the plain DECEL-only model predicts. So this
// term is subtracted from the decel ticks rather than added - it speeds the simulation up to match, not
// slows it down. (An earlier attempt wrongly used an "add ticks" shape here, carried over from a
// physical-reasoning error - a KI-integral-unwind hypothesis that implied the real loco should be
// *slower*, before hardware testing confirmed the opposite. Caught and fixed the same day.)
//
// Originally scoped to genuine-steady-state-only deceleration, mutually exclusive with a separate WINDUP
// mechanism that covered interrupted-acceleration deceleration - WINDUP was removed after extensive
// hardware testing found this same correction, applied unconditionally to *any* deceleration, already
// covers the interrupted-acceleration case just as well (see CLAUDE.md for the full removal writeup).
//
// Calibrated from 15 hardware measurements across DECEL 32-255 and speeds 4-50mph: lag(speed) = slope x
// max(0, speedStep - threshold). threshold was found roughly constant across DECEL values (~10.5 steps),
// while slope was found proportional to DECEL's own full-range crossing time (ticksToCross(DECEL)) - a
// "percentage of a CV-derived reference time" shape, scaled by "how far above a threshold speed" instead
// of by a gap. DECEL>230 showed non-monotonic behavior on the decoder
// side during calibration (confirmed not a bug in this codebase's math - ticksToCross() is strictly
// linear in the raw CV) and was excluded from the fit; this approximation's practical scope is DECEL<=230.
//
// Two more bugs found and fixed during initial hardware testing, alongside the sign error above: the
// capture formula's scaling denominator (should be the actual distance this decel run covers, i.e.
// current, not the full 126-step range - silently weakened the effect) and delta's plain per-tick integer
// truncation, see computeDelta() (could make the effect disappear entirely - small DECPCT changes not
// enough to cross delta's next truncation boundary). All three fixed; DECTHR=11, DECPCT=22 is the
// re-derived best-fit starting point against the fully-corrected implementation (predictions within
// ~0.1-0.3s of all 12 clean DECEL=192/216/230 measurements) - like every other wind-up parameter, a
// starting point for on-device tuning, not a guaranteed-final value.
#define SPEED_DECEL_THRESHOLD_DEFAULT    11
#define SPEED_DECEL_PCT_DEFAULT          22

// SSFLOOR/SSFLOORCUT (a guaranteed minimum low-speed top-up for the correction above, only armed when a
// deceleration run itself began below SSFLOORCUT - never for the low-speed tail of a run that started
// higher) were removed.

// ACCPCT: standing-start head-start budget. Hardware measurements (30 points, ACCEL 30-180, from a
// genuine stop) found the real locomotive reaches every measured target speed *faster* than the plain
// ACCEL/CV3 linear model predicts - not proportional to how far into the ramp you are (unlike
// DECTHR/DECPCT's shape), close to a *constant number of seconds* across every target speed within a
// given ACCEL curve, scaling with ACCEL itself: leadTime =~ 0.0285 x ACCEL seconds (through-origin fit,
// residuals under 0.25s across the full 30-180 sweep). Expressed here as a fraction of ACCEL's own
// full-range crossing time (0.0285/0.896 =~ 3.18%), same 0-255=0-100% convention as DECPCT.
//
// Originally implemented as a one-time position jump - replaced (see updateSpeed10Hz()) by a smooth
// cubic ramp once on-device testing found the instant jump gave `current` a step-change with no physical
// basis, which fed an inflated value into DECTHR/DECPCT/SSFLOOR whenever a deceleration interrupted an
// early climb. ACCPCT's own meaning is unchanged - still the total head-start budget - only how that
// budget gets spent changed (a smoother average-rate boost across the whole 0-15mph ramp, instead of an
// instant jump). The ramp's own construction - position(0)=0 always, rate(0)=0 originally (see the
// SPEED_ACCEL_TARGET comment below for why rate(0) is now r0, not necessarily 0) - means there's no jump
// left to hide, so the LIFTOFF hold this section previously described (an artificial pause at zero before
// the jump) was found redundant and removed once on-device testing with LIFTOFF=0 confirmed the ramp alone
// already looks and measures the same: the ramp begins the instant Start Delay elapses, with no separate
// onset delay of its own.
//
// The ramp's own shape (rampS/rampP/rampQ/rampT/rampR0 in updateSpeed10Hz()) always targets the
// 15mph-equivalent speed, regardless of the commanded notch/target - the point is a uniform onset (paid
// back by 15mph) for every standing start, not a per-target ramp. A commanded target below 15mph clamps
// the ramp's *output* to that target (the same liveRampTarget clamp already used every tick a ramp is in
// progress) rather than building a shorter-duration ramp of its own - an earlier version clamped rampS
// itself to the commanded target when it was lower, which meant a low notch got a shorter rampT and so a
// visibly steeper/faster early climb than a high notch, making time-to-first-visible-mph depend on the
// commanded speed. Found and fixed 2026-08-19 after on-hardware timing measurements (video-timed 0->1mph)
// showed a >1s difference between a low-notch start and a high-notch start that should have been identical.
//
// Deliberately scoped to a genuine standing start (current==0) only - a follow-up sweep (47 further
// measurements, ACCEL 30-110, mid-cruise notch-up rather than standing starts) confirmed the same effect
// also occurs when accelerating from an already-moving steady state, smaller in magnitude and with a
// genuinely non-monotonic shape relative to starting speed that doesn't fit any of this codebase's existing
// simple shapes - simulated and confirmed that applying this same leadTime uniformly to those mid-cruise
// cases makes predictions *worse* on average (mean absolute error 0.54s -> 1.04s across 40 clean
// measurements), not better, since the mid-cruise effect is smaller than the full standing-start value at
// almost every starting point. Left as a documented, separate follow-up rather than folded in here.
#define SPEED_ACCEL_PCT_DEFAULT           8

// SPEED_ACCEL_TARGET: the *target* time-to-1mph, directly in ticks (0.1s/tick, since updateSpeed10Hz() runs
// at exactly 10Hz) - not a literal hold. The firmware solves for whatever ramp shape hits this target
// exactly, rather than the caller reasoning about hold-vs-slope mechanics. Distinct from ACCPCT: ACCPCT's
// leadTime budget determines the ramp's *total*, physically-calibrated duration to 15mph (rampT), which
// must keep matching the hardware-measured arrival time at 15mph regardless of how the onset is tuned -
// see the ACCPCT comment above. SPEED_ACCEL_TARGET never changes rampT; it only reshapes the curve inside
// that fixed window.
//
// Mechanism: originally the cubic's boundary condition was rate(0)=0 (see cst-speed.c's older revisions/
// CLAUDE.md) - correct for reproducing ACCPCT's own hardware-measured *total* arrival time, but it forces a
// tangent-flat opening whose absolute duration scales with rampT (~14.5s in a typical config), which
// measured out to ~1.9s before any visible movement - more than double the ~0.8s "getting going" delay
// this was supposed to reproduce (see CLAUDE.md's "smooth cubic ramp"/`LIFTOFF` history). Generalizing the
// boundary condition to rate(0)=r0 (a configurable nonzero initial slope, instead of a forced 0) still
// gives a fully-determined cubic - solving the 2x2 system for the a/b Hermite coefficients (c=r0, d=0
// fixed) gives, in the same A/T^3 form cst-speed.c's cubicRampPosition() already uses:
//
//   A = (v+r0)*T - 2*S          (was: P = v*T - 2*S)
//   B = 3*S*T - v*T^2 - 2*r0*T^2  (was: Q = 3*S*T - v*T^2)
//   position(tau) = tau^2*(A*tau+B)/T^3 + r0*tau
//
// Verified by direct substitution: position(0)=0 (unchanged - the loco still visibly starts from a dead
// stop, no jump/discontinuity, just a nonzero initial slope), and both position(T)=S and rate(T)=v still
// hold exactly - the r0 terms cancel out of both endpoint conditions algebraically. r0=0 reduces to
// exactly the original formula (A=P, B=Q). This is why SPEED_ACCEL_TARGET can't perturb ACCPCT's calibrated
// arrival time at 15mph: r0 only reshapes the *interior* of the curve, never its two fixed endpoints.
//
// Solving for r0 given a *target* tick (rather than picking r0 directly) is possible because position(tau)
// is linear in r0 for any fixed tau: position(tau) = basePos(tau) + r0*tau*(tau-T)^2/T^2, where basePos is
// the r0=0 formula - see cst-speed.c's solveRampR0(). desiredPos is the ~0.5mph threshold (not 1.0mph) -
// printSpeed() rounds to the nearest whole mph, so "shows 1mph" really means "reaches 0.5mph".
//
// Graceful fallback when the configured target is *slower* than the ramp's own r0=0 natural onset (the
// solved r0 comes out negative - not physical, since the loco can't have a negative initial speed):
// solveRampR0() clamps to r0=0 and the ramp uses its unmodified natural shape - a no-worse-than-before
// fallback, not a regression, and the same path SPEED_ACCEL_TARGET=0 itself takes (target tick 0 is a
// genuinely impossible target, since position(0)=0 always) - so 0 reproduces the exact pre-SPEED_ACCEL_
// TARGET behavior.
//
// Default 5 (0.5s) - hardware-tested and confirmed accurate for this locomotive (originally shipped at 8,
// "as originally scoped", before on-device re-tuning settled on 5). Devices with an already-saved value
// keep it until SPEED CFG is next visited and re-saved (the value's *meaning* changed here, from a literal
// hold to a target).
#define SPEED_ACCEL_TARGET_DEFAULT       5

// ACCELADJ / DECELADJ mirror ESU LokSound/LokPilot 5 CV23 (Adjust Acceleration) / CV24 (Adjust
// Deceleration): a signed factor -127..+127 added to CV3 / CV4. The stored byte uses the decoder's
// own encoding - magnitude 0-127 in bits 0-6, subtract if bit 7 (0x80) is set - so it reads the same
// as the value on the physical decoder. -127 is stored as 0xFF, which collides with the
// readByteOrDefault sentinel, so readConfig() reads these two bytes RAW (like ACCEL/DECEL/HOLDFN) and
// the layout -> 4 migration seeds 0x61/0x62 to 0. speedEffAccelCV()/speedEffDecelCV() in cst-speed.c
// decode the byte, add to the base, and clamp. V5 only; a V4 decoder has no CV23/CV24, so V4 forces
// both to 0.
#define SPEED_ACCEL_ADJ_DEFAULT           0
#define SPEED_DECEL_ADJ_DEFAULT           0
#define SPEED_ADJ_MAG_MAX               127   // full ESU CV23/CV24 magnitude range

// --- Decoder-family descriptor ---
// TYPE (SPEED_ITEM_TYPE) tags a decoder family. Five SPEED CFG items are type-agnostic - they mean
// the same thing for every family and always show first, in this order: TYPE, MAXSPEED, UNIT,
// ACCEL, DECEL - except that ACCELADJ/DECELADJ (V5 only) are spliced into that run immediately
// after the ACCEL/DECEL they adjust. Everything else is a model parameter whose presence and menu
// order come from the per-family descriptor in cst-speed.c; speedItemAt() walks that descriptor
// (after the agnostic spine and the adjust splice) to map a menu position to a SPEED_ITEM_*.
// BRK1 (CV179) is the shared lead brake item every family exposes - present for all three, but
// grouped with BRK2/BRK3 rather than sitting in the agnostic block. V5DCC and V5MULT carry the
// identical 16-parameter model set (only the multiplier differs); V4 carries an 8-parameter subset -
// it drops ACCELADJ/DECELADJ, BRK2/BRK3 and the load CVs.
#define SPEED_TYPE_COUNT                 3

uint8_t speedType(void);

// 8-char padded display label for a TYPE (fills the LCD row); clamps an out-of-range type to the
// default. The PC tooling keeps its own type-name map.
const char *speedTypeName(uint8_t type);

// Map a 1-based SPEED_CONFIG_SCREEN position to a SPEED_ITEM_*, for the current TYPE and the ADV
// FUNC state (which reveals the four correction tunables). Returns SPEED_ITEM_COUNT once pos is
// past the last visible item - the menu-wrap sentinel.
uint8_t speedItemAt(uint8_t pos, uint8_t advFunc);

// Call when TYPE changes (old -> new): a model parameter the new family does not use is set inert
// (so the model ignores it and the export hides it); one the new family gains that the old lacked
// is set to its default (a fresh start, not the stale value from the last time this family was
// picked). V5DCC <-> V5MULT is a no-op - identical parameter sets.
void speedResetModel(uint8_t oldType, uint8_t newType);

// Force the current TYPE's inapplicable model parameters inert, regardless of what is stored - a
// one-shot guard called from readConfig() so a hand-edited or pre-split EEPROM cannot feed the
// model a value the current family should ignore.
void speedApplyTypeInert(void);

void updateSpeed10Hz(uint8_t commandedSpeedStep, uint8_t brake1Active, uint8_t brake2Active,
                      uint8_t brake3Active, uint8_t emergencyActive, uint8_t watchedFunctionActive,
                      uint8_t oploadActive, uint8_t prloadActive, uint8_t holdActive);
void resetSpeed(void);
void printSpeed(void);

uint8_t speedGet(uint8_t item);
void    speedSet(uint8_t item, uint8_t value);

#endif
