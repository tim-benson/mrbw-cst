/*************************************************************************
Title:    Reference-trace test for the air-brake model (AIRBRAKE)
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-pressure-test/test_pressure.c
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

/*
 * WHAT THIS IS
 *   A host-compiled harness (native cc, not avr-gcc) that drives the real
 *   updateBrake10Hz() air-brake model in cst-pressure.c through a fixed set of
 *   scenarios and records the model state plus every output accessor at every
 *   10 Hz tick into a plain-text "trace", one file per scenario. `make
 *   pressuretest` diffs the freshly generated traces against the checked-in
 *   copies under reference/. Any diff means the model output moved - either an
 *   intended change (regenerate with `make pressuretest-accept`) or a regression
 *   to investigate. The technique is snapshot / characterization / golden-master
 *   testing; "reference trace" is the term used in this tree.
 *
 * HOW IT COMPILES ON THE HOST
 *   cst-pressure.h includes only <stdint.h>; cst-pressure.c adds only <stdlib.h>
 *   (for rand() in initAirBrake()). No lcd.h, no avr headers, no util/atomic.h -
 *   the model is main-loop-only, no ATOMIC_BLOCK - so this file just #includes
 *   ../cst-pressure.c directly, with no include-path shims at all (unlike the
 *   scale-speed harness, which needs stubs/avr/pgmspace.h for src/lcd.h).
 *
 * AVR-vs-host arithmetic fidelity
 *   updateBrake10Hz() and every accessor use uint32_t / UL literals throughout,
 *   with no bare `int` intermediates - so the model is width-identical between
 *   AVR 16-bit int and host 32-bit int by construction, a stronger guarantee
 *   than the scale-speed model. The one host/AVR divergence point is the
 *   (uint8_t)rand() gauge jitter initAirBrake() applies to bpMilliPsi/mrMilliPsi;
 *   modelReset() below overwrites those two right after, so the jitter never
 *   reaches a trace.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The model under test, pulled in whole so its file-statics (bpMilliPsi,
 * mrMilliPsi, syncDrawMilliPsi, the latches) are visible here. */
#include "../cst-pressure.c"

/* ---- scenario harness ------------------------------------------------------- */

static const char *g_outdir;
static FILE        *g_tf;
static int          g_traceCount;

/* One tick's worth of updateBrake10Hz() inputs. atRest is mrbw-cst.c's
 * per-BRK-TYPE "independent brake genuinely at rest" check (see CLAUDE.md "Brake
 * logic") - driven directly here; deriving it is out of scope for a
 * cst-pressure.c unit test. */
typedef struct
{
	uint8_t lever;    /* leverPcnt 0..100 */
	uint8_t atRest;   /* independentBrakeAtRest */
	uint8_t estp;     /* emergencyBrakeEnabled (the BRK ESTP option) */
} Inputs;

/* Reset AIRBRAKE CFG to the shipped defaults - mirrors the airbrakeCfg[]
 * initializer in cst-pressure.c. */
static void cfgReset(void)
{
	airbrakeSet(AIRBRAKE_CHARGED,     AIRBRAKE_CHARGED_DEFAULT);
	airbrakeSet(AIRBRAKE_MR_LOAD,     AIRBRAKE_MR_LOAD_DEFAULT);
	airbrakeSet(AIRBRAKE_MR_CUTIN,    AIRBRAKE_MR_CUTIN_DEFAULT);
	airbrakeSet(AIRBRAKE_MR_CUTOUT,   AIRBRAKE_MR_CUTOUT_DEFAULT);
	airbrakeSet(AIRBRAKE_CHARGE_RATE, AIRBRAKE_CHARGE_RATE_DEFAULT);
	airbrakeSet(AIRBRAKE_LEAK_RATE,   AIRBRAKE_LEAK_RATE_DEFAULT);
	airbrakeSet(AIRBRAKE_PUMP_RATE,   AIRBRAKE_PUMP_RATE_DEFAULT);
	airbrakeSet(AIRBRAKE_DISPLAY,     AIRBRAKE_DISPLAY_DEFAULT);
	airbrakeSet(AIRBRAKE_COMP_MODE,   AIRBRAKE_COMP_MODE_DEFAULT);
}

/* Deterministic model reset. initAirBrake() zeroes the eight latch/timer/credit
 * vars (which is what we want) but seeds bpMilliPsi/mrMilliPsi with a
 * (uint8_t)rand() gauge jitter; pin them to exact values right after so the
 * traces do not depend on the host rand(). Call after cfgReset() + any per-
 * scenario airbrakeSet() overrides. */
static void modelReset(void)
{
	initAirBrake();
	bpMilliPsi = (uint32_t)airbrakeGet(AIRBRAKE_CHARGED)   * 1000UL;
	mrMilliPsi = (uint32_t)airbrakeGet(AIRBRAKE_MR_CUTOUT) * 1000UL - 1000UL;
}

static void traceOpen(const char *name, const char *cfgLine, const char *inputsLine)
{
	char path[512];
	snprintf(path, sizeof path, "%s/%s.txt", g_outdir, name);
	g_tf = fopen(path, "w");
	if (!g_tf)
	{
		perror(path);
		exit(2);
	}
	fprintf(g_tf, "# scenario: %s\n", name);
	fprintf(g_tf, "# config:   %s\n", cfgLine);
	fprintf(g_tf, "# inputs:   %s\n", inputsLine);
	fprintf(g_tf, "#\n");
	fprintf(g_tf, "# flags: A applied  S brk-set-pulse  C compressor  P pending-deep  X run-is-deep  E emergency\n");
	fprintf(g_tf, "#  tick  lvr R E     bp_mPSI     mr_mPSI  sync_mPSI  flags(ASCPXE)   BP   MR\n");
	g_traceCount++;
}

static void traceTick(int t, const Inputs *in)
{
	updateBrake10Hz(in->lever, in->atRest, in->estp);

	char flags[7];
	flags[0] = airBrakeReleased()            ? '-' : 'A';   /* applied = not released */
	flags[1] = airBrakeSetPulse()            ? 'S' : '-';
	flags[2] = airCompressorOn()             ? 'C' : '-';
	flags[3] = airCompressorPendingRelease() ? 'P' : '-';
	flags[4] = airCompressorReleaseRun()     ? 'X' : '-';
	flags[5] = airEmergencyActive()          ? 'E' : '-';
	flags[6] = '\0';

	fprintf(g_tf, "  %5d  %3u %u %u  %10lu  %10lu  %9lu  %s        %3u  %3u\n",
	        t, in->lever, in->atRest, in->estp,
	        (unsigned long)bpMilliPsi, (unsigned long)mrMilliPsi, (unsigned long)syncDrawMilliPsi,
	        flags, airBrakePipePsi(), airMainResPsi());
}

static void traceClose(void)
{
	fclose(g_tf);
	g_tf = NULL;
}

/* Run `in` unchanged for `ticks` ticks starting at tick `t0`; returns the next tick index. */
static int runPhase(int t0, int ticks, const Inputs *in)
{
	for (int i = 0; i < ticks; i++)
		traceTick(t0 + i, in);
	return t0 + ticks;
}

/* ---- scenarios ------------------------------------------------------------- */

/* No brake, held long enough for a full governor cycle: the reservoir sawtooths
 * between MR LOW and MR HIGH, the compressor cuts in (~t1124) and out (~t1363),
 * and the rounded whole-PSI readouts dwell at the setpoints (airMainResPsi()
 * rounds, so MR reads 130/140 for a real slice of the cycle, not flashing past).
 * No release ever happens, so the deep-run (X) and pending (P) flags stay clear. */
static void sc_idle_governor(void)
{
	cfgReset();
	modelReset();
	traceOpen("idle_governor",
	          "defaults - BP CHARGE 90, MR 130/140, LEAK 5, PUMP 30, RECHARGE 180, MR LOAD 35, NORMAL",
	          "lever 0, atRest 1, estp 0 - held 1700 ticks (one full compressor cycle)");
	Inputs in = { 0, 1, 0 };
	runPhase(0, 1700, &in);
	traceClose();
}

/* The initial full-service reduction and its release. Lever to 70 % (=
 * AIRBRAKE_FULLSVC_PCNT) vents the pipe to BP CHARGE - FULLSVC at 6 PSI/s with a
 * ~1 s BRK SET hiss pulse; the applied latch sets (BRAKE_REL_FN drops); back to
 * rest and the pipe tapers back to charged while the recharge draws the
 * reservoir (MR LOAD) hard enough to kick the compressor. */
static void sc_initial_full_service(void)
{
	cfgReset();
	modelReset();
	traceOpen("initial_full_service",
	          "defaults",
	          "20 ticks at rest, lever 70 (atRest 0) for 120, then lever 0 (atRest 1) - vent, hold, recharge");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 20, &in);
	in.lever = 70; in.atRest = 0;
	t = runPhase(t, 120, &in);
	in.lever = 0; in.atRest = 1;
	runPhase(t, 700, &in);
	traceClose();
}

/* Lever just into the service zone: the computed reduction (a few PSI) is
 * floored to AIRBRAKE_MIN_REDUCTION_PSI - a 26L cannot make a smaller one - so
 * the pipe drops exactly 7 PSI, no less. */
static void sc_min_reduction_floor(void)
{
	cfgReset();
	modelReset();
	traceOpen("min_reduction_floor",
	          "defaults",
	          "15 ticks at rest, then lever 15 (atRest 0) held - reduction floors at 7 PSI");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	in.lever = 15; in.atRest = 0;
	runPhase(t, 100, &in);
	traceClose();
}

/* Graduated service: lever stepped 20 -> 40 -> 60 %. Each further reduction is a
 * fresh venting episode, so each fires its own BRK SET pulse and the pipe vents
 * down another step. */
static void sc_graduated_service(void)
{
	cfgReset();
	modelReset();
	traceOpen("graduated_service",
	          "defaults",
	          "lever 20, then 40, then 60 (atRest 0), 80 ticks each - a BRK SET pulse per step");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	in.atRest = 0;
	in.lever = 20; t = runPhase(t, 80, &in);
	in.lever = 40; t = runPhase(t, 80, &in);
	in.lever = 60; runPhase(t, 80, &in);
	traceClose();
}

/* Lapping: past AIRBRAKE_FULLSVC_PCNT (70 %) the handle laps at full service -
 * the reduction clamps to FULLSVC, so 70 -> 85 -> 90 % holds the pipe flat with
 * no further vent and no new BRK SET pulse. */
static void sc_lapping(void)
{
	cfgReset();
	modelReset();
	traceOpen("lapping",
	          "defaults",
	          "lever 70, then 85, then 90 (atRest 0) - pipe holds at charged - FULLSVC past 70 %");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	in.atRest = 0;
	in.lever = 70; t = runPhase(t, 70, &in);
	in.lever = 85; t = runPhase(t, 70, &in);
	in.lever = 90; runPhase(t, 70, &in);
	traceClose();
}

/* Emergency with BRK ESTP on: lever past AIRBRAKE_EMERG_PCNT (95 %) latches
 * emergencyActive and dumps the pipe to 0 at 25 PSI/s. The latch is hysteretic -
 * lever back to 88 % (>= 5 below 95) releases it - and then the pipe recharges
 * from ~0 (a much bigger gap than a service reduction, so faster at first). */
static void sc_emergency_estp_on(void)
{
	cfgReset();
	modelReset();
	traceOpen("emergency_estp_on",
	          "defaults, BRK ESTP on (estp 1)",
	          "lever 97 (atRest 0) for 110, lever 88 for 80 (releases the latch), then lever 0 (atRest 1) - settle");
	Inputs in = { 0, 1, 1 };
	int t = runPhase(0, 15, &in);
	in.lever = 97; in.atRest = 0;
	t = runPhase(t, 110, &in);
	in.lever = 88;
	t = runPhase(t, 80, &in);
	in.lever = 0; in.atRest = 1;
	runPhase(t, 800, &in);
	traceClose();
}

/* Emergency with BRK ESTP off: a 26L automatic brake has no emergency zone
 * without the option - lever to 100 % just laps at full service, emergencyActive
 * never sets, the pipe holds at charged - FULLSVC. */
static void sc_emergency_estp_off(void)
{
	cfgReset();
	modelReset();
	traceOpen("emergency_estp_off",
	          "defaults, BRK ESTP off (estp 0)",
	          "lever 100 (atRest 0) held, then lever 0 (atRest 1) - laps at full service, no emergency");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	in.lever = 100; in.atRest = 0;
	t = runPhase(t, 120, &in);
	in.lever = 0; in.atRest = 1;
	runPhase(t, 150, &in);
	traceClose();
}

/* CONSIST mode: the compressor run right after a full-service release is a
 * "deep"/synchronising event (X - COMPRSR), a plain idle-leak cycle is routine
 * (COMPRSR2). The pending-deep flag (P) tracks the decaying consist-sync credit.
 * A full-service release credits ~9100 mPSI, well past the 8000 mPSI threshold. */
static void sc_consist_deep_release(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_COMP_MODE, AIRBRAKE_COMP_MODE_CONSIST);
	modelReset();
	traceOpen("consist_deep_release",
	          "defaults + COMPMODE CONSIST",
	          "400 ticks idle (no compressor run -> not deep), lever 70 for 60, then lever 0 - release run is deep (X)");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 400, &in);
	in.lever = 70; in.atRest = 0;
	t = runPhase(t, 60, &in);
	in.lever = 0; in.atRest = 1;
	runPhase(t, 900, &in);
	traceClose();
}

/* CONSIST: three 30 % releases within ~25 ticks of each other. Each credits its
 * own reservoir draw once, on the first tick of its recharge, and the credit
 * does not decay while a recharge is delivering, so they stack past
 * AIRBRAKE_SYNC_BAND_PCT - `airCompressorPendingRelease()` (the P flag) sets.
 * (Whether a compressor run actually catches the raised credit at its ON edge is
 * timing-dependent; the P flag is the direct observable of the stacking, and
 * `consist_light_spaced` below is the same releases judged alone.) */
static void sc_consist_light_stacked(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_COMP_MODE, AIRBRAKE_COMP_MODE_CONSIST);
	modelReset();
	traceOpen("consist_light_stacked",
	          "defaults + COMPMODE CONSIST",
	          "three 30 %/25-tick releases back to back, then a settle - credits stack past the threshold (P)");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	for (int k = 0; k < 3; k++)
	{
		in.lever = 30; in.atRest = 0;
		t = runPhase(t, 30, &in);
		in.lever = 0;  in.atRest = 1;
		t = runPhase(t, 25, &in);
	}
	runPhase(t, 650, &in);
	traceClose();
}

/* CONSIST: the same three 30 % releases, now ~400 ticks apart. Between them the
 * pipe fully recharges and the credit decays (LEAK, quiet gaps only), so each is
 * judged alone and none of the compressor runs is classified deep. */
static void sc_consist_light_spaced(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_COMP_MODE, AIRBRAKE_COMP_MODE_CONSIST);
	modelReset();
	traceOpen("consist_light_spaced",
	          "defaults + COMPMODE CONSIST",
	          "three 30 % releases 400 ticks apart - each judged alone, none deep");
	Inputs in = { 0, 1, 0 };
	int t = runPhase(0, 15, &in);
	for (int k = 0; k < 3; k++)
	{
		in.lever = 30; in.atRest = 0;
		t = runPhase(t, 30, &in);
		in.lever = 0;  in.atRest = 1;
		t = runPhase(t, 400, &in);
	}
	traceClose();
}

/* PUMP RATE <= LEAK RATE misconfiguration: updateBrake10Hz() floors the net fill
 * at 10 mPSI/tick so the compressor still (slowly) cuts out instead of sticking
 * on forever. */
static void sc_guard_pump_le_leak(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_PUMP_RATE, 3);
	airbrakeSet(AIRBRAKE_LEAK_RATE, 10);
	modelReset();
	traceOpen("guard_pump_le_leak",
	          "defaults + PUMP RATE 3, LEAK RATE 10 (pump <= leak)",
	          "lever 0, atRest 1 - the net-fill floor still lets the governor cut out");
	Inputs in = { 0, 1, 0 };
	runPhase(0, 1800, &in);
	traceClose();
}

/* MR LOW >= MR HIGH misconfiguration (a plausible fat-finger on two adjacent
 * menu items): updateBrake10Hz() forces MR HIGH = MR LOW + 1 PSI so the two
 * governor comparisons cannot fight every tick - the compressor cycles slowly
 * instead of stuttering at 1-2 Hz. */
static void sc_guard_inverted_band(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_MR_CUTIN, 140);
	airbrakeSet(AIRBRAKE_MR_CUTOUT, 130);
	modelReset();
	traceOpen("guard_inverted_band",
	          "defaults + MR LOW 140, MR HIGH 130 (inverted)",
	          "lever 0, atRest 1 - governed band guarded to 1 PSI, no per-tick stutter");
	Inputs in = { 0, 1, 0 };
	runPhase(0, 600, &in);
	traceClose();
}

/* The tapered recharge's asymptotic tail: from an emergency dump (~0) the pipe
 * refills toward BP CHARGE, and the fixed AIRBRAKE_RECHARGE_TAIL_MPSI crawl plus
 * the `fill > gap` clamp land it on exactly BP CHARGE rather than stalling one
 * LSB short forever. */
static void sc_recharge_tail(void)
{
	cfgReset();
	modelReset();
	traceOpen("recharge_tail",
	          "defaults, BRK ESTP on (estp 1)",
	          "emergency dump (lever 97) for 60, then lever 0 (atRest 1) - pipe recharges to exactly BP CHARGE");
	Inputs in = { 0, 1, 1 };
	int t = runPhase(0, 15, &in);
	in.lever = 97; in.atRest = 0;
	t = runPhase(t, 60, &in);
	in.lever = 0; in.atRest = 1;
	runPhase(t, 900, &in);
	traceClose();
}

/* ---- invariants (asserted in main() as PASS/FAIL, exit non-zero on any fail) ---
 * These drive updateBrake10Hz() directly and check a property the reference
 * files alone cannot express - the same role as the V4==V5MULT check in the
 * scale-speed harness. */

/* milliPSI/tick working rates, matching updateBrake10Hz()'s own conversions. */
static uint32_t cfgLeakRatePerTick(void) { return (uint32_t)airbrakeGet(AIRBRAKE_LEAK_RATE) * 100UL / 60UL; }
static uint32_t cfgPumpRatePerTick(void)
{
	uint32_t p = (uint32_t)airbrakeGet(AIRBRAKE_PUMP_RATE) * 100UL / 60UL;
	uint32_t l = cfgLeakRatePerTick();
	return (p < l + 10UL) ? (l + 10UL) : p;
}

/* 1. The idle governor stays bounded and actually cycles - MR never runs away or
 *    sticks, and the compressor toggles both ways at least twice over a long run. */
static int inv_idle_governor_bounded(void)
{
	cfgReset();
	modelReset();
	uint32_t cutIn  = (uint32_t)airbrakeGet(AIRBRAKE_MR_CUTIN)  * 1000UL;
	uint32_t cutOut = (uint32_t)airbrakeGet(AIRBRAKE_MR_CUTOUT) * 1000UL;
	uint32_t lo = cutIn - 2000UL;
	uint32_t hi = cutOut + cfgPumpRatePerTick() + 1000UL;
	int onEdges = 0, offEdges = 0, prev = airCompressorOn(), ok = 1;
	for (int i = 0; i < 6000; i++)
	{
		updateBrake10Hz(0, 1, 0);
		if (bpMilliPsi != (uint32_t)airbrakeGet(AIRBRAKE_CHARGED) * 1000UL)
			ok = 0;                                 /* no brake -> pipe stays charged */
		if (mrMilliPsi < lo || mrMilliPsi > hi)
			ok = 0;
		int now = airCompressorOn();
		if (now && !prev) onEdges++;
		if (!now && prev) offEdges++;
		prev = now;
	}
	return ok && onEdges >= 2 && offEdges >= 2;
}

/* 2. With MR LOW >= MR HIGH the compressor must not stutter every tick - the
 *    band guard forces a real (slow) cycle. The unguarded bug toggles at ~1-2 Hz
 *    (an edge every few ticks, ~hundreds of edges over this run); the guarded
 *    1-PSI band gives a ~150-tick cycle, a handful of edges over 900 ticks. */
static int inv_inverted_band_no_stutter(void)
{
	cfgReset();
	airbrakeSet(AIRBRAKE_MR_CUTIN, 140);
	airbrakeSet(AIRBRAKE_MR_CUTOUT, 130);
	modelReset();
	int edges = 0, prev = airCompressorOn();
	for (int i = 0; i < 900; i++)
	{
		updateBrake10Hz(0, 1, 0);
		int now = airCompressorOn();
		if (now != prev) edges++;
		prev = now;
	}
	return edges <= 12;
}

/* 3. After an emergency dump and release, the brake pipe recharges to exactly
 *    BP CHARGE within a bounded time - the tail crawl completes, the clamp lands
 *    it dead-on, it does not stall short. */
static int inv_recharge_completes(void)
{
	cfgReset();
	modelReset();
	for (int i = 0; i < 60; i++) updateBrake10Hz(97, 0, 1);   /* emergency dump */
	uint32_t charged = (uint32_t)airbrakeGet(AIRBRAKE_CHARGED) * 1000UL;
	int reached = 0;
	for (int i = 0; i < 4000; i++)
	{
		updateBrake10Hz(0, 1, 0);
		if (bpMilliPsi == charged) { reached = 1; break; }
	}
	return reached && bpMilliPsi == charged;
}

int main(int argc, char **argv)
{
	g_outdir = (argc > 1) ? argv[1] : "out";

	sc_idle_governor();
	sc_initial_full_service();
	sc_min_reduction_floor();
	sc_graduated_service();
	sc_lapping();
	sc_emergency_estp_on();
	sc_emergency_estp_off();
	sc_consist_deep_release();
	sc_consist_light_stacked();
	sc_consist_light_spaced();
	sc_guard_pump_le_leak();
	sc_guard_inverted_band();
	sc_recharge_tail();

	printf("wrote %d reference traces to %s/\n", g_traceCount, g_outdir);

	int i1 = inv_idle_governor_bounded();
	int i2 = inv_inverted_band_no_stutter();
	int i3 = inv_recharge_completes();
	printf("invariant  idle governor bounded + cycles:  %s\n", i1 ? "PASS" : "FAIL");
	printf("invariant  inverted MR band - no stutter:   %s\n", i2 ? "PASS" : "FAIL");
	printf("invariant  brake-pipe recharge completes:   %s\n", i3 ? "PASS" : "FAIL");
	return (i1 && i2 && i3) ? 0 : 1;
}
