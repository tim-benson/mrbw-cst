/*************************************************************************
Title:    Reference-trace test for the scale-speed simulation model
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-speed-test/test_speed.c
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
 *   updateSpeed10Hz() model in cst-speed.c through a fixed set of scenarios and
 *   records simSpeedStepQ8 plus the printSpeed() render at every 10 Hz tick into a
 *   plain-text "trace", one file per scenario. `make speedtest` diffs the freshly
 *   generated traces against the checked-in copies under reference/. Any diff means
 *   the model output moved - either an intended change (regenerate with
 *   `make speedtest-accept`) or a regression to investigate.
 *
 *   The technique is known in the wider world as snapshot / characterization /
 *   golden-master testing; "reference trace" is the term used in this tree.
 *
 * HOW IT COMPILES ON THE HOST
 *   cst-speed.c has no AVR dependency of its own - it includes only "lcd.h" and
 *   "cst-speed.h". This file #includes ../cst-speed.c directly (so the file-static
 *   simSpeedStepQ8, and the model's internal state, are reachable) and supplies host
 *   definitions of the four LCD symbols printSpeed() calls (lcd_puts / lcd_putc /
 *   printDec2Dig / printDec3Dig). The only include-path shim is stubs/avr/pgmspace.h,
 *   needed solely because src/lcd.h pulls in <avr/pgmspace.h>.
 *
 * AVR-vs-host arithmetic fidelity
 *   AVR int is 16-bit; host int is 32-bit. cst-speed.c uses explicit-width types
 *   (uintN_t, int64_t) with casts throughout, so the two platforms agree exactly as
 *   long as every intermediate value stays <= 32767. The one wildcard is
 *   computeDelta() returning 0xFFFF, which happens only when a momentum CV is zero
 *   ("snap instantly"). The scenarios below therefore keep ACCEL / DECEL / BRKn in
 *   realistic non-zero ranges, where the model is provably width-independent, so
 *   these traces are a faithful stand-in for what the firmware computes. Genuine
 *   bit-for-bit agreement with the flashed firmware is separately covered by the
 *   on-hardware drive check that accompanies each SPEED change.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../cst-common.h"   /* SPEED_H_CHAR - the narrow-H CGRAM glyph printSpeed() now writes for
                              * the MPH/KMH unit; the lcd_putc shim below renders it back as 'H' so
                              * the reference traces stay readable and unchanged. */

/* ---- LCD capture shims -------------------------------------------------------
 * printSpeed() is the only part of cst-speed.c that calls out of the file. It uses
 * lcd_puts() plus printDec2Dig()/printDec3Dig(); those two in turn call lcd_putc().
 * Here they append into a per-tick buffer the harness reads back as the "display"
 * column. printDec2Dig()/printDec3Dig() are verbatim copies of src/lcd.c - keep in
 * sync if that file's digit formatting ever changes.
 */
static char    lcdBuf[64];
static uint8_t lcdLen;

static void lcdReset(void) { lcdLen = 0; lcdBuf[0] = '\0'; }

void lcd_putc(char c)
{
	if ((unsigned char)c == SPEED_H_CHAR)
		c = 'H';   /* narrow-H CGRAM glyph -> 'H' on the real LCD's MPH/KMH unit label */
	if (lcdLen < sizeof(lcdBuf) - 1)
	{
		lcdBuf[lcdLen++] = c;
		lcdBuf[lcdLen] = '\0';
	}
}

void lcd_puts(const char *s)
{
	while (*s)
		lcd_putc(*s++);
}

void printDec3Dig(uint16_t val)  /* verbatim from src/lcd.c */
{
	if (val >= 100)
		lcd_putc('0' + ((val/100)%10));
	else
		lcd_putc(' ');

	if (val >= 10)
		lcd_putc('0' + ((val/10)%10));
	else
		lcd_putc(' ');
	lcd_putc('0' + (val%10));
}

void printDec2Dig(uint8_t val)  /* verbatim from src/lcd.c */
{
	if (val >= 10)
		lcd_putc('0' + ((val/10)%10));
	else
		lcd_putc(' ');
	lcd_putc('0' + (val%10));
}

/* The model under test, pulled in whole so its file-statics are visible here. */
#include "../cst-speed.c"

/* ---- scenario harness ------------------------------------------------------- */

static const char *g_outdir;
static FILE        *g_tf;
static int          g_traceCount;

/* One tick's worth of updateSpeed10Hz() inputs. */
typedef struct
{
	uint8_t cmd;      /* commandedSpeedStep 0..126 */
	uint8_t b1, b2, b3;
	uint8_t estop;    /* emergencyActive */
	uint8_t stopfn;   /* watchedFunctionActive (STOPFN) */
	uint8_t opload, prload;
	uint8_t hold;     /* holdActive (HOLDFN / Drive Hold) */
} Inputs;

/* Reset SPEED CFG to the shipped defaults - mirrors the speedCfg[] initializer in
 * cst-speed.c (resetSpeed() clears model state but deliberately never touches config). */
static void cfgDefaults(void)
{
	speedSet(SPEED_ITEM_ACCEL,           MOMENTUM_ACCEL_CV3_DEFAULT);
	speedSet(SPEED_ITEM_DECEL,           MOMENTUM_DECEL_CV4_DEFAULT);
	speedSet(SPEED_ITEM_BRAKE1,          MOMENTUM_BRAKE1_CV179_DEFAULT);
	speedSet(SPEED_ITEM_BRAKE2,          MOMENTUM_BRAKE2_CV180_DEFAULT);
	speedSet(SPEED_ITEM_BRAKE3,          MOMENTUM_BRAKE3_CV181_DEFAULT);
	speedSet(SPEED_ITEM_START_DELAY,     MOMENTUM_START_DELAY_DEFAULT);
	speedSet(SPEED_ITEM_MAX_MPH,         SPEED_MAX_MPH_DEFAULT);
	speedSet(SPEED_ITEM_UNIT,            SPEED_UNIT_KMH_DEFAULT);
	speedSet(SPEED_ITEM_HOLD_FN,         SPEED_HOLD_WATCH_FN_DEFAULT);
	speedSet(SPEED_ITEM_STOP_FN,         SPEED_STOP_WATCH_FN_DEFAULT);
	speedSet(SPEED_ITEM_OPLOAD,          SPEED_OPLOAD_DEFAULT);
	speedSet(SPEED_ITEM_OPLOAD_FN,       SPEED_OPLOAD_FN_DEFAULT);
	speedSet(SPEED_ITEM_PRLOAD,          SPEED_PRLOAD_DEFAULT);
	speedSet(SPEED_ITEM_PRLOAD_FN,       SPEED_PRLOAD_FN_DEFAULT);
	speedSet(SPEED_ITEM_TYPE,            SPEED_TYPE_DEFAULT);
	speedSet(SPEED_ITEM_ACCEL_PCT,       SPEED_ACCEL_PCT_DEFAULT);
	speedSet(SPEED_ITEM_ACCEL_TARGET,    SPEED_ACCEL_TARGET_DEFAULT);
	speedSet(SPEED_ITEM_DECEL_PCT,       SPEED_DECEL_PCT_DEFAULT);
	speedSet(SPEED_ITEM_DECEL_THRESHOLD, SPEED_DECEL_THRESHOLD_DEFAULT);
	speedSet(SPEED_ITEM_ACCEL_ADJ,       SPEED_ACCEL_ADJ_DEFAULT);
	speedSet(SPEED_ITEM_DECEL_ADJ,       SPEED_DECEL_ADJ_DEFAULT);
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
	fprintf(g_tf, "#  tick  cmd  flags(EHS123OP)      q8   display\n");
	g_traceCount++;
}

static void traceTick(int t, const Inputs *in)
{
	updateSpeed10Hz(in->cmd, in->b1, in->b2, in->b3, in->estop, in->stopfn,
	                in->opload, in->prload, in->hold);

	lcdReset();
	printSpeed();

	char flags[9];
	flags[0] = in->estop  ? 'E' : '-';
	flags[1] = in->hold   ? 'H' : '-';
	flags[2] = in->stopfn ? 'S' : '-';
	flags[3] = in->b1     ? '1' : '-';
	flags[4] = in->b2     ? '2' : '-';
	flags[5] = in->b3     ? '3' : '-';
	flags[6] = in->opload ? 'O' : '-';
	flags[7] = in->prload ? 'P' : '-';
	flags[8] = '\0';

	fprintf(g_tf, "  %5d  %3u  %s        %6u   \"%s\"\n",
	        t, in->cmd, flags, (unsigned)simSpeedStepQ8, lcdBuf);
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

static void sc_standing_start_full(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("standing_start_full",
	          "defaults - ACCEL 60, DECEL 230, DELAY 13, MAXSPEED 50, TYPE V5DCC, MPH",
	          "cmd=126 held from tick 0; no brake / hold / estop");
	Inputs in = {0};
	in.cmd = 126;
	runPhase(0, 600, &in);
	traceClose();
}

static void sc_standing_start_low_notch(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("standing_start_low_notch",
	          "defaults",
	          "cmd=20 held from tick 0 (ramp output clamps to the low target)");
	Inputs in = {0};
	in.cmd = 20;
	runPhase(0, 200, &in);
	traceClose();
}

static void sc_coast_to_stop(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_DECEL, 120);
	resetSpeed();
	traceOpen("coast_to_stop",
	          "DECEL 120, otherwise defaults",
	          "cmd=30 to tick 150 (settle), then cmd=0 - coast down (DECPCT/DECTHR lag)");
	Inputs in = {0};
	in.cmd = 30;
	int t = runPhase(0, 150, &in);
	in.cmd = 0;
	runPhase(t, 400, &in);
	traceClose();
}

static void sc_brake1_from_cruise(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("brake1_from_cruise",
	          "defaults - BRK1 130",
	          "cmd=45 to tick 150 (settle), then Brake1 held (cmd stays 45)");
	Inputs in = {0};
	in.cmd = 45;
	int t = runPhase(0, 150, &in);
	in.b1 = 1;
	runPhase(t, 350, &in);
	traceClose();
}

static void sc_brake12_strength_step(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("brake12_strength_step",
	          "defaults - BRK1 130, BRK2 70",
	          "cmd=45 settle; Brake1 at tick 150; Brake1+2 at tick 210 (strength rises mid-brake)");
	Inputs in = {0};
	in.cmd = 45;
	int t = runPhase(0, 150, &in);
	in.b1 = 1;
	t = runPhase(t, 60, &in);
	in.b2 = 1;
	runPhase(t, 250, &in);
	traceClose();
}

static void sc_brake123_snap(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("brake123_snap",
	          "defaults - BRK1+2+3 = 300, capped at 255 (near-instant stop)",
	          "cmd=45 to tick 150 (settle), then Brake1+2+3 held");
	Inputs in = {0};
	in.cmd = 45;
	int t = runPhase(0, 150, &in);
	in.b1 = in.b2 = in.b3 = 1;
	runPhase(t, 100, &in);
	traceClose();
}

static void sc_estop_midramp(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("estop_midramp",
	          "defaults",
	          "cmd=126 from stop; emergency asserted ticks [40,80); released after "
	          "(re-arms Start Delay + fresh ramp)");
	Inputs in = {0};
	in.cmd = 126;
	int t = runPhase(0, 40, &in);
	in.estop = 1;
	t = runPhase(t, 40, &in);
	in.estop = 0;
	runPhase(t, 140, &in);
	traceClose();
}

static void sc_stopfn_snap_release(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("stopfn_snap_release",
	          "defaults",
	          "cmd=60 settle; STOPFN watched-function active ticks [220,270); released after");
	Inputs in = {0};
	in.cmd = 60;
	int t = runPhase(0, 220, &in);
	in.stopfn = 1;
	t = runPhase(t, 50, &in);
	in.stopfn = 0;
	runPhase(t, 150, &in);
	traceClose();
}

static void sc_hold_freeze_resume(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("hold_freeze_resume",
	          "defaults",
	          "cmd=55; HOLDFN active ticks [150,210) - model frozen; released after");
	Inputs in = {0};
	in.cmd = 55;
	int t = runPhase(0, 150, &in);
	in.hold = 1;
	t = runPhase(t, 60, &in);
	in.hold = 0;
	runPhase(t, 170, &in);
	traceClose();
}

static void sc_hold_edge_skips_delay(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("hold_edge_skips_delay",
	          "defaults - DELAY 13",
	          "from stop: HOLD held ticks [0,30) with cmd rising to 40 during the hold; "
	          "HOLD released at tick 30 with cmd!=0 -> Start Delay skipped");
	Inputs in = {0};
	in.hold = 1;
	in.cmd = 0;
	int t = runPhase(0, 15, &in);
	in.cmd = 40;               /* "revved" while held */
	t = runPhase(t, 15, &in);
	in.hold = 0;               /* falling edge, cmd != 0 */
	runPhase(t, 120, &in);
	traceClose();
}

static void sc_start_delay_long(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_START_DELAY, 40);   /* 40 * 0.25s = 10s = 100 ticks held at 0 */
	resetSpeed();
	traceOpen("start_delay_long",
	          "DELAY 40 (10s spool-up), otherwise defaults",
	          "cmd=35 from stop - holds at 0 through the delay, then ramps");
	Inputs in = {0};
	in.cmd = 35;
	runPhase(0, 240, &in);
	traceClose();
}

static void sc_opload_slows_accel(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_OPLOAD, 200);       /* > 128 -> scales ACCEL/DECEL up (slower) */
	resetSpeed();
	traceOpen("opload_slows_accel",
	          "OPLOAD 200, OPLOADFN active throughout",
	          "cmd=100 from stop with Optional Load engaged");
	Inputs in = {0};
	in.cmd = 100;
	in.opload = 1;
	runPhase(0, 340, &in);
	traceClose();
}

static void sc_prload_wins(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_OPLOAD, 200);
	speedSet(SPEED_ITEM_PRLOAD, 64);        /* < 128 -> faster; Primary wins when both active */
	resetSpeed();
	traceOpen("prload_wins",
	          "OPLOAD 200 + PRLOAD 64, both watched functions active",
	          "cmd=100 from stop - Primary Load (64) overrides Optional Load (200)");
	Inputs in = {0};
	in.cmd = 100;
	in.opload = 1;
	in.prload = 1;
	runPhase(0, 340, &in);
	traceClose();
}

/* The reported hardware failure: a heavy Primary Load from a dead stop. Before the ACCPCT/ACCTGT fix
 * the display ran several seconds ahead of the locomotive right through the standing-start ramp - the
 * head start scaled with the load-stretched accel time, and solveRampR0()'s absolute time-to-0.5mph
 * target injected an r0 bulge that grew with rampT. PRLOAD 255 is also the value that only became
 * storable with the EEPROM layout -> 7 raw read (the top of CV104's native 0-255 range). */
static void sc_prload_heavy_standing_start(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_PRLOAD, 255);       /* CV104 at its 255 max - ~2x the accel time */
	resetSpeed();
	traceOpen("prload_heavy_standing_start",
	          "PRLOAD 255, PRLOADFN active throughout - otherwise defaults (ACCEL 60, MAXSPEED 50, V5DCC)",
	          "cmd=126 from a dead stop with Primary Load engaged");
	Inputs in = {0};
	in.cmd = 126;
	in.prload = 1;
	runPhase(0, 500, &in);
	traceClose();
}

/* The light-load counterpart of the scenario above. A load below 128 shortens the accel time, and the
 * ACCPCT head start shortens with it - holding the head start at its full unloaded value here left the
 * display reading about 1mph high on hardware at OPLOAD 80, and collapsed the ramp outright at very
 * light loads. Covers the regime no other trace reaches: prload_wins also uses a light load but is
 * about which of the two load CVs wins, not the standing-start shape. */
static void sc_opload_light_standing_start(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_OPLOAD, 80);        /* < 128 - hardware-checked against the locomotive */
	resetSpeed();
	traceOpen("opload_light_standing_start",
	          "OPLOAD 80, OPLOADFN active throughout - otherwise defaults (ACCEL 60, MAXSPEED 50, V5DCC)",
	          "cmd=126 from a dead stop with Optional Load engaged");
	Inputs in = {0};
	in.cmd = 126;
	in.opload = 1;
	runPhase(0, 400, &in);
	traceClose();
}

static void sc_accel_cv30(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL, 30);
	resetSpeed();
	traceOpen("accel_cv30", "ACCEL 30, otherwise defaults", "cmd=100 from stop");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 300, &in);
	traceClose();
}

static void sc_accel_cv120(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL, 120);
	resetSpeed();
	traceOpen("accel_cv120", "ACCEL 120, otherwise defaults", "cmd=100 from stop");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 400, &in);
	traceClose();
}

static void sc_decel_cv230(void)
{
	cfgDefaults();
	resetSpeed();
	traceOpen("decel_cv230",
	          "DECEL 230 (default upper bound of the validated range)",
	          "cmd=22 to tick 120 (settle), then cmd=0 - slow coast to stop");
	Inputs in = {0};
	in.cmd = 22;
	int t = runPhase(0, 120, &in);
	in.cmd = 0;
	runPhase(t, 480, &in);
	traceClose();
}

// DECEL 255 sits at the momentum-register ceiling (SPEED_CEIL_FADE_HI), so ceilFadeNum() is 0 and
// the DECPCT deceleration-lag correction is faded fully out - the coast is a pure linear
// ticksToCross(255) ramp, matching a real ESU decoder (which goes linear at the ceiling). Compared
// against the pre-fade reference this reaches q8 = 0 later (~1.7s in this scenario), since the
// "subtract ticks" correction no longer speeds the display toward 0.
static void sc_decel_cv255(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_DECEL, 255);
	resetSpeed();
	traceOpen("decel_cv255",
	          "DECEL 255 (V5DCC) - at the momentum ceiling, DECPCT faded to 0",
	          "cmd=22 to tick 120 (settle), then cmd=0 - pure linear coast to stop");
	Inputs in = {0};
	in.cmd = 22;
	int t = runPhase(0, 120, &in);
	in.cmd = 0;
	runPhase(t, 560, &in);
	traceClose();
}

// ACCEL 255 sits at the momentum-register ceiling (SPEED_CEIL_FADE_HI), so ceilFadeNum() is 0 and
// the standing-start cubic ramp is blended fully to a plain linear climb (rampP = rampQ = 0,
// rampR0 = v). The onset is a steady ~14 q8/tick crawl with no S-curve and no dwell near 15mph -
// the pre-fade reference showed a fast climb to ~14mph then a long flat hold. Runs long enough
// (760 ticks) to cross rampT (~614) and confirm the seamless hand-off to the plain-rate climb.
static void sc_accel_cv255(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL, 255);
	resetSpeed();
	traceOpen("accel_cv255",
	          "ACCEL 255 (V5DCC) - at the momentum ceiling, ramp linearized",
	          "cmd=100 from stop - pure linear standing-start crawl through the rampT hand-off");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 760, &in);
	traceClose();
}

// Effective ACCEL 242 (raw 242, no adjust) - mid fade band. ceilFadeNum(242) = 16*(255-242)/25 = 8,
// so the cubic ramp is a 50/50 blend of the calibrated curve and the linear v*tau climb: a gentler
// onset than the default, with the high-ACCEL dwell roughly halved. Below-230 scenarios are
// untouched; this is the only trace exercising 0 < f < DEN on the accel side.
static void sc_accel_ceil_fade_mid(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL, 242);
	resetSpeed();
	traceOpen("accel_ceil_fade_mid",
	          "ACCEL 242 (V5DCC) - mid ceiling-fade band, cubic 50% blended to linear",
	          "cmd=100 from stop");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 700, &in);
	traceClose();
}

// Effective DECEL 242 (raw 242, no adjust) - mid fade band, ceilFadeNum(242) = 8, so DECPCT is
// halved (22 -> 11) for the steadyStopExtraMs capture. Settles at a higher speed (cmd=40) than
// decel_cv255 so the speed-scaled correction, and the difference the fade makes to it, are both
// clearly visible in the trace.
static void sc_decel_ceil_fade_mid(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_DECEL, 242);
	resetSpeed();
	traceOpen("decel_ceil_fade_mid",
	          "DECEL 242 (V5DCC) - mid ceiling-fade band, DECPCT halved",
	          "cmd=40 to tick 150 (settle), then cmd=0 - coast to stop");
	Inputs in = {0};
	in.cmd = 40;
	int t = runPhase(0, 150, &in);
	in.cmd = 0;
	runPhase(t, 520, &in);
	traceClose();
}

// Raw ACCEL 200 + ACCELADJ +55 -> effective ACCEL 255 (speedEffAccelCV()), so the ramp is fully
// linearized even though the base CV is only 200. Proves the ceiling fade keys off the effective
// CV, not the stored ACCEL byte - a decoder with a large CV23 crosses the ceiling early.
static void sc_accel_adj_over_ceiling(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL, 200);
	speedSet(SPEED_ITEM_ACCEL_ADJ, 55);          /* +55 -> effective 255 */
	resetSpeed();
	traceOpen("accel_adj_over_ceiling",
	          "ACCEL 200 + ACCELADJ +55 (effective 255) - ramp linearized via the adjust",
	          "cmd=100 from stop - pure linear crawl");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 640, &in);
	traceClose();
}

static void sc_maxspeed_120_kmh(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_MAX_MPH, 120);
	speedSet(SPEED_ITEM_UNIT, SPEED_UNIT_KMH);
	resetSpeed();
	traceOpen("maxspeed_120_kmh",
	          "MAXSPEED 120, UNIT KMH - exercises printSpeed() 3-digit + km/h conversion",
	          "cmd=126 from stop");
	Inputs in = {0};
	in.cmd = 126;
	runPhase(0, 560, &in);
	traceClose();
}

static void sc_type_v5dcc(void)
{
	cfgDefaults();   /* TYPE defaults to V5DCC (0.896 multiplier) */
	resetSpeed();
	traceOpen("type_v5dcc",
	          "TYPE V5DCC (0.896 multiplier), otherwise defaults",
	          "cmd=60 to tick 180 (settle), then Brake1 held");
	Inputs in = {0};
	in.cmd = 60;
	int t = runPhase(0, 180, &in);
	in.b1 = 1;
	runPhase(t, 320, &in);
	traceClose();
}

static void sc_type_v5mult(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_TYPE, SPEED_TYPE_V5MULT);   /* 0.25 multiplier - ~3.6x faster */
	resetSpeed();
	traceOpen("type_v5mult",
	          "TYPE V5MULT (0.25 multiplier), otherwise defaults",
	          "cmd=60 to tick 180 (settle), then Brake1 held - compare against type_v5dcc");
	Inputs in = {0};
	in.cmd = 60;
	int t = runPhase(0, 180, &in);
	in.b1 = 1;
	runPhase(t, 320, &in);
	traceClose();
}

/* V4 shares the V5 MultiProtocol model and multiplier; speedResetModel() forces the parameters it
 * drops (BRK2/BRK3, the load CVs) inert. With Brake1 alone this trace must come out byte-identical to
 * type_v5mult - main() asserts exactly that. */
static void sc_type_v4(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_TYPE, SPEED_TYPE_V4);
	speedResetModel(SPEED_TYPE_V5DCC, SPEED_TYPE_V4);   /* BRK2/BRK3 -> 0, loads -> neutral */
	resetSpeed();
	traceOpen("type_v4",
	          "TYPE V4 (V5MULT model, BRK2/BRK3 and load CVs inert)",
	          "cmd=60 to tick 180 (settle), then Brake1 held - byte-identical to type_v5mult");
	Inputs in = {0};
	in.cmd = 60;
	int t = runPhase(0, 180, &in);
	in.b1 = 1;
	runPhase(t, 320, &in);
	traceClose();
}

/* Proof that V4 ignores the parameters it drops: Brake1+2+3 all held, Optional Load engaged, and a
 * non-zero ACCELADJ/DECELADJ set, yet the speed column must still track type_v5mult (brakeSum stays BRK1
 * only, applyLoad is a no-op, the effective ACCEL/DECEL stay the base CVs). If the inert-isation
 * regressed, brakeSum would hit the 255 cap and the loco would stop almost at once, or the adjust
 * would visibly bend the ramp. */
static void sc_type_v4_extras_noop(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_TYPE, SPEED_TYPE_V4);
	speedSet(SPEED_ITEM_OPLOAD, 200);                   /* a real value - must still read as inert */
	speedSet(SPEED_ITEM_ACCEL_ADJ, 40);                 /* +40 - must still read as inert on V4 */
	speedSet(SPEED_ITEM_DECEL_ADJ, 0x80 | 20);          /* -20 - ditto */
	speedResetModel(SPEED_TYPE_V5DCC, SPEED_TYPE_V4);
	resetSpeed();
	traceOpen("type_v4_extras_noop",
	          "TYPE V4; OPLOAD 200, ACCELADJ +40, DECELADJ -20 set; Brake1+2+3 + Optional Load asserted",
	          "cmd=60 settle, then Brake1+2+3 held - speed column tracks type_v5mult (extras no-op)");
	Inputs in = {0};
	in.cmd = 60;
	in.opload = 1;
	int t = runPhase(0, 180, &in);
	in.b1 = in.b2 = in.b3 = 1;
	runPhase(t, 320, &in);
	traceClose();
}

/* ACCELADJ / DECELADJ (ESU CV23 / CV24): a signed factor added to the base ACCEL / DECEL CV before the
 * family multiplier and any load scaling. Default 0 is a pass-through - every other trace proves
 * that by staying byte-identical - so these three exercise the non-zero paths. */
static void sc_adjust_accel_pos(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL_ADJ, 40);     /* +40 -> effective ACCEL 100 (slower standing start) */
	resetSpeed();
	traceOpen("adjust_accel_pos",
	          "V5DCC, ACCELADJ +40 (effective ACCEL 60+40=100), otherwise defaults",
	          "cmd=100 from stop - ramp is slower than the default ACCEL 60");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 400, &in);
	traceClose();
}

static void sc_adjust_accel_neg(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_ACCEL_ADJ, 0x80 | 20);   /* -20 -> effective ACCEL 40 (faster standing start) */
	resetSpeed();
	traceOpen("adjust_accel_neg",
	          "V5DCC, ACCELADJ -20 (effective ACCEL 60-20=40), otherwise defaults",
	          "cmd=100 from stop - ramp is faster than the default");
	Inputs in = {0};
	in.cmd = 100;
	runPhase(0, 300, &in);
	traceClose();
}

static void sc_adjust_decel_pos(void)
{
	cfgDefaults();
	speedSet(SPEED_ITEM_DECEL, 120);        /* same base as coast_to_stop, for a direct comparison */
	speedSet(SPEED_ITEM_DECEL_ADJ, 40);     /* +40 -> effective DECEL 160 (slower coast) */
	resetSpeed();
	traceOpen("adjust_decel_pos",
	          "V5DCC, DECEL 120 + DECELADJ +40 (effective DECEL 160), otherwise defaults",
	          "cmd=30 to tick 150 (settle), then cmd=0 - coast is slower than coast_to_stop (DECEL 120)");
	Inputs in = {0};
	in.cmd = 30;
	int t = runPhase(0, 150, &in);
	in.cmd = 0;
	runPhase(t, 450, &in);
	traceClose();
}

/* Walk the data rows (lines not starting with '#') of two generated traces in lockstep. mode 0
 * compares the whole row; mode 1 compares only the q8 speed column (token 4: tick cmd flags q8 ...).
 * Used to assert an invariant the reference files alone cannot express. */
static int tracesAgree(const char *nameA, const char *nameB, int q8Only)
{
	char pa[512], pb[512];
	snprintf(pa, sizeof pa, "%s/%s.txt", g_outdir, nameA);
	snprintf(pb, sizeof pb, "%s/%s.txt", g_outdir, nameB);
	FILE *fa = fopen(pa, "r"), *fb = fopen(pb, "r");
	if (!fa || !fb) { if (fa) fclose(fa); if (fb) fclose(fb); return 0; }
	char la[256], lb[256];
	int ok = 1, rows = 0;
	while (ok)
	{
		char *ra, *rb;
		do { ra = fgets(la, sizeof la, fa); } while (ra && '#' == la[0]);
		do { rb = fgets(lb, sizeof lb, fb); } while (rb && '#' == lb[0]);
		if (!ra && !rb) break;
		if (!ra || !rb) { ok = 0; break; }
		if (q8Only)
		{
			int ta, ca; unsigned qa; char fa8[32];
			int tb, cb; unsigned qb; char fb8[32];
			if (sscanf(la, "%d %d %31s %u", &ta, &ca, fa8, &qa) != 4 ||
			    sscanf(lb, "%d %d %31s %u", &tb, &cb, fb8, &qb) != 4 || qa != qb)
				ok = 0;
		}
		else if (strcmp(la, lb) != 0)
			ok = 0;
		rows++;
	}
	fclose(fa); fclose(fb);
	return ok && rows > 0;
}

/* Ticks by which the display's arrival at the 15mph-equivalent leads a plain linear climb at the same
 * (load-scaled) rate - i.e. the head start ACCPCT actually spends, measured end to end through the real
 * model rather than read back out of rampT. Start Delay is zeroed so it does not offset the arrival
 * tick; PRLOAD carries the load, since Primary wins outright. Returns -1 if the target is never
 * reached. */
static int accpctLeadTicks(uint8_t load)
{
	int t;
	uint16_t maxMph, target, plain, accelTicks;

	cfgDefaults();
	speedSet(SPEED_ITEM_START_DELAY, 0);
	speedSet(SPEED_ITEM_PRLOAD, load);
	resetSpeed();

	maxMph     = speedGet(SPEED_ITEM_MAX_MPH);
	target     = (uint16_t)(((uint32_t)15 * 126 * 256) / maxMph);
	accelTicks = ticksToCross(applyLoad(speedEffAccelCV(), load));
	plain      = (uint16_t)(((uint32_t)target * accelTicks) / (126UL << 8));

	for (t = 1; t <= 4000; t++)
	{
		updateSpeed10Hz(126, 0, 0, 0, 0, 0, 0, 1, 0);
		if (simSpeedStepQ8 >= target)
			return (int)plain - t;
	}
	return -1;
}

int main(int argc, char **argv)
{
	g_outdir = (argc > 1) ? argv[1] : "out";

	sc_standing_start_full();
	sc_standing_start_low_notch();
	sc_coast_to_stop();
	sc_brake1_from_cruise();
	sc_brake12_strength_step();
	sc_brake123_snap();
	sc_estop_midramp();
	sc_stopfn_snap_release();
	sc_hold_freeze_resume();
	sc_hold_edge_skips_delay();
	sc_start_delay_long();
	sc_opload_slows_accel();
	sc_prload_wins();
	sc_prload_heavy_standing_start();
	sc_opload_light_standing_start();
	sc_accel_cv30();
	sc_accel_cv120();
	sc_decel_cv230();
	sc_maxspeed_120_kmh();
	sc_type_v5dcc();
	sc_type_v5mult();
	sc_type_v4();
	sc_type_v4_extras_noop();
	sc_adjust_accel_pos();
	sc_adjust_accel_neg();
	sc_adjust_decel_pos();
	sc_decel_cv255();
	sc_accel_cv255();
	sc_accel_ceil_fade_mid();
	sc_decel_ceil_fade_mid();
	sc_accel_adj_over_ceiling();

	printf("wrote %d reference traces to %s/\n", g_traceCount, g_outdir);

	/* Invariant: V4 runs the V5 MultiProtocol model exactly. type_v4 (Brake1 only) is byte-for-byte
	 * type_v5mult; type_v4_extras_noop (Brake1+2+3 + Optional Load) still tracks its speed column. */
	int inv = tracesAgree("type_v4", "type_v5mult", 0)
	       && tracesAgree("type_v4_extras_noop", "type_v5mult", 1);

	/* speedApplyTypeInert() (the readConfig() guard) neutralises every V4-dropped param no matter what
	 * is stored - BRK2/BRK3 -> 0, ACCELADJ/DECELADJ -> 0, the load CVs -> 128 / OFF. */
	cfgDefaults();
	speedSet(SPEED_ITEM_TYPE, SPEED_TYPE_V4);
	speedSet(SPEED_ITEM_BRAKE2, 90);
	speedSet(SPEED_ITEM_BRAKE3, 200);
	speedSet(SPEED_ITEM_OPLOAD, 200);
	speedSet(SPEED_ITEM_ACCEL_ADJ, 40);
	speedSet(SPEED_ITEM_DECEL_ADJ, 0x80 | 20);
	speedApplyTypeInert();
	int guard = (0 == speedGet(SPEED_ITEM_BRAKE2)) && (0 == speedGet(SPEED_ITEM_BRAKE3))
	         && (128 == speedGet(SPEED_ITEM_OPLOAD))
	         && (0 == speedGet(SPEED_ITEM_ACCEL_ADJ)) && (0 == speedGet(SPEED_ITEM_DECEL_ADJ));

	/* ACCPCT's head start is ASYMMETRIC in the CV103/CV104 load, which is what hardware measurement
	 * against the locomotive found (see cst-speed.c's headstartTicks). Two halves, both exact:
	 *   - A heavy load (>= 128) must not lengthen it: the train breaks away in the same time, the load
	 *     only stretches the ramp that follows. Scaling it up put the readout seconds ahead of the
	 *     locomotive through the whole climb at PRLOAD 254.
	 *   - A light load (< 128) shortens it in proportion: holding it at the full unloaded value left
	 *     the display ~1mph high at OPLOAD 80, and - since the budget is subtracted from a ramp that
	 *     itself shrinks with the load - collapsed rampT to one tick below about OPLOAD 12, jumping
	 *     the readout straight to ~16mph.
	 * A trace diff alone would show either only as noise, hence direct assertions. */
	int neutralLead = accpctLeadTicks(128), loadRule = (neutralLead > 0);
	static const uint8_t heavyLoads[] = { 128, 160, 192, 224, 254, 255 };
	static const uint8_t lightLoads[] = { 8, 16, 32, 48, 64, 80, 96, 112, 120 };
	for (unsigned k = 0; k < sizeof heavyLoads / sizeof heavyLoads[0]; k++)
		if (accpctLeadTicks(heavyLoads[k]) != neutralLead)
			loadRule = 0;
	for (unsigned k = 0; k < sizeof lightLoads / sizeof lightLoads[0]; k++)
	{
		int want = neutralLead * lightLoads[k] / 128;   /* proportional, +/-1 for integer rounding */
		int got  = accpctLeadTicks(lightLoads[k]);
		if (got > want + 1 || got < want - 1 || got > neutralLead)
			loadRule = 0;
	}

	/* The ramp must never degenerate to a single tick at any load the model can actually move under
	 * (an effective CV of 0 is a genuine no-momentum config and legitimately snaps). This is the
	 * direct regression guard for the collapse above. */
	int rampOk = 1;
	for (unsigned load = 2; load <= 255; load++)
	{
		cfgDefaults();
		speedSet(SPEED_ITEM_START_DELAY, 0);
		speedSet(SPEED_ITEM_OPLOAD, (uint8_t)load);
		resetSpeed();
		updateSpeed10Hz(126, 0, 0, 0, 0, 0, 1, 0, 0);
		if (applyLoad(speedEffAccelCV(), (uint8_t)load) > 0 && rampT < 2)
			rampOk = 0;
	}

	printf("invariant  V4 model == V5MULT model:          %s\n", inv ? "PASS" : "FAIL");
	printf("invariant  speedApplyTypeInert() V4:          %s\n", guard ? "PASS" : "FAIL");
	printf("invariant  ACCPCT head start vs load:         %s\n", loadRule ? "PASS" : "FAIL");
	printf("invariant  ramp never collapses at any load:  %s\n", rampOk ? "PASS" : "FAIL");
	return (inv && guard && loadRule && rampOk) ? 0 : 1;
}
