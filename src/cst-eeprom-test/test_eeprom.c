/*************************************************************************
Title:    Reference-trace test for the EEPROM layout migrations
Authors:  Tim Benson <blw@east-slope.com>
File:     cst-eeprom-test/test_eeprom.c
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
 *   applyEepromMigrations() in cst-eeprom.c over synthetic pre-migration EEPROM
 *   images (a plain 4096-byte array) and dumps the resulting image to a
 *   plain-text "trace", one file per starting layout version. `make eepromtest`
 *   diffs the freshly generated traces against the checked-in copies under
 *   reference/. Any diff means the migration output moved - an intended change
 *   (regenerate with `make eepromtest-accept`) or a regression to investigate.
 *   Same golden-master technique as `make speedtest` / `make pressuretest`.
 *
 *   This is the ONLY coverage of "an old-layout throttle boots newer firmware":
 *   a wrong offset in the 2->3 relocation map silently corrupts every stored
 *   loco profile on upgrade, and there is no other way to exercise that path
 *   without a physical chip holding a specific EEPROM image.
 *
 * HOW IT COMPILES ON THE HOST
 *   cst-eeprom.c includes <avr/eeprom.h> and <avr/wdt.h> (shimmed via
 *   stubs/avr/ on the -I path - the migrations use only the byte-at-a-time
 *   eeprom API), plus cst-eeprom.h / cst-functions.h / cst-speed.h /
 *   cst-pressure.h, which each pull in nothing but <stdint.h>. So this file
 *   defines the g_eeprom[] backing array the eeprom shim needs, then #includes
 *   ../cst-eeprom.c whole so its file-static migration tables are in scope.
 *
 * AVR-vs-host arithmetic fidelity
 *   applyEepromMigrations() does only uint8_t / uint16_t address arithmetic and
 *   byte copies - no signed math, no width-sensitive intermediates - so it is
 *   identical between AVR 16-bit int and host 32-bit int by construction.
 */

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* The eeprom shim (stubs/avr/eeprom.h) externs this; define it here, before the
 * model is pulled in. All migration reads/writes land in this array. */
uint8_t g_eeprom[4096];

/* The code under test, pulled in whole so its file-static tables (repackDefault,
 * v3ModelSrc, speedModelDefault, rawSeedOffset) are reachable for review here. */
#include "../cst-eeprom.c"

/* ---- image builders ------------------------------------------------------- */

static const char *g_outdir;
static int          g_traceCount;

/* A factory-blank chip: every byte 0xFF (including EE_LAYOUT_VERSION, which
 * reads as 0xFF until the first boot on layout-aware firmware). */
static void buildBlank(void)
{
	memset(g_eeprom, 0xFF, sizeof g_eeprom);
}

/* Distinguishable per-in-slot-offset sentinel: offset 0x00..0x7F -> 0x40..0xBF,
 * all below 0xFF (so never mistaken for an erased byte) and all distinct within
 * a slot (so the blessed "after" image shows exactly which source offset each
 * migrated byte came from). */
static uint8_t slotSentinel(int off)
{
	return (uint8_t)(0x40 + off);
}

/* A synthetic pre-migration image at layout version v: blank, stamp the version
 * byte, then fill every profile slot (1..MAX_CONFIGS) and the working config
 * with the per-offset sentinel. The 0x00-0x7F global block and the unused
 * slots 21-30 stay 0xFF - the migrations must never touch them, and the trace
 * shows that by their absence. */
static void buildLayout(uint8_t v)
{
	int off;
	uint8_t s;

	buildBlank();
	g_eeprom[EE_LAYOUT_VERSION] = v;

	for (s = 1; s <= MAX_CONFIGS; s++)
		for (off = 0; off < 0x80; off++)
			g_eeprom[CONFIG_OFFSET(s) + off] = slotSentinel(off);
	for (off = 0; off < 0x80; off++)
		g_eeprom[CONFIG_OFFSET(WORKING_CONFIG) + off] = slotSentinel(off);
}

/* ---- trace dump ---------------------------------------------------------- */

/* Dump the current g_eeprom image: 16 bytes/row, all-0xFF rows elided (a "*"
 * marker stands in for a run of skipped rows, like `hexdump -C`). This is the
 * post-migration state - what `make eepromtest` diffs. */
static void dumpImage(const char *name, const char *startNote)
{
	char path[512];
	FILE *f;
	int base, i, skipping = 0;

	snprintf(path, sizeof path, "%s/%s.txt", g_outdir, name);
	f = fopen(path, "w");
	if (!f)
	{
		perror(path);
		exit(2);
	}

	fprintf(f, "# scenario: %s\n", name);
	fprintf(f, "# start:    %s\n", startNote);
	fprintf(f, "# image:    4096-byte EEPROM after applyEepromMigrations(); 16 bytes/row, all-0xFF rows elided\n");
	fprintf(f, "#\n");

	for (base = 0; base < 4096; base += 16)
	{
		int allFF = 1;
		for (i = 0; i < 16; i++)
			if (g_eeprom[base + i] != 0xFF)
			{
				allFF = 0;
				break;
			}
		if (allFF)
		{
			if (!skipping)
				fprintf(f, "*\n");
			skipping = 1;
			continue;
		}
		skipping = 0;
		fprintf(f, "%04X ", base);
		for (i = 0; i < 16; i++)
			fprintf(f, " %02X", g_eeprom[base + i]);
		fprintf(f, "\n");
	}

	fclose(f);
	g_traceCount++;
}

/* ---- scenarios ---------------------------------------------------------- */

/* Blank chip (EE_LAYOUT_VERSION reads 0xFF) boots current firmware: the version
 * stamp writes 4, the 1->2 and 2->3 blocks are skipped (0xFF is not < 2), and
 * the -> 4 raw-seed block runs (it is the one migration gated != VERSION, not
 * < N, so it also covers a blank chip) - seeding 0x28/0x2E/0x57 defaults and
 * 0x61/0x62 = 0. Everything else self-heals later via readByteOrDefault(), so
 * nothing else is written here.
 *
 * The trace shows every profile slot + the working config with the five bytes
 * seeded (0x28/0x2E/0x57 = defaults, 0x61/0x62 = 0), the version byte at 0x26,
 * and nothing else touched. */
static void sc_from_blank(void)
{
	buildBlank();
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	dumpImage("from_blank", "all 0xFF (EE_LAYOUT_VERSION 0xFF)");
}

/* A stock/pre-guard chip (layout 1): version stamp -> 4; the 1->2 block
 * force-resets 0x3C-0x53 to repackDefault[] in all 21 slots; the "else if
 * (oldLayoutVersion < 2)" arm defaults the 13 SPEED model bytes 0x54-0x60; the
 * -> 4 block preserves the (non-0xFF sentinel) 0x28/0x2E/0x57 and writes
 * 0x61/0x62 = 0. Sentinels survive everywhere outside 0x3C-0x62, showing the
 * migration is confined to that range. */
static void sc_from_layout1(void)
{
	buildLayout(1);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	dumpImage("from_layout1", "layout 1, every slot byte = 0x40 + offset");
}

/* Layout 2 -> current: the interesting one. Version stamp -> 4; the 1->2 block
 * is skipped; the "2 == oldLayoutVersion" block RELOCATES 13 scattered SPEED
 * model bytes into 0x54-0x60, so 0x54+k ends up holding the sentinel from old
 * offset v3ModelSrc[k] (= 0x40 + v3ModelSrc[k]) - a layout-2 throttle keeps
 * every tuned value; the -> 4 block preserves 0x28/0x2E/0x57 and writes
 * 0x61/0x62 = 0. */
static void sc_from_layout2(void)
{
	buildLayout(2);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	dumpImage("from_layout2", "layout 2, every slot byte = 0x40 + offset");
}

/* Layout 3 -> 4: version stamp -> 4; both the 1->2 and 2->3 blocks are skipped;
 * only the -> 4 raw-seed block runs - 0x28/0x2E/0x57 sentinels preserved,
 * 0x61/0x62 = 0. The minimal migration. */
static void sc_from_layout3(void)
{
	buildLayout(3);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	dumpImage("from_layout3", "layout 3, every slot byte = 0x40 + offset");
}

/* Already on the current layout: applyEepromMigrations(4) must be a complete
 * no-op - not even the version stamp is rewritten. The trace is byte-identical
 * to the (sentinel-filled) input; invariant 1 also checks this by memcmp. */
static void sc_from_layout4_noop(void)
{
	buildLayout(4);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	dumpImage("from_layout4_noop", "layout 4 (current), every slot byte = 0x40 + offset");
}

/* Fill one 128-byte slot with the 0x40 + offset sentinel (only that slot, so
 * the reset_model trace stays small). Lets a check tell "written to its default"
 * from "left untouched" even for the fields whose default is 0xFF (STOPFN /
 * OPLOADFN / PRLOADFN). */
static void fillSlotSentinels(uint16_t base)
{
	int off;
	for (off = 0; off < 0x80; off++)
		g_eeprom[base + off] = slotSentinel(off);
}

/* eepromResetProfileModel() over a sentinel-filled working-config slot: the
 * SPEED / AIRBRAKE / STACK "model" bytes 0x28..0x62 are overwritten with their
 * factory defaults; the function slots (0x29/0x2A/0x30-0x33/0x49/0x52) and the
 * freed holes keep their sentinel; nothing outside the model offsets is touched.
 * resetConfig() calls this for the working config then copies it to all 20
 * profiles. */
static void sc_reset_model(void)
{
	uint16_t wc = CONFIG_OFFSET(WORKING_CONFIG);
	buildBlank();
	fillSlotSentinels(wc);
	eepromResetProfileModel(wc);
	dumpImage("reset_model", "working-config slot = 0x40 + offset, then eepromResetProfileModel()");
}

/* ---- invariants (asserted in main() as PASS/FAIL, exit non-zero on any fail) ---
 * Properties the reference files alone cannot express - same role as the
 * V4==V5MULT check in the scale-speed harness. */

/* The 2->3 relocation map, an INDEPENDENT copy of v3ModelSrc[] in cst-eeprom.c:
 * new payload byte EE_SPEED_MODEL_PAYLOAD + k takes its value from old offset
 * v3ModelSrc_check[k]. If cst-eeprom.c's table is edited, this must match or
 * invariant 4 fails - that is the point. */
static const uint8_t v3ModelSrc_check[13] = {
	0x2C, 0x2D, 0x2F, 0x44, 0x3B, 0x3D, 0x3F, 0x3E, 0x40, 0x47, 0x48, 0x46, 0x45
};

/* All 21 slot bases (20 profiles + the working config), for the per-slot checks.
 * CONFIG_OFFSET() does not parenthesize its argument, so feed it a bare variable,
 * never a compound expression. */
static uint16_t slotBase(int idx)
{
	uint8_t cfg = (idx < MAX_CONFIGS) ? (uint8_t)(idx + 1) : (uint8_t)WORKING_CONFIG;
	return CONFIG_OFFSET(cfg);
}

/* 1. A current-layout image is left completely untouched - applyEepromMigrations(4)
 *    writes zero bytes. */
static int inv_layout4_untouched(void)
{
	static uint8_t before[4096];
	buildLayout(4);
	memcpy(before, g_eeprom, sizeof before);
	applyEepromMigrations(4);
	return 0 == memcmp(before, g_eeprom, sizeof before);
}

/* 2. Idempotent: after a real 2 -> current migration, re-running with the
 *    (now-stamped) version byte changes nothing more - exactly what readConfig()
 *    does on every subsequent boot. */
static int inv_idempotent(void)
{
	static uint8_t settled[4096];
	buildLayout(2);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	memcpy(settled, g_eeprom, sizeof settled);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));
	return 0 == memcmp(settled, g_eeprom, sizeof settled);
}

static int seededDefaults(uint16_t b)
{
	return g_eeprom[b + 0x28] == MOMENTUM_ACCEL_CV3_DEFAULT
	    && g_eeprom[b + 0x2E] == MOMENTUM_DECEL_CV4_DEFAULT
	    && g_eeprom[b + 0x57] == SPEED_HOLD_WATCH_FN_DEFAULT
	    && g_eeprom[b + 0x61] == SPEED_ACCEL_ADJ_DEFAULT
	    && g_eeprom[b + 0x62] == SPEED_DECEL_ADJ_DEFAULT;
}

/* 3. Blank chip -> layout 4: the version byte is stamped, and the -> 4 raw-seed
 *    loop populates the five raw-read SPEED bytes (0x28/0x2E/0x57/0x61/0x62 -
 *    readByteOrDefault() no longer covers them) with their real defaults, in
 *    every one of the 20 profile slots AND the working config. */
static int inv_blank_to_valid(void)
{
	int idx, ok = 1;
	buildBlank();
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));

	if (g_eeprom[EE_LAYOUT_VERSION] != EEPROM_LAYOUT_VERSION)
		ok = 0;
	for (idx = 0; idx < MAX_CONFIGS + 1; idx++)
		if (!seededDefaults(slotBase(idx)))
			ok = 0;
	return ok;
}

/* 4. The 2->3 relocation preserves every value: a sentinel at old offset
 *    v3ModelSrc_check[k] lands at EE_SPEED_MODEL_PAYLOAD + k in every slot. */
static int inv_relocation_preserves(void)
{
	int idx, k, ok = 1;
	buildLayout(2);
	applyEepromMigrations(eeprom_read_byte((uint8_t *)EE_LAYOUT_VERSION));

	for (idx = 0; idx < MAX_CONFIGS + 1; idx++)
	{
		uint16_t b = slotBase(idx);
		for (k = 0; k < 13; k++)
			if (g_eeprom[b + 0x54 + k] != slotSentinel(v3ModelSrc_check[k]))
				ok = 0;
	}
	return ok;
}

/* ---- eepromResetProfileModel() coverage ---------------------------------- */

/* The complete set of per-profile model offsets eepromResetProfileModel() must
 * write, with the factory default expected at each - an INDEPENDENT restatement
 * of the writes in cst-eeprom.c (the v3ModelSrc_check[] idiom). Adding a field
 * to the reset function without adding it here (or vice versa) fails a check. */
static const struct { uint8_t off; uint8_t val; } resetModel_check[] = {
	{ 0x28, MOMENTUM_ACCEL_CV3_DEFAULT },    { 0x2B, MOMENTUM_BRAKE1_CV179_DEFAULT }, { 0x2E, MOMENTUM_DECEL_CV4_DEFAULT },
	{ 0x34, STACK_5STEP_DEFAULT_1 }, { 0x35, STACK_5STEP_DEFAULT_2 }, { 0x36, STACK_5STEP_DEFAULT_3 },
	{ 0x37, STACK_5STEP_DEFAULT_4 }, { 0x38, STACK_5STEP_DEFAULT_5 },
	{ 0x39, SPEED_MAX_MPH_DEFAULT }, { 0x3A, SPEED_UNIT_KMH_DEFAULT }, { 0x3C, SPEED_TYPE_DEFAULT },
	{ 0x41, STACK_3STEP_DEFAULT_1 }, { 0x42, STACK_3STEP_DEFAULT_2 }, { 0x43, STACK_3STEP_DEFAULT_3 },
	{ 0x4A, AIRBRAKE_CHARGED_DEFAULT },     { 0x4B, AIRBRAKE_MR_CUTIN_DEFAULT },  { 0x4C, AIRBRAKE_MR_CUTOUT_DEFAULT },
	{ 0x4D, AIRBRAKE_CHARGE_RATE_DEFAULT }, { 0x4E, AIRBRAKE_LEAK_RATE_DEFAULT }, { 0x4F, AIRBRAKE_PUMP_RATE_DEFAULT },
	{ 0x50, AIRBRAKE_MR_LOAD_DEFAULT },     { 0x51, AIRBRAKE_COMP_MODE_DEFAULT }, { 0x53, AIRBRAKE_DISPLAY_DEFAULT },
	{ 0x54, MOMENTUM_BRAKE2_CV180_DEFAULT }, { 0x55, MOMENTUM_BRAKE3_CV181_DEFAULT }, { 0x56, MOMENTUM_START_DELAY_DEFAULT },
	{ 0x57, SPEED_HOLD_WATCH_FN_DEFAULT }, { 0x58, SPEED_STOP_WATCH_FN_DEFAULT },
	{ 0x59, SPEED_OPLOAD_DEFAULT }, { 0x5A, SPEED_OPLOAD_FN_DEFAULT }, { 0x5B, SPEED_PRLOAD_DEFAULT }, { 0x5C, SPEED_PRLOAD_FN_DEFAULT },
	{ 0x5D, SPEED_ACCEL_PCT_DEFAULT }, { 0x5E, SPEED_ACCEL_TARGET_DEFAULT }, { 0x5F, SPEED_DECEL_PCT_DEFAULT }, { 0x60, SPEED_DECEL_THRESHOLD_DEFAULT },
	{ 0x61, SPEED_ACCEL_ADJ_DEFAULT }, { 0x62, SPEED_DECEL_ADJ_DEFAULT },
};
#define RESET_MODEL_CHECK_N ((int)(sizeof resetModel_check / sizeof resetModel_check[0]))

/* Every offset in [0x28, 0x62] is one of: a model field (above), a function
 * slot (cst-functions.c owns it), or a documented freed hole. The three sets
 * partition the range exactly (38 + 8 + 13 = 59), so any offset that fits none
 * is a bug in this test's bookkeeping. */
static int isFunctionSlot(uint8_t off)
{
	return off == 0x29 || off == 0x2A || off == 0x30 || off == 0x31
	    || off == 0x32 || off == 0x33 || off == 0x49 || off == 0x52;
}
static int isFreedHole(uint8_t off)
{
	return off == 0x2C || off == 0x2D || off == 0x2F || off == 0x3B
	    || (off >= 0x3D && off <= 0x40) || (off >= 0x44 && off <= 0x48);
}
static int inResetModelTable(uint8_t off)
{
	int k;
	for (k = 0; k < RESET_MODEL_CHECK_N; k++)
		if (resetModel_check[k].off == off)
			return 1;
	return 0;
}

/* 5. eepromResetProfileModel() overwrites every model offset with its factory
 *    default (checked against a sentinel, so it catches an unwritten field even
 *    where the default is 0xFF), and [0x28, 0x62] is fully accounted for
 *    (model / function slot / freed hole partition the range exactly). This is
 *    the forcing function: a new field added to cst-eeprom.c but not to
 *    resetModel_check[] (or a hole reused without updating the partition) fails. */
static int inv_reset_model_complete(void)
{
	uint16_t b = CONFIG_OFFSET(WORKING_CONFIG);
	int k, ok = 1;
	uint8_t o;

	buildBlank();
	fillSlotSentinels(b);
	eepromResetProfileModel(b);

	for (k = 0; k < RESET_MODEL_CHECK_N; k++)
		if (g_eeprom[b + resetModel_check[k].off] != resetModel_check[k].val)
			ok = 0;
	for (o = 0x28; o <= 0x62; o++)
	{
		int model = inResetModelTable(o), fn = isFunctionSlot(o), hole = isFreedHole(o);
		if (model + fn + hole != 1)                        /* sets must partition the range */
			ok = 0;
		if ((fn || hole) && g_eeprom[b + o] != slotSentinel(o))   /* reset must not touch these */
			ok = 0;
	}
	return ok;
}

/* 6. eepromResetProfileModel() writes ONLY the model offsets - it must not
 *    disturb the function bytes, loco address, notch table or padding that
 *    resetConfig() writes separately, nor any byte of any other slot. */
static int inv_reset_confined(void)
{
	static uint8_t before[4096];
	uint16_t b = CONFIG_OFFSET(WORKING_CONFIG);
	int o, ok = 1;

	buildBlank();
	fillSlotSentinels(b);
	memcpy(before, g_eeprom, sizeof before);
	eepromResetProfileModel(b);

	for (o = 0; o < 4096; o++)
	{
		int shouldChange = 0;
		if (o >= (int)b && o < (int)b + 0x80)
			shouldChange = inResetModelTable((uint8_t)(o - b));
		if (!shouldChange && g_eeprom[o] != before[o])
			ok = 0;
	}
	return ok;
}

/* 7. The SPEED model-payload defaults (0x54-0x60) eepromResetProfileModel()
 *    writes must equal what applyEepromMigrations()'s speedModelDefault[] seeds
 *    on a pre-2 chip - the two default sources in cst-eeprom.c must not drift. */
static int inv_reset_agrees_with_migration_defaults(void)
{
	uint16_t b = slotBase(MAX_CONFIGS);
	uint8_t viaMigration[13];
	int k, ok = 1;

	buildBlank();
	applyEepromMigrations(1);   /* < 2 -> the speedModelDefault[] branch seeds 0x54-0x60 */
	for (k = 0; k < 13; k++)
		viaMigration[k] = g_eeprom[b + 0x54 + k];

	buildBlank();
	eepromResetProfileModel(b);
	for (k = 0; k < 13; k++)
		if (g_eeprom[b + 0x54 + k] != viaMigration[k])
			ok = 0;
	return ok;
}

int main(int argc, char **argv)
{
	int i1, i2, i3, i4, i5, i6, i7;

	g_outdir = (argc > 1) ? argv[1] : "out";

	sc_from_blank();
	sc_from_layout1();
	sc_from_layout2();
	sc_from_layout3();
	sc_from_layout4_noop();
	sc_reset_model();

	printf("wrote %d reference traces to %s/\n", g_traceCount, g_outdir);

	i1 = inv_layout4_untouched();
	i2 = inv_idempotent();
	i3 = inv_blank_to_valid();
	i4 = inv_relocation_preserves();
	i5 = inv_reset_model_complete();
	i6 = inv_reset_confined();
	i7 = inv_reset_agrees_with_migration_defaults();
	printf("invariant  layout-4 image untouched (no-op):        %s\n", i1 ? "PASS" : "FAIL");
	printf("invariant  migration is idempotent:                 %s\n", i2 ? "PASS" : "FAIL");
	printf("invariant  blank chip -> valid layout 4:            %s\n", i3 ? "PASS" : "FAIL");
	printf("invariant  2->3 relocation preserves values:        %s\n", i4 ? "PASS" : "FAIL");
	printf("invariant  reset-model: every field at its default: %s\n", i5 ? "PASS" : "FAIL");
	printf("invariant  reset-model: confined to [0x28,0x62]:     %s\n", i6 ? "PASS" : "FAIL");
	printf("invariant  reset-model agrees with migration seed:  %s\n", i7 ? "PASS" : "FAIL");
	return (i1 && i2 && i3 && i4 && i5 && i6 && i7) ? 0 : 1;
}
