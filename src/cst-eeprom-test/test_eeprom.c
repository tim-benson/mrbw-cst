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

int main(int argc, char **argv)
{
	int i1, i2, i3, i4;

	g_outdir = (argc > 1) ? argv[1] : "out";

	sc_from_blank();
	sc_from_layout1();
	sc_from_layout2();
	sc_from_layout3();
	sc_from_layout4_noop();

	printf("wrote %d reference traces to %s/\n", g_traceCount, g_outdir);

	i1 = inv_layout4_untouched();
	i2 = inv_idempotent();
	i3 = inv_blank_to_valid();
	i4 = inv_relocation_preserves();
	printf("invariant  layout-4 image untouched (no-op):   %s\n", i1 ? "PASS" : "FAIL");
	printf("invariant  migration is idempotent:            %s\n", i2 ? "PASS" : "FAIL");
	printf("invariant  blank chip -> valid layout 4:       %s\n", i3 ? "PASS" : "FAIL");
	printf("invariant  2->3 relocation preserves values:   %s\n", i4 ? "PASS" : "FAIL");
	return (i1 && i2 && i3 && i4) ? 0 : 1;
}
