/*************************************************************************
Title:    EEPROM layout migrations for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
          Tim Benson <blw@east-slope.com>
File:     cst-eeprom.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2018 Michael Petersen & Nathan Holmes
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

#include <stdint.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>

#include "cst-eeprom.h"
#include "cst-functions.h"   // FN_OFF
#include "cst-speed.h"       // MOMENTUM_* / SPEED_* defaults
#include "cst-pressure.h"    // AIRBRAKE_* defaults

// One-shot EEPROM layout migrations, factored out of readConfig() in mrbw-cst.c. Everything else in
// readConfig() decodes the *current* layout into RAM globals; this rewrites the EEPROM itself when a
// throttle boots newer firmware over an older EEPROM_LAYOUT_VERSION. It touches only the EEPROM (no
// globals, no LCD, no radio) and uses only the byte-at-a-time eeprom API, so cst-eeprom-test/ can
// drive it host-side over a plain 4096-byte array. See CLAUDE.md "EEPROM layout".
//
// oldLayoutVersion is the EE_LAYOUT_VERSION byte as read by the caller, BEFORE the stamp below.
// readConfig() calls this once per boot; it is a no-op once oldLayoutVersion == EEPROM_LAYOUT_VERSION
// (which the stamp guarantees on any subsequent call within the same boot).
void applyEepromMigrations(uint8_t oldLayoutVersion)
{
	// EEPROM layout version, independent of the git-tag-derived major/minor stamp in readConfig() -
	// see the comment by EEPROM_LAYOUT_VERSION's definition in cst-eeprom.h. Lets offline tooling
	// detect a layout mismatch.
	if(oldLayoutVersion != EEPROM_LAYOUT_VERSION)
		eeprom_write_byte((uint8_t*)EE_LAYOUT_VERSION, EEPROM_LAYOUT_VERSION);

	// The per-slot SPEED + AIRBRAKE config (plus the HORN2/COMPRESSOR2 function slots wedged between
	// them) was repacked contiguous into bytes 0x3C-0x53 - see cst-eeprom.h - closing the single-byte
	// holes left by fields removed during their own uncommitted development. A throttle coming from the
	// last committed layout has its old-offset values sitting where the repacked fields now read, so
	// force the whole 0x3C-0x53 range back to defaults in every profile slot + the working config.
	// Harmless on a fresh chip (would self-heal to the same values anyway) and a no-op once past this
	// version. A configured throttle should be backed up (cst_cfgtransfer.py export) before upgrading
	// and restored (import) afterward.
	if(oldLayoutVersion < 2)
	{
		// One entry per byte 0x3C..0x53, in EEPROM-byte order.
		static const uint8_t repackDefault[0x53 - 0x3C + 1] = {
			SPEED_TYPE_DEFAULT, SPEED_OPLOAD_DEFAULT, SPEED_PRLOAD_DEFAULT,       // 0x3C-0x3E
			SPEED_OPLOAD_FN_DEFAULT, SPEED_PRLOAD_FN_DEFAULT,                     // 0x3F-0x40
			STACK_3STEP_DEFAULT_1, STACK_3STEP_DEFAULT_2, STACK_3STEP_DEFAULT_3,  // 0x41-0x43 (3-STEP combos)
			SPEED_HOLD_WATCH_FN_DEFAULT,                                         // 0x44
			SPEED_DECEL_THRESHOLD_DEFAULT, SPEED_DECEL_PCT_DEFAULT,              // 0x45-0x46
			SPEED_ACCEL_PCT_DEFAULT, SPEED_ACCEL_TARGET_DEFAULT,                // 0x47-0x48
			FN_OFF,                                                             // 0x49 HORN2_FUNCTION
			AIRBRAKE_CHARGED_DEFAULT, AIRBRAKE_MR_CUTIN_DEFAULT,                // 0x4A-0x4B
			AIRBRAKE_MR_CUTOUT_DEFAULT, AIRBRAKE_CHARGE_RATE_DEFAULT,           // 0x4C-0x4D
			AIRBRAKE_LEAK_RATE_DEFAULT, AIRBRAKE_PUMP_RATE_DEFAULT,             // 0x4E-0x4F
			AIRBRAKE_MR_LOAD_DEFAULT, AIRBRAKE_COMP_MODE_DEFAULT,               // 0x50-0x51
			FN_OFF,                                                             // 0x52 COMPRESSOR2_FUNCTION
			AIRBRAKE_DISPLAY_DEFAULT };                                         // 0x53
		uint8_t s, k;
		for(s = 1; s <= MAX_CONFIGS; s++)
		{
			wdt_reset();
			for(k = 0; k < sizeof(repackDefault); k++)
				eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(s) + 0x3C + k), repackDefault[k]);
		}
		wdt_reset();
		for(k = 0; k < sizeof(repackDefault); k++)
			eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(WORKING_CONFIG) + 0x3C + k), repackDefault[k]);
	}

	// EEPROM_LAYOUT_VERSION 2 -> 3: the 13 decoder-type-specific SPEED model parameters (BRK2/BRK3/
	// DELAY, HOLDFN/STOPFN, the load CVs and their watch functions, the four correction tunables) moved
	// out of their scattered holes in 0x2C-0x48 into the contiguous EE_SPEED_MODEL_PAYLOAD block at
	// 0x54-0x63 - see cst-eeprom.h. The 6 type-agnostic SPEED items (ACCEL/DECEL/BRK1/MAXSPEED/UNIT/
	// TYPE) did not move. A blank chip reads 0xFF (255, not < 3) here and is skipped - every field
	// self-heals via readByteOrDefault.
	if(2 == oldLayoutVersion)
	{
		// Layout 2 has a real per-profile SPEED model config. Relocate every value in place - the new
		// payload byte at 0x54+k takes the value from this profile's old scattered offset - so the
		// upgrade preserves all tuning and needs no backup/restore. v3ModelSrc[k] is the pre-move
		// offset for new payload byte 0x54+k (BRK2, BRK3, DELAY, HOLDFN, STOPFN, OPLOAD, OPLOADFN,
		// PRLOAD, PRLOADFN, ACCPCT, ACCTGT, DECPCT, DECTHR). 0x61-0x63 stay as-is (reserved).
		static const uint8_t v3ModelSrc[13] = {
			0x2C, 0x2D, 0x2F, 0x44, 0x3B, 0x3D, 0x3F, 0x3E, 0x40, 0x47, 0x48, 0x46, 0x45 };
		uint8_t s, k;
		for(s = 1; s <= MAX_CONFIGS; s++)
		{
			wdt_reset();
			for(k = 0; k < sizeof(v3ModelSrc); k++)
				eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(s) + 0x54 + k),
				                  eeprom_read_byte((uint8_t*)(CONFIG_OFFSET(s) + v3ModelSrc[k])));
		}
		wdt_reset();
		for(k = 0; k < sizeof(v3ModelSrc); k++)
			eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(WORKING_CONFIG) + 0x54 + k),
			                  eeprom_read_byte((uint8_t*)(CONFIG_OFFSET(WORKING_CONFIG) + v3ModelSrc[k])));
	}
	else if(oldLayoutVersion < 2)
	{
		// A stock/pre-guard chip has no fork SPEED model config, and its bytes at the old scattered
		// offsets are not meaningful - default the 13 payload bytes (0x54-0x60). 0x61-0x63 are reserved
		// and left as read (0xFF on an erased chip; a future field self-heals via readByteOrDefault).
		static const uint8_t speedModelDefault[13] = {
			MOMENTUM_BRAKE2_CV180_DEFAULT, MOMENTUM_BRAKE3_CV181_DEFAULT, MOMENTUM_START_DELAY_DEFAULT,
			SPEED_HOLD_WATCH_FN_DEFAULT, SPEED_STOP_WATCH_FN_DEFAULT,
			SPEED_OPLOAD_DEFAULT, SPEED_OPLOAD_FN_DEFAULT, SPEED_PRLOAD_DEFAULT, SPEED_PRLOAD_FN_DEFAULT,
			SPEED_ACCEL_PCT_DEFAULT, SPEED_ACCEL_TARGET_DEFAULT,
			SPEED_DECEL_PCT_DEFAULT, SPEED_DECEL_THRESHOLD_DEFAULT };
		uint8_t s, k;
		for(s = 1; s <= MAX_CONFIGS; s++)
		{
			wdt_reset();
			for(k = 0; k < sizeof(speedModelDefault); k++)
				eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(s) + 0x54 + k), speedModelDefault[k]);
		}
		wdt_reset();
		for(k = 0; k < sizeof(speedModelDefault); k++)
			eeprom_write_byte((uint8_t*)(CONFIG_OFFSET(WORKING_CONFIG) + 0x54 + k), speedModelDefault[k]);
	}

	// EEPROM_LAYOUT_VERSION -> 4. Five SPEED bytes leave readByteOrDefault() and are read RAW below, so
	// a stored 0xFF now means a real value (ACCEL/DECEL 0x28/0x2E = 255; HOLDFN 0x57 = OFF;
	// ACCELADJ/DECELADJ 0x61/0x62 = -127). readByteOrDefault()'s heal-on-0xFF no longer covers a
	// never-written byte, so seed the defaults here. Gated on != EEPROM_LAYOUT_VERSION (not < 4) so it
	// ALSO runs on a blank/wiped chip (oldLayoutVersion 0xFF) - the one migration that does. Runs once
	// (the version stamp at the top). 0x28/0x2E/0x57 are only rewritten if currently 0xFF (a real value
	// is preserved); 0x61/0x62 are written to 0 unconditionally - no layout-4 chip triggers this block,
	// so there can be no real ADJ value to lose, and B1 left them as arbitrary reserved bytes.
	if(oldLayoutVersion != EEPROM_LAYOUT_VERSION)
	{
		static const uint8_t rawSeedOffset[3]  = { 0x28, 0x2E, 0x57 };
		static const uint8_t rawSeedDefault[3] = { MOMENTUM_ACCEL_CV3_DEFAULT, MOMENTUM_DECEL_CV4_DEFAULT, SPEED_HOLD_WATCH_FN_DEFAULT };
		uint8_t s, k;
		for(s = 1; s <= MAX_CONFIGS + 1; s++)
		{
			// CONFIG_OFFSET() does not parenthesise its argument, so hand it a bare variable: passing the
			// ?: directly bound the macro's `- 1` to the `: WORKING_CONFIG` branch only, skipping slot 1.
			uint8_t cfgNum = (s <= MAX_CONFIGS) ? s : WORKING_CONFIG;
			uint16_t base = CONFIG_OFFSET(cfgNum);
			wdt_reset();
			for(k = 0; k < 3; k++)
				if(0xFF == eeprom_read_byte((uint8_t*)(base + rawSeedOffset[k])))
					eeprom_write_byte((uint8_t*)(base + rawSeedOffset[k]), rawSeedDefault[k]);
			eeprom_write_byte((uint8_t*)(base + 0x61), SPEED_ACCEL_ADJ_DEFAULT);
			eeprom_write_byte((uint8_t*)(base + 0x62), SPEED_DECEL_ADJ_DEFAULT);
		}
	}
}

// Writes the SPEED / AIRBRAKE / STACK "model" bytes of one 128-byte profile slot (at configBase) to
// their factory defaults - the portion of a profile that grows as decoder families and simulation
// parameters are added. resetConfig() in mrbw-cst.c calls this for the working config, then copies
// that slot to all 20 profiles. NOT here (resetConfig() writes them inline, defaults #define'd in
// mrbw-cst.c): the loco address, force-func words, brake pulse width, optionBits and the notch table;
// the 28 function-assignment bytes are owned by cst-functions.c (resetFunctionConfiguration() +
// writeFunctionConfiguration(), which covers HORN2 / BK2 / BK3 / COMPRESSOR2 at 0x49 / 0x29 / 0x2A /
// 0x52). Raw in-slot offsets + configBase, like the migrations above, so any slot base works and
// cst-eeprom-test/ can drive it host-side. Covered by `make eepromtest` (sc_reset_model +
// inv_reset_*). When adding a per-profile model field: add its in-slot offset here AND to
// resetModel_check[] in test_eeprom.c - see CLAUDE.md's maintenance checklist.
void eepromResetProfileModel(uint16_t configBase)
{
	wdt_reset();

	// Type-agnostic momentum + MAXSPEED / UNIT / TYPE (scattered - see cst-eeprom.h).
	eeprom_write_byte((uint8_t*)(configBase + 0x28), MOMENTUM_ACCEL_CV3_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x2B), MOMENTUM_BRAKE1_CV179_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x2E), MOMENTUM_DECEL_CV4_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x39), SPEED_MAX_MPH_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x3A), SPEED_UNIT_KMH_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x3C), SPEED_TYPE_DEFAULT);

	// STACK band->combo maps (band 0 is fixed to "none", not stored). 5-STEP 0x34-0x38, 3-STEP 0x41-0x43.
	eeprom_write_byte((uint8_t*)(configBase + 0x34), STACK_5STEP_DEFAULT_1);
	eeprom_write_byte((uint8_t*)(configBase + 0x35), STACK_5STEP_DEFAULT_2);
	eeprom_write_byte((uint8_t*)(configBase + 0x36), STACK_5STEP_DEFAULT_3);
	eeprom_write_byte((uint8_t*)(configBase + 0x37), STACK_5STEP_DEFAULT_4);
	eeprom_write_byte((uint8_t*)(configBase + 0x38), STACK_5STEP_DEFAULT_5);
	eeprom_write_byte((uint8_t*)(configBase + 0x41), STACK_3STEP_DEFAULT_1);
	eeprom_write_byte((uint8_t*)(configBase + 0x42), STACK_3STEP_DEFAULT_2);
	eeprom_write_byte((uint8_t*)(configBase + 0x43), STACK_3STEP_DEFAULT_3);

	// AIRBRAKE CFG 0x4A-0x53 (0x49 HORN2 / 0x52 COMPRESSOR2 are function slots - left to cst-functions.c).
	eeprom_write_byte((uint8_t*)(configBase + 0x4A), AIRBRAKE_CHARGED_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x4B), AIRBRAKE_MR_CUTIN_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x4C), AIRBRAKE_MR_CUTOUT_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x4D), AIRBRAKE_CHARGE_RATE_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x4E), AIRBRAKE_LEAK_RATE_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x4F), AIRBRAKE_PUMP_RATE_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x50), AIRBRAKE_MR_LOAD_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x51), AIRBRAKE_COMP_MODE_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x53), AIRBRAKE_DISPLAY_DEFAULT);

	// SPEED model payload 0x54-0x62 (EE_SPEED_MODEL_PAYLOAD; 0x63 reserved). Order matches the block.
	eeprom_write_byte((uint8_t*)(configBase + 0x54), MOMENTUM_BRAKE2_CV180_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x55), MOMENTUM_BRAKE3_CV181_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x56), MOMENTUM_START_DELAY_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x57), SPEED_HOLD_WATCH_FN_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x58), SPEED_STOP_WATCH_FN_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x59), SPEED_OPLOAD_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5A), SPEED_OPLOAD_FN_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5B), SPEED_PRLOAD_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5C), SPEED_PRLOAD_FN_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5D), SPEED_ACCEL_PCT_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5E), SPEED_ACCEL_TARGET_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x5F), SPEED_DECEL_PCT_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x60), SPEED_DECEL_THRESHOLD_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x61), SPEED_ACCEL_ADJ_DEFAULT);
	eeprom_write_byte((uint8_t*)(configBase + 0x62), SPEED_DECEL_ADJ_DEFAULT);
}
