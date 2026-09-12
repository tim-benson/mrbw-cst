/*************************************************************************
Title:    DCC function handling code
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-functions.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2018 Michael Petersen & Nathan Holmes
    
    This program is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.
*************************************************************************/

#include <avr/eeprom.h>

#include "lcd.h"
#include "cst-eeprom.h"
#include "cst-functions.h"
#include "cst-pressure.h"

#define SOFTWARE_LATCH   0x01
#define SPECIAL_FUNC     0x02
#define MENU_FUNC        0x04
#define LOAD_FUNC        0x08
#define CLOCK_FUNC       0x10

typedef struct
{
	const char *name;
	FunctionValues fn;
	uint16_t eeAddr;
	uint8_t attributes;
} FunctionData;

static FunctionData functions[] = {
	[HORN_FN]                = {.name = "HORN",     .eeAddr = EE_HORN_FUNCTION},
	[HORN2_FN]               = {.name = "HORN2",    .eeAddr = EE_HORN2_FUNCTION},
	[BELL_FN]                = {.name = "BELL",     .eeAddr = EE_BELL_FUNCTION},
	[BRAKE_FN]               = {.name = "BRAKE",    .eeAddr = EE_BRAKE_FUNCTION},
	[BRAKE_REL_FN]           = {.name = "BRK REL",  .eeAddr = EE_BRAKE_REL_FUNCTION},
	[EMERGENCY_FN]           = {.name = "EMRG FN",  .eeAddr = EE_EMERGENCY_FUNCTION},
	[AUX_FN]                 = {.name = "AUX",      .eeAddr = EE_AUX_FUNCTION,              .attributes = SPECIAL_FUNC},
	[ENGINE_ON_FN]           = {.name = "ENG ON",   .eeAddr = EE_ENGINE_ON_FUNCTION},
	[ENGINE_OFF_FN]          = {.name = "ENG STOP", .eeAddr = EE_ENGINE_OFF_FUNCTION},
	[THR_UNLOCK_FN]          = {.name = "THR UNLK", .eeAddr = EE_THR_UNLOCK_FUNCTION},
	[REV_SWAP_FN]            = {.name = "REV SWAP", .eeAddr = EE_REV_SWAP_FUNCTION},
	[NEUTRAL_FN]             = {.name = "CENTERED", .eeAddr = EE_NEUTRAL_FUNCTION},
	[ALERTER_FN]             = {.name = "ALERTER",  .eeAddr = EE_ALERTER_FUNCTION,          .attributes = SPECIAL_FUNC},
	[COMPRESSOR_FN]          = {.name = "COMPRSR",  .eeAddr = EE_COMPRESSOR_FUNCTION},
	[COMPRESSOR2_FN]         = {.name = "COMPRSR2", .eeAddr = EE_COMPRESSOR2_FUNCTION}, // routine/staggered compressor event (COMPMODE = CONSIST)
	[BRAKE_SET_FN]          = {.name = "BRK SET",  .eeAddr = EE_BRAKE_SET_FUNCTION},  // AIRBRAKE brake-pipe-reduction pulse (EEPROM slot 0x31, was BRK VENT / BRK TEST)
	[FRONT_DIM1_FN]          = {.name = "F.DIM #1", .eeAddr = EE_FRONT_DIM1_FUNCTION},
	[FRONT_DIM2_FN]          = {.name = "F.DIM #2", .eeAddr = EE_FRONT_DIM2_FUNCTION},
	[FRONT_HEADLIGHT_FN]     = {.name = "F.HEAD",   .eeAddr = EE_FRONT_HEADLIGHT_FUNCTION},
	[FRONT_DITCH_FN]         = {.name = "F.DITCH",  .eeAddr = EE_FRONT_DITCH_FUNCTION},
	[REAR_DIM1_FN]           = {.name = "R.DIM #1", .eeAddr = EE_REAR_DIM1_FUNCTION},
	[REAR_DIM2_FN]           = {.name = "R.DIM #2", .eeAddr = EE_REAR_DIM2_FUNCTION},
	[REAR_HEADLIGHT_FN]      = {.name = "R.HEAD",   .eeAddr = EE_REAR_HEADLIGHT_FUNCTION},
	[REAR_DITCH_FN]          = {.name = "R.DITCH",  .eeAddr = EE_REAR_DITCH_FUNCTION},
	[UP_FN]                  = {.name = "UP BTN",   .eeAddr = EE_UP_BUTTON_FUNCTION,        .attributes = SOFTWARE_LATCH|SPECIAL_FUNC|MENU_FUNC|LOAD_FUNC|CLOCK_FUNC},
	[DOWN_FN]                = {.name = "DOWN BTN", .eeAddr = EE_DOWN_BUTTON_FUNCTION,      .attributes = SOFTWARE_LATCH|SPECIAL_FUNC|MENU_FUNC|LOAD_FUNC|CLOCK_FUNC},
	[MENU_FN]                = {.name = "MENU BTN", .eeAddr = EE_MENU_BUTTON_FUNCTION,      .attributes = SOFTWARE_LATCH|SPECIAL_FUNC|MENU_FUNC|LOAD_FUNC|CLOCK_FUNC},
	[SEL_FN]                 = {.name = "SEL BTN",  .eeAddr = EE_SEL_BUTTON_FUNCTION,       .attributes = SOFTWARE_LATCH|SPECIAL_FUNC|MENU_FUNC|LOAD_FUNC|CLOCK_FUNC},
	[BK2_FN]                 = {.name = "BRAKE2",   .eeAddr = EE_BK2_FUNCTION},
	[BK3_FN]                 = {.name = "BRAKE3",   .eeAddr = EE_BK3_FUNCTION},
};

static Functions currentFunction = 0;

void printCurrentFunctionName(void)
{
	lcd_puts(functions[currentFunction].name);
}

void printCurrentFunctionValue(void)
{
	switch(functions[currentFunction].fn)
	{
		case FN_OFF:
			lcd_puts("F--     ");
			break;
		case FN_EMRG:
			lcd_puts("EMRG BRK");
			break;
		case FN_LOAD:
			lcd_puts("LOAD    ");
			break;
		case FN_AIRBRAKE:
			lcd_puts("AIRBRAKE");
			break;
		case FN_CLOCK:
			lcd_puts("CLOCK   ");
			break;
		case F00_MOM: case F10_MOM: case F20_MOM:
		case F01_MOM: case F11_MOM: case F21_MOM:
		case F02_MOM: case F12_MOM: case F22_MOM:
		case F03_MOM: case F13_MOM: case F23_MOM:  
		case F04_MOM: case F14_MOM: case F24_MOM:
		case F05_MOM: case F15_MOM: case F25_MOM:
		case F06_MOM: case F16_MOM: case F26_MOM:
		case F07_MOM: case F17_MOM: case F27_MOM:
		case F08_MOM: case F18_MOM: case F28_MOM:
		case F09_MOM: case F19_MOM:
			lcd_putc('F');
			printDec2DigWZero((functions[currentFunction].fn) & 0x1F);
			if(functions[currentFunction].attributes & SOFTWARE_LATCH)
				lcd_puts("  MOM");
			else
				lcd_puts("     ");
			break;
		case F00_LAT: case F10_LAT: case F20_LAT:
		case F01_LAT: case F11_LAT: case F21_LAT:
		case F02_LAT: case F12_LAT: case F22_LAT:
		case F03_LAT: case F13_LAT: case F23_LAT:  
		case F04_LAT: case F14_LAT: case F24_LAT:
		case F05_LAT: case F15_LAT: case F25_LAT:
		case F06_LAT: case F16_LAT: case F26_LAT:
		case F07_LAT: case F17_LAT: case F27_LAT:
		case F08_LAT: case F18_LAT: case F28_LAT:
		case F09_LAT: case F19_LAT:
			lcd_putc('F');
			printDec2DigWZero((functions[currentFunction].fn) & 0x1F);
			if(functions[currentFunction].attributes & SOFTWARE_LATCH)
				lcd_puts("  LAT");
			else
				lcd_puts("     ");
			break;
		default:
			lcd_puts("UNKNOWN ");
			break;
	}
}

void advanceCurrentFunction(uint8_t airbrakeEnabled)
{
	// COMPRSR2 only appears in the walk when AIRBRAKE is enabled AND its COMPMODE is set to CONSIST -
	// with AIRBRAKE off, none of its outputs (COMPRSR2 included) ever reach the DCC packet regardless
	// of COMPMODE, so the slot is meaningless to configure. COMPMODE itself is read live on every
	// advance, so a mode change elsewhere takes effect immediately, no stale cache to track.
	// airbrakeEnabled is passed in explicitly (mirrors cst-speed.c's inputs) - cst-functions.c has no
	// reason to know mrbw-cst.c's configBits/CONFIGBITS_AIRBRAKE internals.
	do
	{
		currentFunction++;
		if(currentFunction >= LAST_FN)
			currentFunction = 0;
	} while((COMPRESSOR2_FN == currentFunction) &&
	        (!airbrakeEnabled || (AIRBRAKE_COMP_MODE_CONSIST != airbrakeGet(AIRBRAKE_COMP_MODE))));
}

void resetCurrentFunction(void)
{
	currentFunction = 0;
}

// True if some OTHER UP/DOWN/MENU/SEL slot (not currentFunction, the one currently being edited)
// already holds FN_LOAD. LOAD's corner glyph rewrites a single shared CGRAM slot live (see
// cst-common.h / LOAD_CHAR) for whichever button holds it - a second simultaneous holder in a
// different state would corrupt the display for whichever corner drew first, since CGRAM slot
// content is global, not per-cell. This is what makes the single-slot design sound by construction
// rather than by convention: incrementCurrentFunctionValue()/decrementCurrentFunctionValue() below
// skip FN_LOAD as an option here, so it is never reachable on a second button while it is already
// held elsewhere.
static uint8_t loadUsedElsewhere(void)
{
	static const Functions loadSlots[] = { UP_FN, DOWN_FN, MENU_FN, SEL_FN };
	uint8_t i;
	for(i = 0; i < (sizeof(loadSlots)/sizeof(loadSlots[0])); i++)
	{
		if((loadSlots[i] != currentFunction) && (FN_LOAD == functions[loadSlots[i]].fn))
			return 1;
	}
	return 0;
}

/*

                                  +------------------------------------------------------->
                                  |-------------------------->
   FN_OFF ---> F00_MOM ... F28_MOM --> F00_LAT ... F28_LAT --> FN_EMRG --> FN_LOAD --> FN_AIRBRAKE --> FN_CLOCK
   ^  ^ <-------------------------+                       |-------------->      |            |             |
   |  |---------------------------------------------------+           |         |            |             |
   |  +----------------------------------------------------------------------------+          |             |
   +---------------------------------------------------------------------------------------+---------------+

*/


void incrementCurrentFunctionValue(uint8_t loadEnabled, uint8_t clockEnabled)
{
	switch(functions[currentFunction].fn)
	{
		case FN_OFF:
			functions[currentFunction].fn = F00_MOM;
			break;
		case F28_MOM:
			if(functions[currentFunction].attributes & SOFTWARE_LATCH)
			{
				functions[currentFunction].fn = F00_LAT;
			}
			else if(functions[currentFunction].attributes & SPECIAL_FUNC)
			{
				functions[currentFunction].fn = FN_EMRG;
			}
			else if(functions[currentFunction].attributes & MENU_FUNC)
			{
				functions[currentFunction].fn = FN_AIRBRAKE;
			}
			else
			{
				functions[currentFunction].fn = FN_OFF;
			}
			break;
		case F28_LAT:
			if(functions[currentFunction].attributes & SPECIAL_FUNC)
			{
				functions[currentFunction].fn = FN_EMRG;
			}
			else if(functions[currentFunction].attributes & MENU_FUNC)
			{
				functions[currentFunction].fn = FN_AIRBRAKE;
			}
			else
			{
				functions[currentFunction].fn = FN_OFF;
			}
			break;
		case FN_EMRG:
			if((functions[currentFunction].attributes & LOAD_FUNC) && loadEnabled && !loadUsedElsewhere())
			{
				functions[currentFunction].fn = FN_LOAD;
			}
			else if(functions[currentFunction].attributes & MENU_FUNC)
			{
				functions[currentFunction].fn = FN_AIRBRAKE;
			}
			else
			{
				functions[currentFunction].fn = FN_OFF;
			}
			break;
		case FN_LOAD:
			// Every slot that can ever reach FN_LOAD also carries MENU_FUNC (see functions[] above),
			// so this always lands on FN_AIRBRAKE - no need to re-test attributes the way FN_EMRG's
			// case above does.
			functions[currentFunction].fn = FN_AIRBRAKE;
			break;
		case FN_AIRBRAKE:
			if((functions[currentFunction].attributes & CLOCK_FUNC) && clockEnabled)
			{
				functions[currentFunction].fn = FN_CLOCK;
			}
			else
			{
				functions[currentFunction].fn = FN_OFF;
			}
			break;
		case FN_CLOCK:
			functions[currentFunction].fn = FN_OFF;
			break;
		case F00_MOM: case F10_MOM: case F20_MOM:
		case F01_MOM: case F11_MOM: case F21_MOM:
		case F02_MOM: case F12_MOM: case F22_MOM:
		case F03_MOM: case F13_MOM: case F23_MOM:  
		case F04_MOM: case F14_MOM: case F24_MOM:
		case F05_MOM: case F15_MOM: case F25_MOM:
		case F06_MOM: case F16_MOM: case F26_MOM:
		case F07_MOM: case F17_MOM: case F27_MOM:
		case F08_MOM: case F18_MOM: /*case F28_MOM:*/
		case F09_MOM: case F19_MOM:
		case F00_LAT: case F10_LAT: case F20_LAT:
		case F01_LAT: case F11_LAT: case F21_LAT:
		case F02_LAT: case F12_LAT: case F22_LAT:
		case F03_LAT: case F13_LAT: case F23_LAT:  
		case F04_LAT: case F14_LAT: case F24_LAT:
		case F05_LAT: case F15_LAT: case F25_LAT:
		case F06_LAT: case F16_LAT: case F26_LAT:
		case F07_LAT: case F17_LAT: case F27_LAT:
		case F08_LAT: case F18_LAT: /*case F28_LAT:*/
		case F09_LAT: case F19_LAT:
			// Take advantage of the bottom 5 bits being the function number
			functions[currentFunction].fn++;
			break;
		default:
			// Handle undefined (or reset default) values from EEPROM
			functions[currentFunction].fn = FN_OFF;
			break;
	}
}


/*

   FN_OFF <--- F00_MOM ... F28_MOM <--- F00_LAT ... F28_LAT <--- FN_EMRG <--- FN_LOAD <--- FN_AIRBRAKE <--- FN_CLOCK

*/

void decrementCurrentFunctionValue(uint8_t loadEnabled, uint8_t clockEnabled)
{
	switch(functions[currentFunction].fn)
	{
		case F00_MOM:
			functions[currentFunction].fn = FN_OFF;
			break;
		case F00_LAT:
			functions[currentFunction].fn = F28_MOM;
			break;
		case FN_EMRG:
			if(functions[currentFunction].attributes & SOFTWARE_LATCH)
			{
				functions[currentFunction].fn = F28_LAT;
			}
			else
			{
				functions[currentFunction].fn = F28_MOM;
			}
			break;
		case FN_LOAD:
			// Every slot that can ever reach FN_LOAD also carries SPECIAL_FUNC (see functions[]
			// above), so this always lands on FN_EMRG - no need to re-test attributes the way
			// FN_AIRBRAKE's case below does.
			functions[currentFunction].fn = FN_EMRG;
			break;
		case FN_AIRBRAKE:
			if((functions[currentFunction].attributes & LOAD_FUNC) && loadEnabled && !loadUsedElsewhere())
			{
				functions[currentFunction].fn = FN_LOAD;
			}
			else if(functions[currentFunction].attributes & SPECIAL_FUNC)
			{
				functions[currentFunction].fn = FN_EMRG;
			}
			else if(functions[currentFunction].attributes & SOFTWARE_LATCH)
			{
				functions[currentFunction].fn = F28_LAT;
			}
			else
			{
				functions[currentFunction].fn = F28_MOM;
			}
			break;
		case FN_CLOCK:
			// Symmetric with FN_LOAD's decrement above: every clock-capable slot also carries
			// MENU_FUNC, so this always lands on FN_AIRBRAKE - no need to re-test attributes.
			functions[currentFunction].fn = FN_AIRBRAKE;
			break;
		case FN_OFF:
			if((functions[currentFunction].attributes & CLOCK_FUNC) && clockEnabled)
			{
				functions[currentFunction].fn = FN_CLOCK;
			}
			else if(functions[currentFunction].attributes & MENU_FUNC)
			{
				functions[currentFunction].fn = FN_AIRBRAKE;
			}
			else if(functions[currentFunction].attributes & SPECIAL_FUNC)
			{
				functions[currentFunction].fn = FN_EMRG;
			}
			else if(functions[currentFunction].attributes & SOFTWARE_LATCH)
			{
				functions[currentFunction].fn = F28_LAT;
			}
			else
			{
				functions[currentFunction].fn = F28_MOM;
			}
			break;
		/*case F00_MOM:*/ case F10_MOM: case F20_MOM:
		case F01_MOM: case F11_MOM: case F21_MOM:
		case F02_MOM: case F12_MOM: case F22_MOM:
		case F03_MOM: case F13_MOM: case F23_MOM:  
		case F04_MOM: case F14_MOM: case F24_MOM:
		case F05_MOM: case F15_MOM: case F25_MOM:
		case F06_MOM: case F16_MOM: case F26_MOM:
		case F07_MOM: case F17_MOM: case F27_MOM:
		case F08_MOM: case F18_MOM: case F28_MOM:
		case F09_MOM: case F19_MOM:
		/*case F00_LAT:*/ case F10_LAT: case F20_LAT:
		case F01_LAT: case F11_LAT: case F21_LAT:
		case F02_LAT: case F12_LAT: case F22_LAT:
		case F03_LAT: case F13_LAT: case F23_LAT:  
		case F04_LAT: case F14_LAT: case F24_LAT:
		case F05_LAT: case F15_LAT: case F25_LAT:
		case F06_LAT: case F16_LAT: case F26_LAT:
		case F07_LAT: case F17_LAT: case F27_LAT:
		case F08_LAT: case F18_LAT: case F28_LAT:
		case F09_LAT: case F19_LAT:
			// Take advantage of the bottom 5 bits being the function number
			functions[currentFunction].fn--;
			break;
		default:
			// Handle undefined (or reset default) values from EEPROM
			functions[currentFunction].fn = FN_OFF;
			break;
	}
}

void readFunctionConfiguration(void)
{
	uint8_t i;
	for(i=0; i<(sizeof(functions)/sizeof(FunctionData)); i++)
	{
		functions[i].fn = eeprom_read_byte((uint8_t*)(functions[i].eeAddr));
	}
}

void writeFunctionConfiguration(void)
{
	uint8_t i;
	for(i=0; i<(sizeof(functions)/sizeof(FunctionData)); i++)
	{
		eeprom_write_byte((uint8_t*)(functions[i].eeAddr), functions[i].fn);
	}
}

uint8_t isFunctionOff(Functions functionName)
{
	if(FN_OFF == functions[functionName].fn)
		return 1;
	else
		return 0;
}

uint8_t isFunctionEstop(Functions functionName)
{
	if(FN_EMRG == functions[functionName].fn)
		return 1;
	else
		return 0;
}

uint8_t isFunctionAirBrake(Functions functionName)
{
	if(FN_AIRBRAKE == functions[functionName].fn)
		return 1;
	else
		return 0;
}

uint8_t isFunctionLoad(Functions functionName)
{
	if(FN_LOAD == functions[functionName].fn)
		return 1;
	else
		return 0;
}

// Clears any UP/DOWN/MENU/SEL BTN currently set to FN_LOAD back to FN_OFF, updating both RAM and
// EEPROM directly (not via writeFunctionConfiguration(), which would also re-persist every other
// slot's current RAM value - unnecessary here and a needlessly wide write). Called from mrbw-cst.c's
// SPEED CFG save whenever the saved TYPE no longer models the load CVs (V4): without this, a button
// already configured to LOAD would keep showing "LOAD" in CONFIG FUNC (printCurrentFunctionValue()
// is unconditional) even though it has become unreachable via the UP/DOWN value cycle - that cycle's
// loadEnabled gate only guards *selecting* LOAD, not an existing stale assignment.
void clearLoadFunctions(void)
{
	static const Functions loadSlots[] = { UP_FN, DOWN_FN, MENU_FN, SEL_FN };
	uint8_t i;
	for(i = 0; i < (sizeof(loadSlots)/sizeof(loadSlots[0])); i++)
	{
		if(FN_LOAD == functions[loadSlots[i]].fn)
		{
			functions[loadSlots[i]].fn = FN_OFF;
			eeprom_write_byte((uint8_t*)(functions[loadSlots[i]].eeAddr), FN_OFF);
		}
	}
}

uint8_t isFunctionClock(Functions functionName)
{
	if(FN_CLOCK == functions[functionName].fn)
		return 1;
	else
		return 0;
}

// Clears any UP/DOWN/MENU/SEL BTN currently set to FN_CLOCK back to FN_OFF, updating both RAM and
// EEPROM directly - mirrors clearLoadFunctions() above. Called from mrbw-cst.c's PREFS save whenever
// the saved DISPLAY no longer shows SPEED: without this, a button already configured to CLOCK would
// keep showing "CLOCK" in CONFIG FUNC even though there is no SPEED readout left to peek away from.
void clearClockFunctions(void)
{
	static const Functions clockSlots[] = { UP_FN, DOWN_FN, MENU_FN, SEL_FN };
	uint8_t i;
	for(i = 0; i < (sizeof(clockSlots)/sizeof(clockSlots[0])); i++)
	{
		if(FN_CLOCK == functions[clockSlots[i]].fn)
		{
			functions[clockSlots[i]].fn = FN_OFF;
			eeprom_write_byte((uint8_t*)(functions[clockSlots[i]].eeAddr), FN_OFF);
		}
	}
}

uint8_t isFunctionLatching(Functions functionName)
{
	switch(functions[functionName].fn)
	{
		case F00_LAT: case F10_LAT: case F20_LAT:
		case F01_LAT: case F11_LAT: case F21_LAT:
		case F02_LAT: case F12_LAT: case F22_LAT:
		case F03_LAT: case F13_LAT: case F23_LAT:  
		case F04_LAT: case F14_LAT: case F24_LAT:
		case F05_LAT: case F15_LAT: case F25_LAT:
		case F06_LAT: case F16_LAT: case F26_LAT:
		case F07_LAT: case F17_LAT: case F27_LAT:
		case F08_LAT: case F18_LAT: case F28_LAT:
		case F09_LAT: case F19_LAT:
		case FN_EMRG:
			return 1;
			break;
		default:
			return 0;
			break;
	}
}

uint32_t getFunctionMask(Functions functionName)
{
	if(
		((functions[functionName].fn >= F00_MOM) && (functions[functionName].fn <= F28_MOM)) ||
		((functions[functionName].fn >= F00_LAT) && (functions[functionName].fn <= F28_LAT))
	)
		return ((uint32_t)1 << (functions[functionName].fn & 0x1F));
	else
		return 0x00000000;
}

void resetFunctionConfiguration(void)
{
	functions[HORN_FN].fn = F02_MOM;
	functions[HORN2_FN].fn = FN_OFF;
	functions[BELL_FN].fn = F01_MOM;
	functions[FRONT_DIM1_FN].fn = FN_OFF;
	functions[FRONT_DIM2_FN].fn = FN_OFF;
	functions[FRONT_HEADLIGHT_FN].fn = F00_MOM;
	functions[FRONT_DITCH_FN].fn = FN_OFF;
	functions[REAR_DIM1_FN].fn = FN_OFF;
	functions[REAR_DIM2_FN].fn = FN_OFF;
	functions[REAR_HEADLIGHT_FN].fn = F00_MOM;
	functions[REAR_DITCH_FN].fn = FN_OFF;
	functions[BRAKE_FN].fn = F10_MOM;
	functions[BRAKE_REL_FN].fn = FN_OFF;
	functions[AUX_FN].fn = F09_MOM;
	functions[ENGINE_ON_FN].fn = F08_MOM;
	functions[ENGINE_OFF_FN].fn = FN_OFF;
	functions[UP_FN].fn = F05_MOM;
	functions[DOWN_FN].fn = F06_MOM;
	functions[MENU_FN].fn = FN_OFF;
	functions[SEL_FN].fn = FN_OFF;
	functions[THR_UNLOCK_FN].fn = FN_OFF;
	functions[REV_SWAP_FN].fn = FN_OFF;
	functions[NEUTRAL_FN].fn = FN_OFF;
	functions[ALERTER_FN].fn = FN_OFF;
	functions[COMPRESSOR_FN].fn = FN_OFF;
	functions[COMPRESSOR2_FN].fn = FN_OFF;
	functions[BRAKE_SET_FN].fn = FN_OFF;
	functions[EMERGENCY_FN].fn = FN_OFF;
	functions[BK2_FN].fn = FN_OFF;
	functions[BK3_FN].fn = FN_OFF;
}

