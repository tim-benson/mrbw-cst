/*************************************************************************
Title:    LCD support for Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     cst-lcd.c
License:  GNU General Public License v3

LICENSE:
    Copyright (C) 2017 Michael Petersen & Nathan Holmes
    
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
#include <string.h>

#include "lcd.h"

#include "cst-common.h"
#include "cst-hardware.h"
#include "cst-lcd.h"
#include "cst-hardware.h"
#include "cst-battery.h"
#include "cst-math.h"
#include "cst-time.h"

const uint8_t Bell[8] =
{
	0b00000100,
	0b00001110,
	0b00001110,
	0b00001110,
	0b00011111,
	0b00000000,
	0b00000100,
	0b00000000
};

const uint8_t Horn[8] =
{
	0b00000000,
	0b00000001,
	0b00010011,
	0b00011111,
	0b00010011,
	0b00000001,
	0b00000000,
	0b00000000
};

const uint8_t Aux[8] =
{
	0b00001000,
	0b00010100,
	0b00011100,
	0b00010100,
	0b00001010,
	0b00000100,
	0b00001010,
	0b00000000
};

const uint8_t SoftkeyInactive[8] =
{
	0b00000000,
	0b00001110,
	0b00010001,
	0b00010001,
	0b00010001,
	0b00001110,
	0b00000000,
	0b00000000
};
const uint8_t SoftkeyActive[8] =
{
	0b00000000,
	0b00001110,
	0b00011111,
	0b00011111,
	0b00011111,
	0b00001110,
	0b00000000,
	0b00000000
};

void setupDiagChars(void)
{
	lcd_setup_custom(BELL_CHAR, Bell);
	lcd_setup_custom(HORN_CHAR, Horn);
}

// Rewrites the given CGRAM slot with the hollow/filled softkey circle bitmap. Split out from
// setupSoftkeyChars() below so mrbw-cst.c's allocateSpecialGlyphSlots() can write either one into
// whichever slot it assigns them on OPS_MODE_SCREEN's pool - see cst-common.h.
void setupSoftkeyInactiveChar(uint8_t slot)
{
	lcd_setup_custom(slot, SoftkeyInactive);
}

void setupSoftkeyActiveChar(uint8_t slot)
{
	lcd_setup_custom(slot, SoftkeyActive);
}

// Unconditional, fixed-slot form for LCD_DEFAULT and LCD_DIAGS, which never reallocate slots 1/2 -
// only LCD_MAIN*/LCD_OPS* fold them into mrbw-cst.c's allocateSpecialGlyphSlots() pool instead (see
// cst-common.h), so setupLCD() no longer calls this for those four modes.
void setupSoftkeyChars(void)
{
	setupSoftkeyInactiveChar(FUNCTION_INACTIVE_CHAR);
	setupSoftkeyActiveChar(FUNCTION_ACTIVE_CHAR);
}

void setupAuxChars(void)
{
	lcd_setup_custom(AUX_CHAR, Aux);
}

// "+/-" for the SPEED CFG ACCEL/DECEL adjust labels (ACCELADJ/DECELADJ). A full '+' (rows 0-4) over a
// '-' bar (row 6). Reuses the AUX CGRAM slot under LCD_SPEED_ADJ - see cst-common.h.
const uint8_t PlusMinus[8] =
{
	0b00000100,
	0b00000100,
	0b00011111,
	0b00000100,
	0b00000100,
	0b00000000,
	0b00011111,
	0b00000000
};

void setupPlusMinusChar(void)
{
	lcd_setup_custom(PLUSMINUS_CHAR, PlusMinus);
}

// "PSI" unit label for the AIRBRAKE screen - a hand-drawn 2-cell glyph (P S I across 10x8).
const uint8_t PsiCharL[8] =
{
	0b00000110,
	0b00000101,
	0b00000101,
	0b00000110,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00000000
};

const uint8_t PsiCharR[8] =
{
	0b00001001,
	0b00010101,
	0b00010001,
	0b00001001,
	0b00000101,
	0b00010101,
	0b00001001,
	0b00000000
};

void setupPsiChars(void)
{
	lcd_setup_custom(PSI_CHAR_L, PsiCharL);
	lcd_setup_custom(PSI_CHAR_R, PsiCharR);
}

// OPS MODE "Fn active" reminder glyph - a stylised "Fn". Shown on the base screen (OPS MODE enabled
// but exited) at column 0 while a MENU BTN / SEL BTN function is still latched on. Occupies the
// PSI_CHAR_R slot, loaded only under LCD_MAIN / LCD_MAIN_SPEED - OPS_MODE_SCREEN's own LCD_OPS /
// LCD_OPS_SPEED never draws this glyph, so it is left unloaded there (the genuinely free slot). The
// AIRBRAKE screen (the only other PSI_CHAR_R user) forces LCD_DEFAULT, which reloads PSI_CHAR_R.
const uint8_t OpsFnActive[8] =
{
	0b00011100,
	0b00010000,
	0b00011000,
	0b00010010,
	0b00010101,
	0b00010101,
	0b00010101,
	0b00000000
};

void setupOpsChars(void)
{
	lcd_setup_custom(OPS_FN_ACTIVE_CHAR, OpsFnActive);
}

// Bold "A" glyph shown on an AIRBRAKE-bound button corner (UP/DOWN/MENU/SEL BTN = AIRBRAKE), in
// place of the softkey circle - the button jumps to the AIRBRAKE gauge rather than driving a DCC
// function. Written into whichever pool slot mrbw-cst.c's allocateSpecialGlyphSlots() assigns it
// this render pass (see cst-common.h) - not a fixed slot, and not loaded by setupLCD().
const uint8_t AirbrakeGlyph[8] =
{
	0b00000000,
	0b00001110,
	0b00010001,
	0b00010001,
	0b00011111,
	0b00010001,
	0b00010001,
	0b00000000
};

void setupAirbrakeGlyphChar(uint8_t slot)
{
	lcd_setup_custom(slot, AirbrakeGlyph);
}

// Small clock-face glyph shown on a CLOCK-bound button corner (UP/DOWN/MENU/SEL BTN = CLOCK), shown
// for as long as that assignment exists (not gated on the button being held). Written into whichever
// pool slot allocateSpecialGlyphSlots() assigns it this render pass - see setupAirbrakeGlyphChar()
// above.
const uint8_t ClockPeekGlyph[8] =
{
	0b00000000,
	0b00001110,
	0b00010101,
	0b00010111,
	0b00010001,
	0b00001110,
	0b00000000,
	0b00000000
};

void setupClockPeekGlyphChar(uint8_t slot)
{
	lcd_setup_custom(slot, ClockPeekGlyph);
}

// Narrow "H" for the MPH/KMH unit in the running SPEED readout (printSpeed()), tighter than the
// font-ROM 'H' so the 6-char readout field reads less cramped. Occupies the AMPM_CHAR slot under
// LCD_MAIN_SPEED / LCD_OPS_SPEED - only loaded when the DISPLAY pref shows SPEED, so it never
// collides with the AM/PM clock indicator.
const uint8_t SpeedNarrowH[8] =
{
	0b00010010,
	0b00010010,
	0b00010010,
	0b00011110,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00000000
};

void setupSpeedHChar(void)
{
	lcd_setup_custom(SPEED_H_CHAR, SpeedNarrowH);
}

// LOAD button (UP/DOWN/MENU/SEL BTN = LOAD) OFF/OPLOAD/PRLOAD glyphs. Written into whichever pool
// slot mrbw-cst.c's allocateSpecialGlyphSlots() assigns it this render pass - see cst-common.h.
const uint8_t LoadOff[8] =
{
	0b00000000,
	0b00000000,
	0b00000000,
	0b00000010,
	0b00000010,
	0b00000010,
	0b00000011,
	0b00000000
};

const uint8_t LoadOpLoad[8] =
{
	0b00001000,
	0b00010100,
	0b00010100,
	0b00001010,
	0b00000010,
	0b00000010,
	0b00000011,
	0b00000000
};

const uint8_t LoadPrLoad[8] =
{
	0b00011000,
	0b00010100,
	0b00011000,
	0b00010010,
	0b00000010,
	0b00000010,
	0b00000011,
	0b00000000
};

// Rewrites the given slot for the button's current 3-way state - called every render pass while a
// button is LOAD-active (mrbw-cst.c's allocateSpecialGlyphSlots()), not gated by setupLCD()'s
// currentMode, since the bitmap must track live state even while currentMode stays LCD_OPS_SPEED
// across passes.
void setupLoadChar(uint8_t slot, LoadMode loadMode)
{
	switch(loadMode)
	{
		case LOAD_MODE_OPLOAD: lcd_setup_custom(slot, LoadOpLoad); break;
		case LOAD_MODE_PRLOAD: lcd_setup_custom(slot, LoadPrLoad); break;
		default:                lcd_setup_custom(slot, LoadOff);   break;
	}
}

// --- AIRBRAKE ALT: the original ISE analogue pressure gauge, revived from the last commit before
// BRAKESIM replaced it (git 3cde842:src/cst-pressure.c) and rewired to the new sim's BP value. The
// dial artwork, canvas geometry, and Bresenham needle plotter are unchanged from the original -
// only the value driving the needle angle changed (was PumpState's milliPressure, now
// airBrakePipePsi()/AIRBRAKE_CHARGED, passed in by the caller so this file has no dependency on
// cst-pressure.c's internals).

#define GAUGE_CANVAS_ROWS       16
#define GAUGE_CANVAS_COLS       20
#define GAUGE_ROWS_PER_CHAR      8
#define GAUGE_COLS_PER_CHAR      5

// East = 0deg, South = 90deg, West = 180deg, North = 270deg
#define GAUGE_MIN_ANGLE        112
#define GAUGE_MAX_ANGLE        300
#define GAUGE_ORIGIN_X          10
#define GAUGE_ORIGIN_Y           8
#define GAUGE_NEEDLE_LENGTH      7

static uint8_t gaugeCanvas[GAUGE_CANVAS_ROWS / GAUGE_ROWS_PER_CHAR][GAUGE_CANVAS_COLS / GAUGE_COLS_PER_CHAR][GAUGE_ROWS_PER_CHAR];

const uint8_t Gauge[GAUGE_CANVAS_ROWS / GAUGE_ROWS_PER_CHAR][GAUGE_CANVAS_COLS / GAUGE_COLS_PER_CHAR][GAUGE_ROWS_PER_CHAR] =
{
	{
		{
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000001,
			0b00000011,
			0b00000010,
			0b00000010,
			0b00000010
		},
		{
			0b00000111,
			0b00001000,
			0b00011000,
			0b00000100,
			0b00000000,
			0b00010000,
			0b00000000,
			0b00000000
		},
		{
			0b00011110,
			0b00010001,
			0b00010001,
			0b00000010,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000
		},
		{
			0b00000000,
			0b00000000,
			0b00010000,
			0b00001000,
			0b00001100,
			0b00010100,
			0b00000100,
			0b00000100
		}
	},
	{
		{
			0b00000011,
			0b00000010,
			0b00000010,
			0b00000010,
			0b00000001,
			0b00000000,
			0b00000000,
			0b00000000
		},
		{
			0b00010000,
			0b00000000,
			0b00000000,
			0b00001000,
			0b00010000,
			0b00010010,
			0b00001100,
			0b00000111
		},
		{
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000000,
			0b00000001,
			0b00011110
		},
		{
			0b00011100,
			0b00000100,
			0b00000100,
			0b00000100,
			0b00001000,
			0b00010000,
			0b00000000,
			0b00000000
		}
	}
};

static void gaugePlot(uint8_t x, uint8_t y)
{
	uint8_t row = y / GAUGE_ROWS_PER_CHAR;
	if(row >= (GAUGE_CANVAS_ROWS / GAUGE_ROWS_PER_CHAR))
		return;
	uint8_t col = x / GAUGE_COLS_PER_CHAR;
	if(col >= (GAUGE_CANVAS_COLS / GAUGE_COLS_PER_CHAR))
		return;
	gaugeCanvas[row][col][y % GAUGE_ROWS_PER_CHAR] |= 1 << ((GAUGE_COLS_PER_CHAR - 1) - (x % GAUGE_COLS_PER_CHAR));
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
static void gaugePlotLineLow(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	int8_t dx = x1 - x0;
	int8_t dy = y1 - y0;
	int8_t yi = 1;
	if(dy < 0)
	{
		yi = -1;
		dy = -dy;
	}
	int8_t D = 2*dy - dx;
	int8_t y = y0;
	int8_t x;

	for(x=x0; x<=x1; x++)
	{
		gaugePlot(x,y);
		if(D > 0)
		{
			y = y + yi;
			D = D - 2*dx;
		}
		D = D + 2*dy;
	}
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
static void gaugePlotLineHigh(uint8_t x0, uint8_t y0, uint8_t x1, uint8_t y1)
{
	int8_t dx = x1 - x0;
	int8_t dy = y1 - y0;
	int8_t xi = 1;
	if(dx < 0)
	{
		xi = -1;
		dx = -dx;
	}
	int8_t D = 2*dx - dy;
	int8_t x = x0;
	int8_t y;

	for(y=y0; y<=y1; y++)
	{
		gaugePlot(x,y);
		if(D > 0)
		{
			x = x + xi;
			D = D - 2*dy;
		}
		D = D + 2*dx;
	}
}

/* https://en.wikipedia.org/wiki/Bresenham's_line_algorithm */
static void gaugePlotLine(int8_t x0, int8_t y0, int8_t x1, int8_t y1)
{
	if(abs(y1 - y0) < abs(x1 - x0))
	{
		if(x0 > x1)
			gaugePlotLineLow(x1, y1, x0, y0);
		else
			gaugePlotLineLow(x0, y0, x1, y1);
	}
	else
	{
		if(y0 > y1)
			gaugePlotLineHigh(x1, y1, x0, y0);
		else
			gaugePlotLineHigh(x0, y0, x1, y1);
	}
}

// Redraws the needle at an angle proportional to psi/maxPsi (clamped by construction - the sim never
// lets BP exceed the configured charge) and uploads all 8 CGRAM cells. Called every render pass while
// AIRBRAKE ALT is on screen - the needle moves live, so unlike the rest of this file's setupXxxChars()
// helpers (called once per LcdMode change via setupLCD()), this one is not gated by currentMode.
void setupGaugeChars(uint8_t psi, uint8_t maxPsi)
{
	float ratio = (maxPsi > 0) ? ((float)psi / (float)maxPsi) : 0.0;
	float degrees = ((GAUGE_MAX_ANGLE - GAUGE_MIN_ANGLE) * ratio) + GAUGE_MIN_ANGLE;
	float radians = degrees * PI / 180.0;

	int8_t x = round(cos_32(radians) * GAUGE_NEEDLE_LENGTH);
	int8_t y = round(sin_32(radians) * GAUGE_NEEDLE_LENGTH);

	memcpy(gaugeCanvas, Gauge, sizeof(gaugeCanvas));

	gaugePlotLine(GAUGE_ORIGIN_X, GAUGE_ORIGIN_Y, GAUGE_ORIGIN_X + x, GAUGE_ORIGIN_Y + y);

	lcd_setup_custom(GAUGE_CHAR_A0, gaugeCanvas[0][0]);
	lcd_setup_custom(GAUGE_CHAR_A1, gaugeCanvas[0][1]);
	lcd_setup_custom(GAUGE_CHAR_A2, gaugeCanvas[0][2]);
	lcd_setup_custom(GAUGE_CHAR_A3, gaugeCanvas[0][3]);
	lcd_setup_custom(GAUGE_CHAR_B0, gaugeCanvas[1][0]);
	lcd_setup_custom(GAUGE_CHAR_B1, gaugeCanvas[1][1]);
	lcd_setup_custom(GAUGE_CHAR_B2, gaugeCanvas[1][2]);
	lcd_setup_custom(GAUGE_CHAR_B3, gaugeCanvas[1][3]);
}

// Splash Screen Characters
const uint8_t Splash1[8] =
{
	0b00011100,
	0b00010010,
	0b00010010,
	0b00011100,
	0b00010000,
	0b00010000,
	0b00000000,
	0b00000000
};

const uint8_t Splash2[8] =
{
	0b00011100,
	0b00010010,
	0b00010010,
	0b00011100,
	0b00010010,
	0b00010010,
	0b00000000,
	0b00000000
};

const uint8_t Splash3[8] =
{
	0b00001100,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00010010,
	0b00001100,
	0b00000000,
	0b00000000
};

const uint8_t Splash4[8] =
{
	0b00011100,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00001001,
	0b00001000,
	0b00000000,
	0b00000000
};

const uint8_t Splash5A[8] =
{
	0b00011001,
	0b00000101,
	0b00000101,
	0b00000101,
	0b00000101,
	0b00011000,
	0b00000000,
	0b00000000
};

const uint8_t Splash5C[8] =
{
	0b00011001,
	0b00000101,
	0b00000100,
	0b00000101,
	0b00000111,
	0b00011100,
	0b00000100,
	0b00000011
};

const uint8_t Splash6A[8] =
{
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash6B[8] =
{
	0b00011111,
	0b00000001,
	0b00000001,
	0b00000001,
	0b00011101,
	0b00000001,
	0b00000001,
	0b00000000
};

const uint8_t Splash6C[8] =
{
	0b00011111,
	0b00001111,
	0b00011110,
	0b00011101,
	0b00011010,
	0b00010100,
	0b00011000,
	0b00010000
};

const uint8_t Splash7A[8] =
{
	0b00011111,
	0b00011111,
	0b00001111,
	0b00000111,
	0b00011011,
	0b00000001,
	0b00000000,
	0b00000000
};

const uint8_t Splash7B[8] =
{
	0b00011111,
	0b00011100,
	0b00011100,
	0b00011100,
	0b00011101,
	0b00000100,
	0b00000100,
	0b00011000
};

const uint8_t Splash7C[8] =
{
	0b00011111,
	0b00001000,
	0b00010000,
	0b00000000,
	0b00011111,
	0b00000000,
	0b00000000,
	0b00000000
};

const uint8_t Splash8A[8] =
{
	0b00011100,
	0b00010100,
	0b00011000,
	0b00011100,
	0b00011110,
	0b00011001,
	0b00011001,
	0b00001110
};

const uint8_t Splash8B[8] =
{
	0b00011100,
	0b00000100,
	0b00000100,
	0b00000100,
	0b00011100,
	0b00000000,
	0b00000000,
	0b00000000
};

void displaySplashScreen(void)
{
	uint8_t i;
	
	lcd_clrscr();

	lcd_setup_custom(0, Splash1);
	lcd_setup_custom(1, Splash2);
	lcd_setup_custom(2, Splash3);
	lcd_setup_custom(3, Splash4);
	lcd_setup_custom(4, Splash5A);
	lcd_setup_custom(5, Splash6A);
	lcd_setup_custom(6, Splash7A);
	lcd_setup_custom(7, Splash8A);

	lcd_gotoxy(0,0);
	for(i=0; i<8; i++)
		lcd_putc(i);
	
	lcd_gotoxy(0,1);
	lcd_puts("THROTTLE");

	wait100ms(8);

	lcd_setup_custom(5, Splash6B);
	lcd_setup_custom(6, Splash7B);
	lcd_setup_custom(7, Splash8B);

	wait100ms(4);

	lcd_setup_custom(4, Splash5C);
	lcd_setup_custom(5, Splash6C);
	lcd_setup_custom(6, Splash7C);

	wait100ms(15);
}

void printLocomotiveAddress(uint16_t addr)
{
	if(addr & LOCO_ADDRESS_SHORT)
	{
		lcd_putc('s');
		printDec3DigWZero(addr & ~(LOCO_ADDRESS_SHORT));
	}
	else
	{
		printDec4DigWZero(addr);
	}
}

LcdMode currentMode = LCD_RESET;

void setupLCD(LcdMode mode)
{
	if(currentMode != mode)
	{
		switch(mode)
		{
			case LCD_RESET:
				break;
			case LCD_DEFAULT:
				// No clock glyph here: printTime() (the only reader of AMPM_CHAR) is only ever
				// reached from renderBaseScreen(), which is only entered under LCD_MAIN*/LCD_OPS* -
				// never LCD_DEFAULT. Loading it here would be dead weight (confirmed: no screen that
				// uses LCD_DEFAULT - menu screens, the AIRBRAKE DUAL view - ever calls printTime()).
				setupBatteryChar();
				setupSoftkeyChars();
				setupAuxChars();
				setupPsiChars();
				break;
			case LCD_DIAGS:
				setupSoftkeyChars();
				setupDiagChars();
				break;
			case LCD_AIRBRAKE_ALT:
				// No static setup here - all 8 cells are rewritten every render pass by
				// setupGaugeChars() instead (the needle moves live). This case only exists so
				// currentMode tracks reality, letting a later setupLCD(LCD_DEFAULT) correctly
				// detect the change and reload the battery/softkey/aux/PSI glyphs.
				break;
			case LCD_SPEED_ADJ:
				// = LCD_DEFAULT with the AUX slot reused for the "+/-" glyph. SPEED CFG is always
				// entered from a LCD_DEFAULT context so the other 7 slots are already loaded; the
				// menu-exit setupLCD(LCD_DEFAULT) restores AUX because currentMode changed here.
				setupPlusMinusChar();
				break;
			case LCD_MAIN:
			case LCD_MAIN_SPEED:
			case LCD_OPS:
			case LCD_OPS_SPEED:
				// MAIN_SCREEN (LCD_MAIN*) and OPS_MODE_SCREEN (LCD_OPS*) each get their own palette.
				// Slots 1, 2 (softkey circle), 4, and 6 (PSI_CHAR_L) - plus, on LCD_OPS*, slot 7
				// (PSI_CHAR_R, otherwise unused there) - are a dynamic pool shared by the plain
				// hollow/filled circle and every icon-bearing special button function (AIRBRAKE,
				// LOAD, CLOCK, and any added later): deliberately never written here in any of the
				// four cases, since which concept belongs in which pool slot depends on live button
				// configuration, not just LcdMode, so it stays exclusively runtime-managed by
				// mrbw-cst.c's allocateSpecialGlyphSlots() every render pass (see cst-common.h for
				// why this is always sufficient, however many special functions exist). PSI_CHAR_R is
				// the OPS "Fn active" glyph only under LCD_MAIN* - OPS_MODE_SCREEN never draws it
				// (renderBaseScreen() only references OPS_FN_ACTIVE_CHAR when opsScreen is false), so
				// LCD_OPS* leaves that slot to the pool instead. The _SPEED variants reuse AMPM_CHAR's
				// slot for the narrow "H" unit glyph instead; the non-speed variants only invalidate
				// the AM/PM sentinel here, never write a bitmap directly - see invalidateAmPmChar()
				// (cst-time.h) for why.
				setupBatteryChar();
				setupAuxChars();
				if((LCD_MAIN == mode) || (LCD_MAIN_SPEED == mode))
					setupOpsChars();
				if((LCD_MAIN_SPEED == mode) || (LCD_OPS_SPEED == mode))
					setupSpeedHChar();
				else
					invalidateAmPmChar();
				break;
		}
		currentMode = mode;
	}
}

void initLCD(void)
{
	lcd_init(LCD_DISP_ON);
	enableLCDBacklight();

	displaySplashScreen();

	lcd_clrscr();
	
	currentMode = LCD_RESET;
	setupLCD(LCD_DEFAULT);
}

