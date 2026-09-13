#ifndef _CST_LCD_H_
#define _CST_LCD_H_

#include "cst-speed.h"    // LoadMode, for setupLoadChar()'s parameter

typedef enum
{
	LCD_RESET = 0,
	LCD_DEFAULT,
	LCD_DIAGS,
	LCD_AIRBRAKE_ALT,
	LCD_SPEED_ADJ,     // = LCD_DEFAULT with the AUX CGRAM slot reused for the "+/-" glyph (SPEED CFG ACCEL/DECEL adjust items)
	LCD_MAIN,          // MAIN_SCREEN CGRAM: slots 1/2 (softkey circle), 4, and 6 (PSI_CHAR_L) are the dynamic hollow/filled/AIRBRAKE/LOAD/CLOCK pool (see cst-common.h), PSI_CHAR_R -> OPS "Fn active" glyph, AMPM_CHAR shown
	LCD_MAIN_SPEED,    // = LCD_MAIN with the AMPM_CHAR slot reused for the narrow "H" of MPH/KMH - selected when the DISPLAY pref shows SPEED (never both AM/PM and the SPEED readout)
	LCD_OPS,           // OPS_MODE_SCREEN CGRAM: same as LCD_MAIN minus OPS_FN_ACTIVE_CHAR (OPS_MODE_SCREEN never draws its own "Fn active" reminder - that glyph exists only for the base screen), folding that slot into the pool as a 5th member instead
	LCD_OPS_SPEED      // = LCD_OPS with the AMPM_CHAR slot reused for the narrow "H" of MPH/KMH, same as LCD_MAIN_SPEED
} LcdMode;

void displaySplashScreen(void);
void printLocomotiveAddress(uint16_t addr);
void setupLCD(LcdMode mode);
void initLCD(void);
// Redraws the AIRBRAKE ALT gauge's needle - call every render pass while that view is on screen
// (not gated by setupLCD()'s currentMode, since the needle moves live). See cst-lcd.c.
void setupGaugeChars(uint8_t psi, uint8_t maxPsi);
// Rewrites the given CGRAM slot with the hollow/filled softkey circle bitmap - the same bitmaps
// setupSoftkeyChars() below loads at their usual fixed slots (1/2) for LCD_DEFAULT/LCD_DIAGS, but
// parameterized so mrbw-cst.c's allocateSpecialGlyphSlots() can write either into whichever slot its
// OPS_MODE_SCREEN pool assigns them (see cst-common.h). See cst-lcd.c.
void setupSoftkeyInactiveChar(uint8_t slot);
void setupSoftkeyActiveChar(uint8_t slot);
// Rewrites the given CGRAM slot with the LOAD button's OFF/OPLOAD/PRLOAD glyph for the given 3-way
// state. Call every render pass while a button is LOAD-active (not gated by setupLCD()'s
// currentMode, since the bitmap must track the button's live state) - which slot number to pass is
// decided by mrbw-cst.c's allocateSpecialGlyphSlots() (see cst-common.h). See cst-lcd.c.
void setupLoadChar(uint8_t slot, LoadMode loadMode);
// Rewrites the given CGRAM slot with the "A" glyph for an AIRBRAKE-bound button corner. Which slot
// number to pass is decided by mrbw-cst.c's allocateSpecialGlyphSlots() (see cst-common.h), called
// every render pass - not a fixed slot. See cst-lcd.c.
void setupAirbrakeGlyphChar(uint8_t slot);
// Rewrites the given CGRAM slot with the clock-face glyph for a CLOCK-bound button corner. See
// setupAirbrakeGlyphChar() above - same calling convention. See cst-lcd.c.
void setupClockPeekGlyphChar(uint8_t slot);
// Rewrites the given CGRAM slot with the STOP glyph for a button corner whose configured DCC
// function number matches STOPFN. See setupAirbrakeGlyphChar() above - same calling convention.
// See cst-lcd.c.
void setupStopGlyphChar(uint8_t slot);
// Loads the narrow "H" bitmap into SPEED_H_CHAR (cst-common.h, slot 3). Called by setupLCD() on
// entry to LCD_MAIN_SPEED/LCD_OPS_SPEED, and again by mrbw-cst.c's renderBaseScreen() every render
// pass while not peeking - not gated by currentMode, since a CLOCK peek's AM/PM draw
// (displayTime(), cst-time.c) may have left the slot holding that bitmap instead. See cst-lcd.c.
void setupSpeedHChar(void);

#endif

