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
	LCD_MAIN,          // MAIN_SCREEN CGRAM: PSI_CHAR_L -> AIRBRAKE "A" glyph, PSI_CHAR_R -> OPS "Fn active" glyph, AMPM_CHAR shown
	LCD_MAIN_SPEED,    // = LCD_MAIN with the AMPM_CHAR slot reused for the narrow "H" of MPH/KMH - selected when the DISPLAY pref shows SPEED (never both AM/PM and the SPEED readout)
	LCD_OPS,           // OPS_MODE_SCREEN CGRAM: same as LCD_MAIN minus OPS_FN_ACTIVE_CHAR (OPS_MODE_SCREEN never draws its own "Fn active" reminder - that glyph exists only for the base screen), leaving one slot genuinely free
	LCD_OPS_SPEED      // = LCD_OPS with the AMPM_CHAR slot reused for the narrow "H" of MPH/KMH, same as LCD_MAIN_SPEED
} LcdMode;

void displaySplashScreen(void);
void printLocomotiveAddress(uint16_t addr);
void setupLCD(LcdMode mode);
void initLCD(void);
// Redraws the AIRBRAKE ALT gauge's needle - call every render pass while that view is on screen
// (not gated by setupLCD()'s currentMode, since the needle moves live). See cst-lcd.c.
void setupGaugeChars(uint8_t psi, uint8_t maxPsi);
// Rewrites the LOAD_CHAR slot (cst-common.h) for the given 3-way state - call every render pass
// while a button is LOAD-active (not gated by setupLCD()'s currentMode, since the bitmap must
// track the button's live state). See cst-lcd.c.
void setupLoadChar(LoadMode loadMode);

#endif

