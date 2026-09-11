#ifndef _CST_LCD_H_
#define _CST_LCD_H_

typedef enum
{
	LCD_RESET = 0,
	LCD_DEFAULT,
	LCD_DIAGS,
	LCD_AIRBRAKE_ALT,
	LCD_SPEED_ADJ,     // = LCD_DEFAULT with the AUX CGRAM slot reused for the "+/-" glyph (SPEED CFG ACCEL/DECEL adjust items)
	LCD_OPS,           // base screen + OPS MODE screen CGRAM (used regardless of the OPS MODE pref): PSI_CHAR_L -> AIRBRAKE "A" glyph, PSI_CHAR_R -> OPS "Fn active" glyph
	LCD_OPS_SPEED      // = LCD_OPS with the AM_CHAR slot reused for the narrow "H" of MPH/KMH - selected when the DISPLAY pref shows SPEED (never both AM/PM and the SPEED readout)
} LcdMode;

void displaySplashScreen(void);
void printLocomotiveAddress(uint16_t addr);
void setupLCD(LcdMode mode);
void initLCD(void);
// Redraws the AIRBRAKE ALT gauge's needle - call every render pass while that view is on screen
// (not gated by setupLCD()'s currentMode, since the needle moves live). See cst-lcd.c.
void setupGaugeChars(uint8_t psi, uint8_t maxPsi);

#endif

