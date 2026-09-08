#ifndef _CST_LCD_H_
#define _CST_LCD_H_

typedef enum
{
	LCD_RESET = 0,
	LCD_DEFAULT,
	LCD_DIAGS,
	LCD_AIRBRAKE_ALT,
	LCD_SPEED_ADJ       // = LCD_DEFAULT with the AUX CGRAM slot reused for the "+/-" glyph (SPEED CFG ACCEL/DECEL adjust items)
} LcdMode;

void displaySplashScreen(void);
void printLocomotiveAddress(uint16_t addr);
void setupLCD(LcdMode mode);
void initLCD(void);
// Redraws the AIRBRAKE ALT gauge's needle - call every render pass while that view is on screen
// (not gated by setupLCD()'s currentMode, since the needle moves live). See cst-lcd.c.
void setupGaugeChars(uint8_t psi, uint8_t maxPsi);

#endif

