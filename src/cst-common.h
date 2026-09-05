#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

// CGRAM slots are reused per LcdMode: slots 6/7 - PSI_CHAR_L/R under LCD_DEFAULT (AIRBRAKE unit
// glyph), BELL_CHAR/HORN_CHAR under LCD_DIAGS. All 8 slots (0-7) - GAUGE_CHAR_A0-A3/B0-B3 under
// LCD_AIRBRAKE_ALT (the analogue gauge dial face, one custom char per 5x8 cell of a 2-row x 4-col
// canvas - no room left for anything else while this mode is active). setupLCD() reloads the
// static ones on every mode change; the gauge's cells are additionally rewritten every render pass
// (setupGaugeChars() in cst-lcd.c) since the needle moves live.

// Default
#define BATTERY_CHAR            0
#define FUNCTION_INACTIVE_CHAR  1
#define FUNCTION_ACTIVE_CHAR    2
#define AM_CHAR                 3
#define PM_CHAR                 4
#define AUX_CHAR                5
#define PSI_CHAR_L              6
#define PSI_CHAR_R              7

// Diags
#define BELL_CHAR               6
#define HORN_CHAR               7

// AIRBRAKE ALT (analogue gauge) - consumes all 8 slots
#define GAUGE_CHAR_A0           0
#define GAUGE_CHAR_A1           1
#define GAUGE_CHAR_A2           2
#define GAUGE_CHAR_A3           3
#define GAUGE_CHAR_B0           4
#define GAUGE_CHAR_B1           5
#define GAUGE_CHAR_B2           6
#define GAUGE_CHAR_B3           7

void wait100ms(uint16_t loops);

#endif
