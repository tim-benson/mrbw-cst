#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

// CGRAM slots are reused per LcdMode: slots 6/7 - PSI_CHAR_L/R under LCD_DEFAULT (AIRBRAKE unit
// glyph), BELL_CHAR/HORN_CHAR under LCD_DIAGS, slot 5 - PLUSMINUS_CHAR under LCD_SPEED_ADJ (SPEED
// CFG ACCEL/DECEL adjust labels, borrowing the AUX slot which is main-screen-only). The base screen
// and the OPS MODE screen use LCD_OPS / LCD_OPS_SPEED (the DISPLAY pref picks which): slot 6 -
// AIRBRAKE_GLYPH_CHAR (the "A" glyph on an AIRBRAKE-bound button corner, borrowing the PSI_CHAR_L
// slot which is AIRBRAKE-screen-only), slot 7 - OPS_FN_ACTIVE_CHAR (the "Fn active" reminder glyph,
// borrowing PSI_CHAR_R), and under LCD_OPS_SPEED slot 3 - SPEED_H_CHAR (the narrow "H" of MPH/KMH,
// borrowing the AM_CHAR slot since the SPEED readout and the AM/PM clock indicator are never both
// shown) and slot 4 - LOAD_CHAR (the LOAD button's OFF/OPLOAD/PRLOAD glyph, borrowing the PM_CHAR
// slot - see the LOAD_CHAR definition below for why this is safe). All 8 slots (0-7) -
// GAUGE_CHAR_A0-A3/B0-B3 under LCD_AIRBRAKE_ALT (the analogue gauge dial face, one custom char per
// 5x8 cell of a 2-row x 4-col canvas - no room left for anything else while this mode is active).
// setupLCD() reloads the static ones on every mode change; the gauge's cells and LOAD_CHAR are
// additionally rewritten every render pass (setupGaugeChars()/setupLoadChar() in cst-lcd.c) since
// their content moves/changes live rather than only on a mode change.

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

// SPEED CFG ACCEL/DECEL adjust ("ACCEL +/-") - reuses the AUX slot (rendered only on MAIN_SCREEN;
// the menu-exit setupLCD(LCD_DEFAULT) restores AUX before the main screen can draw it)
#define PLUSMINUS_CHAR          5

// OPS MODE "Fn active" reminder glyph - reuses the PSI_CHAR_R slot (rendered only on the AIRBRAKE
// screen; setupLCD() reloads PSI_CHAR_R whenever currentMode leaves LCD_OPS / LCD_OPS_SPEED)
#define OPS_FN_ACTIVE_CHAR      7

// "A" glyph on an AIRBRAKE-bound button corner - reuses the PSI_CHAR_L slot (rendered only on the
// AIRBRAKE DUAL screen, which reloads it on its own LCD_DEFAULT mode change)
#define AIRBRAKE_GLYPH_CHAR     6

// Narrow "H" of MPH/KMH in the running SPEED readout - reuses the AM_CHAR slot under LCD_OPS_SPEED
// (the SPEED readout and the AM/PM clock indicator are mutually exclusive - the DISPLAY pref shows
// one or the other)
#define SPEED_H_CHAR            3

// LOAD button (UP/DOWN/MENU/SEL BTN = LOAD) OFF/OPLOAD/PRLOAD glyph - reuses the PM_CHAR slot.
// Safe because LOAD is only ever active while SPEED is enabled (CONFIG FUNC's runtime gate, see
// loadEligible()/loadActive() in mrbw-cst.c), which is exactly the condition that selects
// LCD_OPS_SPEED (AM/PM not shown, SPEED_H_CHAR above reuses AM_CHAR's slot instead) over LCD_OPS -
// and because LOAD is firmware-restricted to at most one of the four buttons at a time
// (loadUsedElsewhere() in cst-functions.c), so this one dynamic slot is never asked to represent
// two different states at once. setupLoadChar() rewrites this slot's bitmap every render pass (like
// the AIRBRAKE gauge needle), NOT via setupLCD()'s mode-guarded reload, so it tracks the button's
// live 3-way state - and the render-time loadActive() gate is what stops that rewrite from firing
// (and so corrupting the real PM glyph) once DISPLAY drops back to CLOCK. A future MAIN/OPS MODE
// renderer split would let this become a plain, unconditional static slot like AIRBRAKE_GLYPH_CHAR
// - see the LOAD plan's "Follow-on work" section.
#define LOAD_CHAR                4

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
