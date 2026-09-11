#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

// CGRAM slots are reused per LcdMode: slots 6/7 - PSI_CHAR_L/R under LCD_DEFAULT (AIRBRAKE unit
// glyph), BELL_CHAR/HORN_CHAR under LCD_DIAGS, slot 5 - PLUSMINUS_CHAR under LCD_SPEED_ADJ (SPEED
// CFG ACCEL/DECEL adjust labels, borrowing the AUX slot which is main-screen-only). MAIN_SCREEN and
// OPS_MODE_SCREEN use two genuinely separate palettes (LCD_MAIN / LCD_MAIN_SPEED and LCD_OPS /
// LCD_OPS_SPEED respectively, the DISPLAY pref picking the _SPEED variant of whichever): both load
// slot 6 - AIRBRAKE_GLYPH_CHAR (the "A" glyph on an AIRBRAKE-bound button corner, borrowing the
// PSI_CHAR_L slot which is AIRBRAKE-screen-only) and slot 4 - LOAD_CHAR (the LOAD button's OFF/
// OPLOAD/PRLOAD glyph, permanently reserved - see the LOAD_CHAR definition below). LCD_MAIN(_SPEED)
// additionally loads slot 7 - OPS_FN_ACTIVE_CHAR (the "Fn active" reminder glyph, borrowing
// PSI_CHAR_R); LCD_OPS(_SPEED) does not (OPS_MODE_SCREEN never draws it), leaving that slot free -
// LCD_MAIN(_SPEED) is 8/8, LCD_OPS(_SPEED) is 7/8. Both palettes load slot 3 - AMPM_CHAR (non-speed)
// or SPEED_H_CHAR (speed variant, the narrow "H" of MPH/KMH) - the SPEED readout and the AM/PM clock
// indicator are mutually exclusive, the DISPLAY pref shows one or the other. All 8 slots (0-7) -
// GAUGE_CHAR_A0-A3/B0-B3 under LCD_AIRBRAKE_ALT (the analogue gauge dial face, one custom char per
// 5x8 cell of a 2-row x 4-col canvas - no room left for anything else while this mode is active).
// setupLCD() reloads the static ones on every mode change; the gauge's cells, LOAD_CHAR, and
// AMPM_CHAR are additionally rewritten from the main render loop (setupGaugeChars()/setupLoadChar()
// in cst-lcd.c, the change-detected block in cst-time.c's displayTime()) since their content moves/
// changes live rather than only on a mode change.

// Default
#define BATTERY_CHAR            0
#define FUNCTION_INACTIVE_CHAR  1
#define FUNCTION_ACTIVE_CHAR    2
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
// screen; setupLCD() reloads PSI_CHAR_R whenever currentMode leaves LCD_MAIN / LCD_MAIN_SPEED)
#define OPS_FN_ACTIVE_CHAR      7

// "A" glyph on an AIRBRAKE-bound button corner - reuses the PSI_CHAR_L slot (rendered only on the
// AIRBRAKE DUAL screen, which reloads it on its own LCD_DEFAULT mode change)
#define AIRBRAKE_GLYPH_CHAR     6

// Narrow "H" of MPH/KMH in the running SPEED readout - reuses the AMPM_CHAR slot under
// LCD_MAIN_SPEED / LCD_OPS_SPEED (the SPEED readout and the AM/PM clock indicator are mutually
// exclusive - the DISPLAY pref shows one or the other)
#define SPEED_H_CHAR            3

// AM/PM clock indicator - one dynamically-rewritten slot rather than two static ones (AM_CHAR/
// PM_CHAR previously), since the clock is drawn at exactly one screen position and changes at most
// twice a day. cst-time.c's displayTime() rewrites this slot only when the AM/PM state actually
// differs from what was last drawn (mirrors cst-battery.c's printBattery() change-detection); the
// only thing setupLCD() itself does on mode entry is invalidateAmPmChar() (cst-time.h), so
// displayTime()'s own guaranteed same-pass call is the sole writer of the bitmap - see cst-time.c.
#define AMPM_CHAR                3

// LOAD button (UP/DOWN/MENU/SEL BTN = LOAD) OFF/OPLOAD/PRLOAD glyph - permanently reserved, loaded
// on both LCD_MAIN* and LCD_OPS* in both DISPLAY states (freed by the AM/PM collapse above, which
// retired the separate PM_CHAR slot this used to borrow). Firmware-restricted to at most one of the
// four buttons at a time (loadUsedElsewhere() in cst-functions.c), so this one dynamic slot is never
// asked to represent two different states at once. setupLoadChar() rewrites this slot's bitmap every
// render pass (like the AIRBRAKE gauge needle), NOT via setupLCD()'s mode-guarded reload, so it
// tracks the button's live 3-way state - setupLCD() never writes this slot in any mode. The
// loadActive()/loadEligible() runtime gate (mrbw-cst.c) is kept regardless of this slot now being
// unconditionally safe - CONFIG FUNC's own availability rule (SPEED enabled, TYPE models the load
// CVs) is a product decision, not a CGRAM-safety workaround, and LOAD is deliberately left going
// inert if DISPLAY later drops to CLOCK, matching already-shipped behavior.
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
