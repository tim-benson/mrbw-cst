#ifndef _CST_COMMON_H_
#define _CST_COMMON_H_

#define LOCO_ADDRESS_SHORT 0x8000

// CGRAM slots are reused per LcdMode: slots 6/7 - PSI_CHAR_L/R under LCD_DEFAULT (AIRBRAKE unit
// glyph), BELL_CHAR/HORN_CHAR under LCD_DIAGS, slot 5 - PLUSMINUS_CHAR under LCD_SPEED_ADJ (SPEED
// CFG ACCEL/DECEL adjust labels, borrowing the AUX slot which is main-screen-only). MAIN_SCREEN and
// OPS_MODE_SCREEN use two genuinely separate palettes (LCD_MAIN / LCD_MAIN_SPEED and LCD_OPS /
// LCD_OPS_SPEED respectively, the DISPLAY pref picking the _SPEED variant of whichever), and each
// screen's four button corners share a dynamic pool of CGRAM slots among the plain hollow/filled
// softkey circle and every icon-bearing special button function (AIRBRAKE, LOAD, CLOCK, and any
// added later) - see "Button-corner glyph pool" below for the full architecture and why it can never
// run out regardless of how many special functions exist. LCD_MAIN(_SPEED) separately, statically
// loads slot 7 - OPS_FN_ACTIVE_CHAR (the "Fn active" reminder glyph, borrowing PSI_CHAR_R); LCD_OPS
// (_SPEED) does not (OPS_MODE_SCREEN never draws that reminder), folding slot 7 into its pool
// instead. Both palettes load slot 3 - AMPM_CHAR (non-speed) or SPEED_H_CHAR (speed variant, the
// narrow "H" of MPH/KMH) - the SPEED readout and the AM/PM clock indicator are mutually exclusive,
// the DISPLAY pref shows one or the other. All 8 slots (0-7) - GAUGE_CHAR_A0-A3/B0-B3 under
// LCD_AIRBRAKE_ALT (the analogue gauge dial face, one custom char per 5x8 cell of a 2-row x 4-col
// canvas - no room left for anything else while this mode is active). setupLCD() reloads the static
// ones on every mode change; the gauge's cells, the pool slots, and AMPM_CHAR are additionally
// rewritten from the main render loop (setupGaugeChars()/allocateSpecialGlyphSlots() in mrbw-cst.c,
// the change-detected block in cst-time.c's displayTime()) since their content moves/changes live
// rather than only on a mode change.

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

// --- Button-corner glyph pool (MAIN_SCREEN / OPS_MODE_SCREEN) ---
//
// AIRBRAKE_GLYPH_CHAR / CLOCK_PEEK_GLYPH_CHAR / LOAD_CHAR (the "A" glyph on an AIRBRAKE-bound button
// corner, the clock-face glyph on a CLOCK-bound one, and the LOAD button's OFF/OPLOAD/PRLOAD glyph)
// have no fixed slot number of their own - a button can be independently configured to any of
// AIRBRAKE/LOAD/CLOCK, and any combination may be simultaneously active across the four buttons, so
// which physical slot represents which concept is decided fresh every render pass by
// mrbw-cst.c's allocateSpecialGlyphSlots(). On OPS_MODE_SCREEN the plain hollow/filled softkey circle
// (FUNCTION_INACTIVE_CHAR/FUNCTION_ACTIVE_CHAR, slots 1/2) joins the same pool, for the reason
// explained below; on MAIN_SCREEN those two stay at their fixed slot numbers, since it never needs
// them to move.
//
// The underlying argument is a pigeonhole one, not a policy, and it is what makes this architecture
// permanently future-proof rather than tuned to today's specific 3 special functions: a screen that
// draws N button corners can never need more than N distinct corner bitmaps at once, because each
// button resolves to exactly one concept (plain-hollow, plain-filled, or one icon-bearing special
// function) per render pass - regardless of how many concept TYPES exist in total.
//   - MAIN_SCREEN draws only 2 corners (UP BTN/DOWN BTN - MENU BTN/SEL BTN's own corners are never
//     drawn there, only their separate OPS_FN_ACTIVE_CHAR "still latched" reminder is), so its
//     dedicated 2-slot pool {4, 6} always covers its worst case. Slots 1/2 need not join this pool -
//     2 slots already suffice for 2 buttons no matter how many special function types are ever added.
//   - OPS_MODE_SCREEN draws all 4 corners. Its non-circle pool alone ({4, 6, 7} - slot 7 is otherwise
//     unused on LCD_OPS/LCD_OPS_SPEED, since that screen never draws the OPS_FN_ACTIVE_CHAR reminder)
//     happens to exactly match today's 3 special function types, which is a coincidence, not
//     headroom: a 4th icon-bearing special function would need a 4th simultaneous slot the moment all
//     4 buttons held 4 different values. Folding slots 1 and 2 into the SAME pool ({1, 2, 4, 6, 7},
//     5 candidates) removes that ceiling entirely: with 4 buttons, at most 4 distinct concepts (out
//     of hollow, filled, and however many special functions exist) are ever needed at once, so 5
//     candidate slots always leave at least one spare, for any number of future special functions.
//
// Adding a new icon-bearing special function therefore needs no change to this pool architecture at
// all - only the same three pieces every existing one already has: an isFunctionXxx() predicate
// (cst-functions.c), a needXxx check + setupXxxChar(slot) glyph loader wired into
// allocateSpecialGlyphSlots() (mrbw-cst.c) and cst-lcd.c respectively, and a case in
// buttonCornerGlyph() to read the resulting tracking slot back.
//
// LOAD keeps its own, separate restriction to at most one button at a time
// (loadUsedElsewhere() in cst-functions.c) - unrelated to slot-number scarcity, since its bitmap
// reflects one button's live 3-way state and cannot represent two different buttons' states at once
// even given an unlimited number of slots.

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
