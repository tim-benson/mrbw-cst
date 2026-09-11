/*************************************************************************
title:    Control Stand Throttle
Authors:  Michael D. Petersen <railfan@drgw.net>
          Nathan D. Holmes <maverick@drgw.net>
File:     mrbw-cst.c
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
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/eeprom.h>
#include <avr/wdt.h>
#include <avr/sleep.h>
#include <util/delay.h>
#include <util/atomic.h>
#include <ctype.h>

#include "mrbee.h"

#include "lcd.h"

#include "cst-common.h"
#include "cst-lcd.h"
#include "cst-hardware.h"
#include "cst-eeprom.h"
#include "cst-functions.h"
#include "cst-battery.h"
#include "cst-engine.h"
#include "cst-pressure.h"
#include "cst-time.h"
#include "cst-math.h"
#include "cst-speed.h"
#include "cst-sync.h"

//#define FAST_SLEEP
#ifdef FAST_SLEEP
#warning "Fast Sleep Enabled!"
#endif


#define LONG_PRESS_10MS_TICKS             100
#define BUTTON_AUTOINCREMENT_10MS_TICKS    50
#define BUTTON_AUTOINCREMENT_ACCEL         10
#define BUTTON_AUTOINCREMENT_MINIMUM        5

#define BACKLIGHT_HOLD_DECISECS           30   // ~3s the LCD backlight lingers after the last MENU
                                               // press / return from a menu screen

// LOAD/SAVE CNF picker's N01-N20 loco-address preview settle delay - the query it fires
// (syncQuerySharedLocoAddress()) is a blocking radio round-trip, so firing it on every single UP/DOWN
// step during a fast sweep across many shared slots would freeze the throttle for a beat at each one.
// sharedQuerySettleTicks (reset on every UP/DOWN move into/within the shared range) must reach this
// many 10ms ticks with no further move before the query is allowed to fire at all - a fast sweep across
// N slots then never fires it until navigation actually comes to rest on one.
#define SHARED_QUERY_SETTLE_10MS_TICKS     60

// LOAD/SAVE CONFIG's newConfigNumber selector: 1..MAX_CONFIGS are ordinary local slots (unchanged), sitting
// above 20 shared/network CNF slots on mrbw-cabbus - a distinct value range, not repurposed local slots, so
// existing slot data is never silently reinterpreted. Positioned below slot 1 (not above slot MAX_CONFIGS)
// so shared slot 1 (displayed "N01") has exactly one local neighbor (slot 1) - UP always moves further into
// local slots 1..MAX_CONFIGS, DOWN from slot 1 reaches N01 first, continuing deeper through N02..N20 (the
// floor). Raw numeric value order intentionally does NOT match traversal direction here (see the zone-aware
// UP_BUTTON/DOWN_BUTTON handling in the picker) - local slots keep values 1..MAX_CONFIGS completely
// unchanged, so no existing CONFIG_OFFSET()/local-slot code needed to change. See CLAUDE.md, "Shared
// network CNF store".
#define SHARED_CONFIG_BASE                (MAX_CONFIGS + 1)                          // N01 = entry 0
#define SHARED_CONFIG_COUNT               20
#define SHARED_CONFIG_MAX                 (SHARED_CONFIG_BASE + SHARED_CONFIG_COUNT - 1)  // N20 = entry 19
#define IS_SHARED_CONFIG(n)                ((n) >= SHARED_CONFIG_BASE)
#define SHARED_CONFIG_ENTRY(n)             ((n) - SHARED_CONFIG_BASE)                // 0-19

#define SLEEP_TMR_RESET_VALUE_MIN           1
#define SLEEP_TMR_RESET_VALUE_DEFAULT       5
#define SLEEP_TMR_RESET_VALUE_MAX          99

// Alerter timer in units of 1/4 minute (15 seconds), zero is off
#define ALERTER_TMR_RESET_VALUE_MIN         0
#define ALERTER_TMR_RESET_VALUE_DEFAULT     0
#define ALERTER_TMR_RESET_VALUE_MAX        60

#define IS_ALERTER_ENABLED                 (0 != alerter_tmr_reset_value)

#define TX_HOLDOFF_MIN                     10
#define TX_HOLDOFF_DEFAULT                 15
#define TX_HOLDOFF_MAX                    254   // one below the 0xFF erased-byte sentinel (see readConfig)

#define UPDATE_DECISECS_MIN                10
#define UPDATE_DECISECS_DEFAULT            10
#define UPDATE_DECISECS_MAX                100

#define MRBUS_DEV_ADDR_MIN                 0x30
#define MRBUS_DEV_ADDR_DEFAULT             0x30
#define MRBUS_DEV_ADDR_MAX                 0x49

#define MRBUS_BASE_ADDR_MIN                0xD0
#define MRBUS_BASE_ADDR_DEFAULT            0xD0
#define MRBUS_BASE_ADDR_MAX                0xEF

#define TIME_SOURCE_ADDRESS_DEFAULT        0x00

#define RESET_COUNTER_RESET_VALUE   5
uint8_t resetCounter = RESET_COUNTER_RESET_VALUE;

// 5 sec timeout for packets from base, base transmits every 1 sec
#define PKT_TIMEOUT_DECISECS   50
volatile uint8_t pktTimeout = 0;

uint32_t baseVersion;
char baseString[9];

#define STATUS_READ_SWITCHES          0x01

#define HORN_CONTROL      0x01
#define BELL_CONTROL      0x02
#define AUX_CONTROL       0x04
#define BRAKE_CONTROL     0x08
#define BRAKE_REL_CONTROL 0x10
#define THR_UNLK_CONTROL  0x80
// controls widened to uint16_t for this bit - the low byte (0x01-0x80) is fully allocated
#define HORN2_CONTROL     0x100

#define HORN_HYSTERESIS   5
#define BRAKE_HYSTERESIS  5

// STACK combo brake mode: two selectable variants, 5-STEP (6 equal bands, the original design) and
// 3-STEP (4 equal bands, added for users who find 5-STEP's independent per-step combos too complex) -
// see OPTIONBITS_STACK_5STEP below for the toggle. Tables rather than hardcoded conditionals, since
// boundaries are expected to become uneven later.
#define STACK_BAND_COUNT_3STEP 4
#define STACK_BAND_COUNT_5STEP 6
static const uint8_t stackBandThresholds3Step[STACK_BAND_COUNT_3STEP - 1] = { 25, 50, 75 };
static const uint8_t stackBandThresholds5Step[STACK_BAND_COUNT_5STEP - 1] = { 17, 33, 50, 67, 83 };

// Brake1 in STACK mode reuses the existing BRAKE_CONTROL/BRAKE_FN plumbing (same DCC function number as
// standard/pulse/stepped mode's brake), same idea as STACK's Band 0 reusing BRAKE_REL_CONTROL/BRAKE_REL_FN.
// Brake2/Brake3 use controls' 2 remaining free bits - no separate control byte needed.
#define BK2_CONTROL 0x20
#define BK3_CONTROL 0x40

// The STACK band->combo storage encoding (and its factory defaults) live in cst-eeprom.h so the
// layout-1->2 migration in cst-eeprom.c can name them. A stored combo byte is OR'd straight into
// `controls`, so the two must agree bit-for-bit:
_Static_assert(BRAKE_CONTROL == STACK_COMBO_BRK1 && BK2_CONTROL == STACK_COMBO_BRK2
               && BK3_CONTROL == STACK_COMBO_BRK3,
               "STACK combo storage bits must match the controls BRAKE/BK2/BK3 bits");

// Band->combo mapping, configurable on-device via OPTION_SCREEN (bands 1-3/1-5; band 0 is fixed to
// "none", not stored/editable). Populated from EE_STACK_BAND_COMBOS_3STEP/EE_STACK_BAND_COMBOS by
// readConfig(); see resetConfig() for factory defaults (STACK_*STEP_DEFAULT_* in cst-eeprom.h).
// Stored completely separately per variant, so toggling OPTIONBITS_STACK_5STEP back and forth never
// cross-contaminates one variant's configured combos with the other's.
uint8_t stackBandCombos3Step[STACK_BAND_COUNT_3STEP];
uint8_t stackBandCombos5Step[STACK_BAND_COUNT_5STEP];

// OPTION_SCREEN's STACK band-editor UP/DOWN cycle order: none -> each single brake -> each pair -> all three,
// rather than a raw binary count - a more intuitive progression for a human turning the dial.
static const uint8_t stackComboSequence[8] = {
	0x00,                                       // ---
	BRAKE_CONTROL,                              // 1--
	BK2_CONTROL,                                // -2-
	BK3_CONTROL,                                // --3
	BRAKE_CONTROL | BK2_CONTROL,                // 12-
	BRAKE_CONTROL | BK3_CONTROL,                // 1-3
	BK2_CONTROL | BK3_CONTROL,                  // -23
	BRAKE_CONTROL | BK2_CONTROL | BK3_CONTROL,  // 123
};

uint8_t currentStackBand = 0;  // sticky band, for hysteresis

// BRAKE_PULSE_WIDTH is in decisecs
// It is the minimum on time for the pulsed brake
#define BRAKE_PULSE_WIDTH_MIN       2
#define BRAKE_PULSE_WIDTH_DEFAULT   5
#define BRAKE_PULSE_WIDTH_MAX      10

uint8_t brakePulseWidth = BRAKE_PULSE_WIDTH_DEFAULT;

// Boolean config bits (EEPROM, global)
#define CONFIGBITS_LED_BLINK         0
// Bit clear = main screen shows the clock (the pre-fork default - preserved for throttles upgrading
// from stock firmware, whose stored config byte already has this bit clear); set = show scale speed.
#define CONFIGBITS_MAIN_SCREEN_SPEED 1
// Bit clear = AIRBRAKE off (default - the air-brake model still ticks but drives nothing, AIRBRAKE CFG
// is hidden); set = AIRBRAKE drives BRAKE_REL_FN / BRK SET / COMPRESSOR_FN. See CLAUDE.md "AIRBRAKE".
#define CONFIGBITS_AIRBRAKE          2
// Bit clear = OPS MODE off (default - MENU/SELECT are navigation keys, base screen is stock, the
// OPS MODE screen and MENU BTN / SEL BTN functions are inert); set = a long-press of MENU on the
// base screen enters the OPS MODE screen, where MENU/SELECT drive MENU BTN / SEL BTN. See CLAUDE.md.
#define CONFIGBITS_OPS_MODE          3
#define CONFIGBITS_REVERSER_LOCK     4
#define CONFIGBITS_STRICT_SLEEP      5

// AIRBRAKE stops asserting its non-latching sound functions (COMPRESSOR_FN/COMPRESSOR2_FN / BRK SET)
// this many decisecs before the throttle sleeps, so a final "off" reaches the loco while packets
// still flow - otherwise the command station holds the last state it heard and the compressor sound
// plays forever.
#define AIRBRAKE_SLEEP_QUIET_DECISECS   3

#define CONFIGBITS_DEFAULT                 (_BV(CONFIGBITS_LED_BLINK) | _BV(CONFIGBITS_REVERSER_LOCK) | _BV(CONFIGBITS_STRICT_SLEEP))
uint8_t configBits = CONFIGBITS_DEFAULT;

// Boolean option bits (EEPROM, per config)
#define OPTIONBITS_ESTOP_ON_BRAKE    0
#define OPTIONBITS_REVERSER_SWAP     1
#define OPTIONBITS_VARIABLE_BRAKE    2

// BRK TYPE is a 2-bit field (bits 3-4) selecting the variable-brake variant, replacing what used
// to be the single boolean OPTIONBITS_STEPPED_BRAKE bit.
#define OPTIONBITS_BRK_TYPE_LSB      3
#define OPTIONBITS_BRK_TYPE_MASK     (0x03 << OPTIONBITS_BRK_TYPE_LSB)
#define BRK_TYPE_PULSE  0
#define BRK_TYPE_STEP   1
#define BRK_TYPE_STACK  2
#define GET_BRK_TYPE(bits)          (((bits) >> OPTIONBITS_BRK_TYPE_LSB) & 0x03)
#define SET_BRK_TYPE(bits, val)     ((bits) = ((bits) & ~OPTIONBITS_BRK_TYPE_MASK) | (((val) & 0x03) << OPTIONBITS_BRK_TYPE_LSB))

// STACK mode's step-count toggle: 0 = 3-STEP (default, for every device - including ones already flashed
// with a 5-STEP config, since no prior firmware ever wrote this bit), 1 = 5-STEP.
#define OPTIONBITS_STACK_5STEP       5

// Horn2 mode: 0 = Additive (default - Horn2 stacks on top of Horn1 past its own threshold),
// 1 = Exclusive (Horn2 replaces Horn1 past its own threshold). See evaluation near HORN2_CONTROL's use.
#define OPTIONBITS_HORN_TYPE         6

#define OPTIONBITS_DEFAULT                 (_BV(OPTIONBITS_ESTOP_ON_BRAKE))
uint8_t optionBits = OPTIONBITS_DEFAULT;

// Boolean system bits (volatile, global)
#define SYSTEMBITS_MENU_LOCK         0
#define SYSTEMBITS_ADV_FUNC          1

#define SYSTEMBITS_DEFAULT                 0x00
uint8_t systemBits = SYSTEMBITS_DEFAULT;


#define MRBUS_TX_BUFFER_DEPTH 16
#define MRBUS_RX_BUFFER_DEPTH 8

MRBusPacket mrbusTxPktBufferArray[MRBUS_TX_BUFFER_DEPTH];
MRBusPacket mrbusRxPktBufferArray[MRBUS_RX_BUFFER_DEPTH];

uint8_t mrbus_dev_addr = 0;
uint8_t mrbus_base_addr = 0;

uint8_t lastRSSI = 0xFF;

uint16_t locoAddress = 0;

#define BRAKE_DEAD_ZONE 5

uint8_t hornThreshold;
uint8_t hornThreshold2;
uint8_t brakeThreshold;
uint8_t brakeLowThreshold;
uint8_t brakeHighThreshold;

volatile uint8_t brakeCounter;

uint8_t notchSpeedStep[8];

volatile uint16_t button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint16_t ticks_autoincrement = BUTTON_AUTOINCREMENT_10MS_TICKS;
volatile uint8_t sharedQuerySettleTicks = SHARED_QUERY_SETTLE_10MS_TICKS;

volatile uint8_t ticks;
volatile uint16_t decisecs = 0;
volatile uint16_t sleepTimeout_decisecs = 0;
volatile uint16_t alerterTimeout_decisecs = 0;
volatile uint8_t backlightTimeout_decisecs = 0;  // uint8_t -> atomic on AVR, no ATOMIC_BLOCK needed
volatile uint8_t txHoldoff = 0;
volatile uint8_t status = 0;

uint16_t update_decisecs = UPDATE_DECISECS_DEFAULT;
uint8_t txHoldoff_centisecs = TX_HOLDOFF_DEFAULT;

static uint8_t timeSourceAddress = 0xFF;

#define THROTTLE_STATUS_SLEEP           0x80
#define THROTTLE_STATUS_ALERTER         0x40
#define THROTTLE_STATUS_ALL_STOP        0x02
#define THROTTLE_STATUS_EMERGENCY       0x01

volatile uint8_t throttleStatus = 0;

uint8_t estopStatus = 0;

#define ESTOP_BRAKE    0x01
#define ESTOP_BUTTON   0x02
#define ESTOP_ALERTER  0x04

uint16_t sleep_tmr_reset_value;
uint16_t alerter_tmr_reset_value;

// Define the menu screens and menu order
// Must end with LAST_SCREEN
typedef enum
{
	MAIN_SCREEN = 0,
	ENGINE_SCREEN,
	AIRBRAKE_SCREEN,        // was AIR_GAUGE_SCREEN
	LOAD_CONFIG_SCREEN,
	SAVE_CONFIG_SCREEN,
	LOCO_SCREEN,
	FORCE_FUNC_SCREEN,
	CONFIG_FUNC_SCREEN,
	NOTCH_CONFIG_SCREEN,
	SPEED_CONFIG_SCREEN,
	AIRBRAKE_CONFIG_SCREEN, // was BRAKE_CONFIG_SCREEN
	OPTION_SCREEN,
	SYSTEM_SCREEN,
	COMM_SCREEN,
	PREFS_SCREEN,
	THRESHOLD_CAL_SCREEN,
	DIAG_SCREEN,
	OPS_MODE_SCREEN,  // OPS MODE base-screen variant - not in the MENU cycle; entered/left only by a long-press of MENU
	LAST_SCREEN  // Must be the last screen
} Screens;

// PREFS_SCREEN items, in on-screen (subscreenState - 1) order. Three kinds: a configBits bit
// (DISPLAY/AIRBRAKE/LED_BLINK/REV_LOCK/STRICT_SLEEP - see prefsItemIsBit()), a staged uint8_t
// (SLEEP/ALERTER, held in new* locals and only pushed to the real values on SELECT-save), and
// the one opaque item (TIMEOUT, edited through the cst-time.c increment/decrement helpers).
// Named-item + switch pattern, replacing the old if(N == subscreenState) chain, the bitPosition
// sentinel byte and the prefsPtr scratch pointer - same direction as SPEED CFG / AIRBRAKE CFG.
// COMM_SCREEN and SYSTEM_SCREEN follow the same pattern (COMM_ITEM_* / SYSTEM_ITEM_* below).
enum
{
	PREFS_ITEM_DISPLAY = 0,   // configBits: CLOCK / SPEED main-screen readout
	PREFS_ITEM_OPS_MODE,     // configBits: OPS MODE base-screen variant on/off
	PREFS_ITEM_AIRBRAKE,      // configBits: AIRBRAKE sound model on/off
	PREFS_ITEM_SLEEP,         // staged: newSleepTimeout (minutes)
	PREFS_ITEM_ALERTER,       // staged: newAlerterTimeout (x15 s, 0 = OFF)
	PREFS_ITEM_TIMEOUT,       // opaque: fast-clock dead-reckoning timeout (cst-time.c)
	PREFS_ITEM_LED_BLINK,     // configBits
	PREFS_ITEM_REV_LOCK,      // configBits
	PREFS_ITEM_STRICT_SLEEP,  // configBits
	PREFS_ITEM_COUNT
};

// COMM_SCREEN items, in on-screen order. Every item edits one uint8_t held elsewhere -
// the new* staging locals (THRTL_ID/BASE_ADR/TIME_ADR/TX_INTVL) or the live txHoldoff_centisecs
// global (TX_HLDOF). TX_INTVL and TX_HLDOF are view-only unless ADV FUNC (see commItemIsAdvGated()).
enum
{
	COMM_ITEM_THRTL_ID = 0,  // newDevAddr,      MRBUS_DEV_ADDR_MIN..MAX,  shown A..Z
	COMM_ITEM_BASE_ADR,      // newBaseAddr,     MRBUS_BASE_ADDR_MIN..MAX
	COMM_ITEM_TIME_ADR,      // newTimeAddr,     0..255 (0 = BASE, 0xFF = ALL, else 0xNN)
	COMM_ITEM_TX_INTVL,      // newUpdate_seconds, 1..UPDATE_DECISECS_MAX/10, seconds
	COMM_ITEM_TX_HLDOF,      // txHoldoff_centisecs, TX_HOLDOFF_MIN..TX_HOLDOFF_MAX, shown N.NN s
	COMM_ITEM_COUNT
};

// SYSTEM_SCREEN items, in on-screen order: two systemBits toggles then the three battery
// thresholds (decivolts, view-only unless ADV FUNC, applied live through setBatteryLevels()).
enum
{
	SYSTEM_ITEM_MENU_LOCK = 0,  // systemBits
	SYSTEM_ITEM_ADV_FUNC,       // systemBits
	SYSTEM_ITEM_BAT_OKAY,       // decivolts
	SYSTEM_ITEM_BAT_WARN,       // decivolts
	SYSTEM_ITEM_BAT_CRIT,       // decivolts
	SYSTEM_ITEM_COUNT
};

// OPTION_SCREEN logical items. Unlike the other config screens, the item at a given
// subscreenState depends on the brake mode: STACK inserts (stackBandCount() - 1) band-editor
// items between BRK TYPE and BRK ESTP, and subscreenState 3 is BRK RATE outside STACK but STEPS
// in it. optionItemAt() resolves subscreenState -> item (and, for STACK_BAND, the band number),
// replacing the old estopItem/revSwapItem/hornTypeItem arithmetic and the 0xFB..0xFE bitPosition
// sentinels.
enum
{
	OPTION_ITEM_VAR_BRK = 0,  // optionBits OPTIONBITS_VARIABLE_BRAKE
	OPTION_ITEM_BRK_TYPE,     // 3-way PULSE/STEP/STACK via GET/SET_BRK_TYPE
	OPTION_ITEM_BRK_RATE,     // brakePulseWidth (0.N s) - subscreenState 3 when not STACK
	OPTION_ITEM_STEPS,        // STACK 3-STEP/5-STEP toggle - subscreenState 3 in STACK
	OPTION_ITEM_STACK_BAND,   // STACK band->combo editor - subscreenState 4..3+bands, STACK only
	OPTION_ITEM_BRK_ESTP,     // optionBits OPTIONBITS_ESTOP_ON_BRAKE
	OPTION_ITEM_REV_SWAP,     // optionBits OPTIONBITS_REVERSER_SWAP
	OPTION_ITEM_HORNTYPE,     // optionBits OPTIONBITS_HORN_TYPE, deterministic set
	OPTION_ITEM_NONE          // subscreenState past the last item -> wrap to 1
};

typedef enum
{
	NO_BUTTON = 0,
	MENU_BUTTON,
	SELECT_BUTTON,
	UP_BUTTON,
	DOWN_BUTTON,
} Buttons;

Buttons button = NO_BUTTON;
Buttons previousButton = NO_BUTTON;
uint8_t buttonCount = 0;

typedef enum
{
	BRAKE_LOW_BEGIN,
	BRAKE_LOW_WAIT,
	BRAKE_20PCNT_BEGIN,
	BRAKE_20PCNT_WAIT,
	BRAKE_40PCNT_BEGIN,
	BRAKE_40PCNT_WAIT,
	BRAKE_60PCNT_BEGIN,
	BRAKE_60PCNT_WAIT,
	BRAKE_80PCNT_BEGIN,
	BRAKE_80PCNT_WAIT,
	BRAKE_FULL_BEGIN,
	BRAKE_FULL_WAIT,
} BrakeStates;

uint32_t functionForceOn  = 0;
uint32_t functionForceOff = 0;

#define UP_OPTION_BUTTON   0x01
#define DOWN_OPTION_BUTTON 0x02
#define MENU_OPTION_BUTTON 0x04   // OPS MODE only - the MENU button driving MENU_FN
#define SEL_OPTION_BUTTON  0x08   // OPS MODE only - the SELECT button driving SEL_FN

uint16_t controls = 0;

// Commanded DCC speed step (0-126, magnitude only), mirrored here so the speed simulation's
// 10Hz hook can see it - activeThrottleSetting/notchSpeedStep[] are main()-locals. Updated once
// per main-loop pass; see the "Calculate active throttle setting" block below.
volatile uint8_t commandedSpeedStep = 0;

// Set by TIMER0_COMPA_vect every 100ms; the main loop consumes it and runs updateSpeed10Hz() from
// there (not the ISR) - its standing-start ramp does 64-bit math that must not stall other interrupts.
volatile uint8_t speed10HzTick = 0;

// Same idea for the AIRBRAKE model - updateBrake10Hz() runs from the main loop so it can
// take the lever percentage as a parameter rather than reaching into main()'s locals from the ISR.
volatile uint8_t brake10HzTick = 0;

// Is the user's SPEED "watched" DCC function (STOPFN) currently part of the outgoing functionMask
// - regardless of which physical control put it there? functionMask itself is a main()-local, not
// visible from the ISR, so this is computed once per main-loop pass right after functionMask is
// finalized (see the "Force specific functions on or off" block below) and mirrored here.
volatile uint8_t stopFunctionActive = 0;

// Same watching mechanism as stopFunctionActive above, for the SPEED OPLOADFN/PRLOADFN - is the
// user's watched Optional/Primary Load DCC function currently part of the outgoing functionMask?
volatile uint8_t oploadFunctionActive = 0;
volatile uint8_t prloadFunctionActive = 0;

// Same watching mechanism again, for the SPEED HOLDFN (ESU Drive Hold) - is the user's watched
// DCC function currently part of the outgoing functionMask? Defaults to F09, not OFF - see cst-speed.h.
volatile uint8_t holdFunctionActive = 0;

// Same watching mechanism again, for Brake1/2/3 (BRAKE_FN/BK2_FN/BK3_FN) - is each one's configured DCC
// function currently part of the outgoing functionMask, regardless of which physical control (the brake
// lever's own state machine, or anything else mapped to the same function number) put it there? Forced
// false whenever BRK TYPE == STEP - Step's brake pulses advance a TCS-style ratchet on the real decoder,
// not "hold to brake at rate X", which the speed simulation's held-while-active model was never a valid
// representation of; see the watching block below.
volatile uint8_t brake1FunctionActive = 0;
volatile uint8_t brake2FunctionActive = 0;
volatile uint8_t brake3FunctionActive = 0;

// STACK 3-STEP/5-STEP: single shared hysteresis-walk algorithm (evaluateStackBrake() below), parameterized
// by these 3 small accessors rather than duplicated per variant - the two variants are structurally
// identical (an N-band walk over a threshold table, ending in an array lookup), differing only in which
// table/array they use.
static uint8_t stackIs5Step(void)
{
	return (optionBits & _BV(OPTIONBITS_STACK_5STEP)) ? 1 : 0;
}
static uint8_t stackBandCount(void)
{
	return stackIs5Step() ? STACK_BAND_COUNT_5STEP : STACK_BAND_COUNT_3STEP;
}
static const uint8_t* stackThresholds(void)
{
	return stackIs5Step() ? stackBandThresholds5Step : stackBandThresholds3Step;
}
static uint8_t* stackCombos(void)
{
	return stackIs5Step() ? stackBandCombos5Step : stackBandCombos3Step;
}

// SPEED_CONFIG_SCREEN: the four watched-DCC-function items (HOLDFN/STOPFN/OPLOADFN/PRLOADFN) share one
// edit behaviour - an 0-28 range plus the SPEED_STOP_WATCH_FN_OFF (255) sentinel, distinct from the plain
// 0-255 numeric items and the two 0/1 toggles (UNIT/TYPE). Not contiguous in the SPEED_ITEM_* enum (menu
// order now comes from the per-TYPE descriptor in cst-speed.c, resolved by speedItemAt()), so this is a
// set test rather than a range check.
static uint8_t speedItemIsWatchFn(uint8_t item)
{
	return (SPEED_ITEM_HOLD_FN == item) || (SPEED_ITEM_STOP_FN == item)
	    || (SPEED_ITEM_OPLOAD_FN == item) || (SPEED_ITEM_PRLOAD_FN == item);
}

// SPEED_CONFIG_SCREEN: ACCELADJ/DECELADJ (CV23/CV24) are shown and edited as a signed value; the stored
// byte carries the ESU sign-bit encoding (bit 7 = subtract, bits 0-6 = magnitude).
static uint8_t speedItemIsSignedAdjust(uint8_t item)
{
	return (SPEED_ITEM_ACCEL_ADJ == item) || (SPEED_ITEM_DECEL_ADJ == item);
}

// SPEED_CONFIG_SCREEN: ACCEL/DECEL are genuine 0-255 fields (read raw, so a stored 0xFF is a real 255);
// the other plain-numeric items still self-heal from 0xFF so their editor ceiling is 254 (matching the
// AIRBRAKE editor - a saved 255 would silently revert on the next load).
static uint8_t speedItemIsFullRange(uint8_t item)
{
	return (SPEED_ITEM_ACCEL == item) || (SPEED_ITEM_DECEL == item);
}
static int8_t speedAdjDecode(uint8_t b)
{
	return (b & 0x80) ? (int8_t)(-(int8_t)(b & 0x7F)) : (int8_t)(b & 0x7F);
}
static uint8_t speedAdjEncode(int8_t v)
{
	return (v < 0) ? (uint8_t)(0x80 | (uint8_t)(-v)) : (uint8_t)v;
}

// PREFS_SCREEN: the three value items (SLEEP/ALERTER/TIMEOUT) are contiguous in the enum;
// every other item toggles the configBits bit given by prefsItemBit().
static uint8_t prefsItemIsBit(uint8_t item)
{
	return (item < PREFS_ITEM_SLEEP) || (item > PREFS_ITEM_TIMEOUT);
}
static uint8_t prefsItemBit(uint8_t item)
{
	switch(item)
	{
		case PREFS_ITEM_OPS_MODE:     return CONFIGBITS_OPS_MODE;
		case PREFS_ITEM_AIRBRAKE:     return CONFIGBITS_AIRBRAKE;
		case PREFS_ITEM_LED_BLINK:    return CONFIGBITS_LED_BLINK;
		case PREFS_ITEM_REV_LOCK:     return CONFIGBITS_REVERSER_LOCK;
		case PREFS_ITEM_STRICT_SLEEP: return CONFIGBITS_STRICT_SLEEP;
		default:                      return CONFIGBITS_MAIN_SCREEN_SPEED;  // PREFS_ITEM_DISPLAY
	}
}

// COMM_SCREEN: TX INTVL and TX HLDOF are only editable with ADV FUNC on.
static uint8_t commItemIsAdvGated(uint8_t item)
{
	return (COMM_ITEM_TX_INTVL == item) || (COMM_ITEM_TX_HLDOF == item);
}

// SYSTEM_SCREEN: the two systemBits toggles vs. the three battery-threshold values.
static uint8_t systemItemIsBit(uint8_t item)
{
	return (item < SYSTEM_ITEM_BAT_OKAY);
}
static uint8_t systemItemBit(uint8_t item)
{
	return (SYSTEM_ITEM_ADV_FUNC == item) ? SYSTEMBITS_ADV_FUNC : SYSTEMBITS_MENU_LOCK;
}

// OPTION_SCREEN: resolve a 1-based subscreenState to its OPTION_ITEM_*, given the live brake mode.
// *band receives the STACK band (1..stackBandCount()-1) for OPTION_ITEM_STACK_BAND, 0 otherwise.
static uint8_t optionItemAt(uint8_t ss, uint8_t *band)
{
	*band = 0;
	uint8_t stackMode = (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) &&
	                    (BRK_TYPE_STACK == GET_BRK_TYPE(optionBits));
	uint8_t estopItem = 4 + (stackMode ? (stackBandCount() - 1) : 0);

	if(1 == ss) return OPTION_ITEM_VAR_BRK;
	if(2 == ss) return OPTION_ITEM_BRK_TYPE;
	if(3 == ss) return stackMode ? OPTION_ITEM_STEPS : OPTION_ITEM_BRK_RATE;
	if((ss >= 4) && (ss < estopItem)) { *band = ss - 3; return OPTION_ITEM_STACK_BAND; }
	if(ss == estopItem)     return OPTION_ITEM_BRK_ESTP;
	if(ss == estopItem + 1) return OPTION_ITEM_REV_SWAP;
	if(ss == estopItem + 2) return OPTION_ITEM_HORNTYPE;
	return OPTION_ITEM_NONE;
}
// The optionBits bit for the three plain toggle items.
static uint8_t optionBitFor(uint8_t item)
{
	switch(item)
	{
		case OPTION_ITEM_BRK_ESTP: return OPTIONBITS_ESTOP_ON_BRAKE;
		case OPTION_ITEM_REV_SWAP: return OPTIONBITS_REVERSER_SWAP;
		default:                   return OPTIONBITS_VARIABLE_BRAKE;  // OPTION_ITEM_VAR_BRK
	}
}
// Cycle a STACK band-combo one step through stackComboSequence[] (none / singles / pairs / all).
static void optionCycleBandCombo(uint8_t band, int8_t dir)
{
	uint8_t *combos = stackCombos();
	uint8_t idx = 0;
	while((idx < 7) && (stackComboSequence[idx] != combos[band]))
		idx++;
	combos[band] = stackComboSequence[(idx + dir) & 0x07];
}

// STACK combo brake mode: stateless per loop pass (band derived fresh from brakePcnt each call), not a
// graft onto BrakeStates - that machine is deliberately asymmetric (advance-only, TCS-style) and is the
// wrong shape for a mode that must track the lever symmetrically in both directions.
void evaluateStackBrake(uint8_t brakePcnt)
{
	uint8_t pcnt = (brakePcnt > 100) ? 100 : brakePcnt;
	uint8_t bandCount = stackBandCount();
	const uint8_t *thresholds = stackThresholds();
	uint8_t *combos = stackCombos();

	// Clamp a leftover band from the other variant (e.g. 4, valid only in 5-STEP) so the walk below can
	// never index either table out of bounds right after the toggle switches variants.
	uint8_t band = (currentStackBand < bandCount) ? currentStackBand : (bandCount - 1);

	// Escalate immediately on crossing a boundary going up.
	while(band < bandCount - 1 && pcnt >= thresholds[band])
		band++;
	// De-escalate only once BRAKE_HYSTERESIS below that same boundary coming back down.
	while(band > 0 && pcnt < ((thresholds[band - 1] > BRAKE_HYSTERESIS) ?
	                           (thresholds[band - 1] - BRAKE_HYSTERESIS) : 0))
		band--;

	currentStackBand = band;

	// Brake1/2/3 combo bits all live directly in controls - Brake1 reuses BRAKE_CONTROL, so this
	// touches only the 3 combo bits, leaving BRAKE_REL_CONTROL (set below) and any other bits alone.
	controls = (controls & ~(BRAKE_CONTROL | BK2_CONTROL | BK3_CONTROL)) | combos[band];

	// BRAKE_REL_FN behaves like standard/pulse mode here (continuous hold), not stepped mode's
	// one-tick pulse - reuses the existing controls bit/function, no new plumbing needed.
	if(0 == band)
		controls |= BRAKE_REL_CONTROL;
	else
		controls &= ~BRAKE_REL_CONTROL;
}

#define ENGINE_TIMER_DECISECS      20
volatile uint8_t engineTimer = 0;

#define ENGINE_SCREEN_TIMER       100
volatile uint8_t engineScreenTimer = 0;

uint8_t debounce(uint8_t debouncedState, uint8_t newInputs)
{
	static uint8_t clock_A=0, clock_B=0;
	uint8_t delta = newInputs ^ debouncedState;   //Find all of the changes
	uint8_t changes;

	clock_A ^= clock_B;                     //Increment the counters
	clock_B  = ~clock_B;

	clock_A &= delta;                       //Reset the counters if no changes
	clock_B &= delta;                       //were detected.

	changes = ~((~delta) | clock_A | clock_B);
	debouncedState ^= changes;
	return(debouncedState);
}


void parseVersionStr(uint8_t* major, uint8_t* minor, uint8_t* delta)
{
	char version_string[] = VERSION_STRING;
	char *ptr = version_string;
	uint8_t version = 0;

	*major = *minor = *delta = 0;

	// Skip the "X" fork-name prefix before parsing major.minor.delta - see git-revision.sh, which tags
	// this fork's releases as "X<major>.<minor>". Only alphabetic characters are skipped (not '.') so
	// the untagged/degenerate VERSION_STRING ".0+" (no matching git tag found at all) still falls
	// through to the existing "empty major defaults to 0" behavior below, rather than misparsing '.'
	// as part of a value.
	while(isalpha((unsigned char)*ptr))
		ptr++;

	while(('.' != *ptr) && ('\0' != *ptr))
	{
		version *= 10;
		version += *ptr - '0';
		ptr++;
	}
	*major = version;
	version = 0;

	if('.' == *ptr)
	{
		ptr++;
	}

	while(('.' != *ptr) && ('\0' != *ptr))
	{
		version *= 10;
		version += *ptr - '0';
		ptr++;
	}

	*minor = version;
	version = 0;

	if('.' == *ptr)
	{
		ptr++;
	}

	while(isdigit(*ptr))
	{
		version *= 10;
		version += *ptr - '0';
		ptr++;
	}

	*delta = version;

}


void createVersionPacket(uint8_t destAddr, uint8_t *buf)
{
	buf[MRBUS_PKT_DEST] = destAddr;
	buf[MRBUS_PKT_SRC] = mrbus_dev_addr;
	buf[MRBUS_PKT_LEN] = 18;
	buf[MRBUS_PKT_TYPE] = 'v';
	buf[6]  = MRBUS_VERSION_WIRELESS;
	// Software Revision
	buf[7]  = 0xFF & ((uint32_t)(GIT_REV))>>16; // Software Revision
	buf[8]  = 0xFF & ((uint32_t)(GIT_REV))>>8; // Software Revision
	buf[9]  = 0xFF & (GIT_REV); // Software Revision
	buf[10] = HWREV_MAJOR; // Hardware Major Revision
	buf[11] = HWREV_MINOR; // Hardware Minor Revision
	buf[12] = 'C';
	buf[13] = 'S';
	buf[14] = 'T';
	parseVersionStr(&buf[15], &buf[16], &buf[17]);
}

void PktHandler(void)
{
	uint16_t crc = 0;
	uint8_t i;
	uint8_t rxBuffer[MRBUS_BUFFER_SIZE];
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];
	uint8_t rssi;

	if (0 == mrbeePktQueuePop(&mrbeeRxQueue, rxBuffer, sizeof(rxBuffer), &rssi))
		return;

	if(mrbus_base_addr == rxBuffer[MRBUS_PKT_SRC])
		pktTimeout = PKT_TIMEOUT_DECISECS;

	//*************** PACKET FILTER ***************
	// Loopback Test - did we send it?  If so, we probably want to ignore it
	if (rxBuffer[MRBUS_PKT_SRC] == mrbus_dev_addr) 
		goto	PktIgnore;

	// Destination Test - is this for us or broadcast?  If not, ignore
	if (0xFF != rxBuffer[MRBUS_PKT_DEST] && mrbus_dev_addr != rxBuffer[MRBUS_PKT_DEST]) 
		goto	PktIgnore;
	
	// CRC16 Test - is the packet intact?
	for(i=0; i<rxBuffer[MRBUS_PKT_LEN]; i++)
	{
		if ((i != MRBUS_PKT_CRC_H) && (i != MRBUS_PKT_CRC_L)) 
			crc = mrbusCRC16Update(crc, rxBuffer[i]);
	}
	if ((UINT16_HIGH_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_H]) || (UINT16_LOW_BYTE(crc) != rxBuffer[MRBUS_PKT_CRC_L]))
		goto	PktIgnore;
		
	//*************** END PACKET FILTER ***************


	//*************** PACKET HANDLER - PROCESS HERE ***************

	// Just smash the transmit buffer if we happen to see a packet directed to us
	// that requires an immediate response
	//
	// If we're in here, then either we're transmitting, then we can't be 
	// receiving from someone else, or we failed to transmit whatever we were sending
	// and we're waiting to try again.  Either way, we're not going to corrupt an
	// in-progress transmission.
	//
	// All other non-immediate transmissions (such as scheduled status updates)
	// should be sent out of the main loop so that they don't step on things in
	// the transmit buffer
	
	if ('A' == rxBuffer[MRBUS_PKT_TYPE])
	{
		// PING packet
		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		txBuffer[MRBUS_PKT_LEN] = 6;
		txBuffer[MRBUS_PKT_TYPE] = 'a';
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('R' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// EEPROM Extended READ Packet
		// [dest][src][len][crcL][crcH]['R'] [addrL][addrH] [bytes]
		// [dest][src][len][crcL][crcH]['r'] [addrL][addrH] [rdData0] ... [rdDataN]
		uint8_t pktPtr;
		uint8_t bytesToRead = rxBuffer[8];

		// Reset the timeout if we're doing read/write stuff
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			sleepTimeout_decisecs = sleep_tmr_reset_value;
		}

		txBuffer[MRBUS_PKT_DEST] = rxBuffer[MRBUS_PKT_SRC];
		txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
		if(bytesToRead > (MRBUS_BUFFER_SIZE - 8))
			bytesToRead = MRBUS_BUFFER_SIZE - 8;
		txBuffer[MRBUS_PKT_LEN] = 8 + bytesToRead;
		txBuffer[MRBUS_PKT_TYPE] = 'r';
		txBuffer[6] = rxBuffer[6];  // Reflect starting addr
		txBuffer[7] = rxBuffer[7];
		uint16_t eepAddr = ((uint16_t)rxBuffer[7] * 256) + rxBuffer[6];
		for(pktPtr = 0; pktPtr < bytesToRead; pktPtr++)
		{
			txBuffer[8+pktPtr] = eeprom_read_byte((uint8_t*)(eepAddr + pktPtr));
		}
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('V' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Version
		createVersionPacket(rxBuffer[MRBUS_PKT_SRC], txBuffer);
		mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		goto PktIgnore;
	}
	else if ('X' == rxBuffer[MRBUS_PKT_TYPE]) 
	{
		// Reset
		cli();
		wdt_reset();
		MCUSR &= ~(_BV(WDRF));
		WDTCSR |= _BV(WDE) | _BV(WDCE);
		WDTCSR = _BV(WDE);
		while(1);  // Force a watchdog reset
		sei();
	}
	else if ('T' == rxBuffer[MRBUS_PKT_TYPE] &&
				( (0xFF == timeSourceAddress) || 
				  (rxBuffer[MRBUS_PKT_SRC] == timeSourceAddress) ||
				  ((rxBuffer[MRBUS_PKT_SRC] == mrbus_base_addr) && (0 == timeSourceAddress)) )
		)
	{
		// It's a time packet from our time reference source
		processTimePacket(rxBuffer);
	}
	else if ( ('V' == (rxBuffer[MRBUS_PKT_TYPE] & 0xDF)) &&
		(mrbus_base_addr == rxBuffer[MRBUS_PKT_SRC]) )
	{
		// It's a version packet from our assigned base station
		lastRSSI = rssi;
		baseVersion = ((uint32_t)rxBuffer[7] << 16) | ((uint16_t)rxBuffer[8] << 8) | rxBuffer[9];
		memset(baseString,' ', 8);  // Fill with spaces before copying string
		baseString[8] = 0;  // NULL terminate
		memcpy(baseString, &rxBuffer[12], min(rxBuffer[MRBUS_PKT_LEN]-12, 8));
	}
	//*************** END PACKET HANDLER  ***************

	
	//*************** RECEIVE CLEANUP ***************
PktIgnore:
	// Yes, I hate gotos as well, but sometimes they're a really handy and efficient
	// way to jump to a common block of cleanup code at the end of a function 

	// This section resets anything that needs to be reset in order to allow us to receive
	// another packet.  Typically, that's just clearing the MRBUS_RX_PKT_READY flag to 
	// indicate to the core library that the mrbus_rx_buffer is clear.
	return;	
}


ISR(PCINT1_vect) { }  // Used for waking from sleep


void processButtons(uint8_t inputButtons)
{
	if(!(inputButtons & _BV(MENU_PIN)))
	{
		button = MENU_BUTTON;
	}
	else if(!(inputButtons & _BV(SELECT_PIN))) // && (inputButtons & UP_PIN) && (inputButtons & DOWN_PIN))
	{
		button = SELECT_BUTTON;
	}
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & UP_PIN))*/
/*	{*/
/*		button = UP_SELECT_BUTTON;*/
/*	}*/
/*	else if(!(inputButtons & SELECT_PIN) && !(inputButtons & DOWN_PIN))*/
/*	{*/
/*		button = DOWN_SELECT_BUTTON;*/
/*	}*/
	else if(!(inputButtons & _BV(UP_PIN)))
	{
		button = UP_BUTTON;
	}
	else if(!(inputButtons & _BV(DOWN_PIN)))
	{
		button = DOWN_BUTTON;
	}
	else
	{
		button = NO_BUTTON;
	}

	if((previousButton == button) && (NO_BUTTON != button))
	{
		if(buttonCount > LONG_PRESS_10MS_TICKS)  // Use greater than, so a long press can be caught as a single event in the menu handler code
		{
			if(button_autoincrement_10ms_ticks >= BUTTON_AUTOINCREMENT_ACCEL)
				button_autoincrement_10ms_ticks -= BUTTON_AUTOINCREMENT_ACCEL;  // Progressively reduce the time delay
			if(button_autoincrement_10ms_ticks < BUTTON_AUTOINCREMENT_MINIMUM)
				button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_MINIMUM; // Clamp to some minimum value, otherwise it goes too fast to control
			buttonCount = 0;
		}
		else
		{
			buttonCount++;
		}
	}
	else
	{
		// Reset the counters
		button_autoincrement_10ms_ticks = BUTTON_AUTOINCREMENT_10MS_TICKS;
		ticks_autoincrement = 0;
	}
}

void processSwitches(uint8_t inputButtons)
{
	// Called every 10ms
	if(inputButtons & _BV(AUX_PIN))
		controls &= ~(AUX_CONTROL);
	else
		controls |= AUX_CONTROL;

	if(inputButtons & _BV(BELL_PIN))
		controls &= ~(BELL_CONTROL);
	else
		controls |= BELL_CONTROL;
}


void initialize100HzTimer(void)
{
	// Set up timer 0 for 100Hz interrupts
	TCNT0 = 0;
	OCR0A = 0x6C;
	ticks = 0;
	decisecs = 0;
	TCCR0A = _BV(WGM01);
	TCCR0B = _BV(CS02) | _BV(CS00);
	enableTimer();
}

ISR(TIMER0_COMPA_vect)
{
	status |= STATUS_READ_SWITCHES;

	if (++ticks >= 10)  // 100ms
	{
		ticks = 0;
		decisecs++;
		
		if (pktTimeout)
			pktTimeout--;
		
		if(sleepTimeout_decisecs)
		{
#ifdef FAST_SLEEP
			sleepTimeout_decisecs-=10;
#else
			sleepTimeout_decisecs--;
#endif
		}

		if(alerterTimeout_decisecs)
		{
#ifdef FAST_SLEEP
			alerterTimeout_decisecs-=10;
#else
			alerterTimeout_decisecs--;
#endif
		}

		if(backlightTimeout_decisecs)
			backlightTimeout_decisecs--;

		ledUpdate();

		if (engineTimer)
			engineTimer--;
		
		if (engineScreenTimer)
			engineScreenTimer++;
		
		brakeCounter++;
		// brakePulseWidth sets the minimum pulse width, which occurs when pulsing on 25% of the time. Therefore the period of the counter should be 4 x brakePulseWidth.
		if(brakeCounter >= (4*brakePulseWidth))
			brakeCounter = 0;
		
		updateTime10Hz();
		speed10HzTick = 1;   // updateSpeed10Hz() runs from the main loop - see speed10HzTick's use there
		brake10HzTick = 1;   // updateBrake10Hz() likewise
	}

	if(txHoldoff)
		txHoldoff--;

	if(ticks_autoincrement < button_autoincrement_10ms_ticks)
			ticks_autoincrement++;

	if(sharedQuerySettleTicks < SHARED_QUERY_SETTLE_10MS_TICKS)
		sharedQuerySettleTicks++;
}

// 0xFF is EEPROM's erased/never-written state - on a chip that was flashed before a given field
// existed (or, for optionBits/configBits, a chip that's never had readConfig() run at all), that's
// what it'll read back as, not the intended default. Detect that and fall back to defaultValue,
// persisting it so the field reads clean from here on (same idiom already used for
// sleep_tmr_reset_value/alerter_tmr_reset_value below, generalized here for every SPEED field,
// both STACK combo arrays, and optionBits/configBits).
static uint8_t readByteOrDefault(uint8_t *eeAddr, uint8_t defaultValue)
{
	uint8_t val = eeprom_read_byte(eeAddr);
	if(0xFF == val)
	{
		val = defaultValue;
		eeprom_write_byte(eeAddr, val);
	}
	return val;
}

void readConfig(void)
{
	uint8_t i;
	uint8_t major = 0, minor = 0, delta = 0;

	parseVersionStr(&major, &minor, &delta);

	if(eeprom_read_byte((uint8_t*)EE_VERSION_MAJOR) != major)
		eeprom_write_byte((uint8_t*)EE_VERSION_MAJOR, major);

	if(eeprom_read_byte((uint8_t*)EE_VERSION_MINOR) != minor)
		eeprom_write_byte((uint8_t*)EE_VERSION_MINOR, minor);

	// EEPROM layout-version stamp + one-shot migrations from an older layout (cst-eeprom.c). Reads the
	// pre-stamp EE_LAYOUT_VERSION byte and hands it in; a no-op once the chip is on the current layout.
	// Covered by `make eepromtest`.
	applyEepromMigrations(eeprom_read_byte((uint8_t*)EE_LAYOUT_VERSION));


	update_decisecs = (uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L) | (((uint16_t)eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H)) << 8);
	if(update_decisecs < UPDATE_DECISECS_MIN)
	{
		update_decisecs = UPDATE_DECISECS_MIN;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
	}
	else if(update_decisecs > UPDATE_DECISECS_MAX)
	{
		update_decisecs = UPDATE_DECISECS_MAX;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
	}
	
	txHoldoff_centisecs = eeprom_read_byte((uint8_t*)EE_TX_HOLDOFF);
	{
		// 0xFF is the erased value, not a real 2.55s hold-off - heal it to the default, then clamp to
		// [MIN, MAX] (MAX is 254, one below the 0xFF sentinel, matching the editor's new ceiling).
		uint8_t healed = txHoldoff_centisecs;
		if(0xFF == healed)
			healed = TX_HOLDOFF_DEFAULT;
		if(healed < TX_HOLDOFF_MIN)
			healed = TX_HOLDOFF_MIN;
		else if(healed > TX_HOLDOFF_MAX)
			healed = TX_HOLDOFF_MAX;
		if(healed != txHoldoff_centisecs)
		{
			txHoldoff_centisecs = healed;
			eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, txHoldoff_centisecs);
		}
	}

	// Battery stuff
	uint8_t decivoltsOkay = eeprom_read_byte((uint8_t*)EE_BATTERY_OKAY);
	uint8_t decivoltsWarn = eeprom_read_byte((uint8_t*)EE_BATTERY_WARN);
	uint8_t decivoltsCritical = eeprom_read_byte((uint8_t*)EE_BATTERY_CRITICAL);
	setBatteryLevels(decivoltsOkay, decivoltsWarn, decivoltsCritical);
	if(getBatteryOkay() != decivoltsOkay)
		eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
	if(getBatteryWarn() != decivoltsWarn)
		eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
	if(getBatteryCritical() != decivoltsCritical)
		eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());

	// Read the number of minutes before sleeping from EEP and store it.
	// If it's not in range, clamp it.
	// Abuse sleep_tmr_reset_value to read the EEPROM value in minutes before converting to decisecs
	sleep_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT);
	if(sleep_tmr_reset_value < SLEEP_TMR_RESET_VALUE_MIN)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_MIN;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(0xFF == sleep_tmr_reset_value)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_DEFAULT;  // Default for unprogrammed EEPROM
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	else if(sleep_tmr_reset_value > SLEEP_TMR_RESET_VALUE_MAX)
	{
		sleep_tmr_reset_value = SLEEP_TMR_RESET_VALUE_MAX;
		eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, sleep_tmr_reset_value);
	}
	sleep_tmr_reset_value *= 600;  // Convert to decisecs

	// Read the number of minutes before sleeping from EEP and store it.
	// If it's not in range, clamp it.
	// Abuse alerter_tmr_reset_value to read the EEPROM value in minutes before converting to decisecs
	alerter_tmr_reset_value = eeprom_read_byte((uint8_t*)EE_ALERTER_TIMEOUT);
	if(alerter_tmr_reset_value < ALERTER_TMR_RESET_VALUE_MIN)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_MIN;
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	else if(0xFF == alerter_tmr_reset_value)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_DEFAULT;  // Default for unprogrammed EEPROM
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	else if(alerter_tmr_reset_value > ALERTER_TMR_RESET_VALUE_MAX)
	{
		alerter_tmr_reset_value = ALERTER_TMR_RESET_VALUE_MAX;
		eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, alerter_tmr_reset_value);
	}
	alerter_tmr_reset_value *= 600/4;  // Convert to decisecs

	// Fast clock
	uint8_t maxDeadReckoningTime = eeprom_read_byte((uint8_t*)EE_DEAD_RECKONING_TIME);
	setMaxDeadReckoningTime(maxDeadReckoningTime);
	if(getMaxDeadReckoningTime() != maxDeadReckoningTime)
		eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, getMaxDeadReckoningTime());

	timeSourceAddress = eeprom_read_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS);

	configBits = readByteOrDefault((uint8_t*)EE_CONFIGBITS, CONFIGBITS_DEFAULT);

	// Initialize MRBus address from EEPROM
	mrbus_dev_addr = eeprom_read_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR);
	// Fix bogus addresses
	if(mrbus_dev_addr < MRBUS_DEV_ADDR_MIN)
	{
		mrbus_dev_addr = MRBUS_DEV_ADDR_MIN;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}
	else if(mrbus_dev_addr > MRBUS_DEV_ADDR_MAX)
	{
		mrbus_dev_addr = MRBUS_DEV_ADDR_MAX;
		eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, mrbus_dev_addr);
	}

	mrbus_base_addr = eeprom_read_byte((uint8_t*)EE_BASE_ADDR);
	// Fix bogus addresses
	if(mrbus_base_addr < MRBUS_BASE_ADDR_MIN)
	{
		mrbus_base_addr = MRBUS_BASE_ADDR_MIN;
		eeprom_write_byte((uint8_t*)EE_BASE_ADDR, mrbus_base_addr);
	}
	else if(mrbus_base_addr > MRBUS_BASE_ADDR_MAX)
	{
		mrbus_base_addr = MRBUS_BASE_ADDR_MAX;
		eeprom_write_byte((uint8_t*)EE_BASE_ADDR, mrbus_base_addr);
	}

	// Locomotive Address
	locoAddress = eeprom_read_word((uint16_t*)EE_LOCO_ADDRESS);
	if(locoAddress & LOCO_ADDRESS_SHORT)
	{
		if((locoAddress & ~(LOCO_ADDRESS_SHORT)) > 127)
		{
			// Invalid Short Address, reset to a sane value
			locoAddress = 127;
			eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, locoAddress);
		}
	}
	else
	{
		if(locoAddress > 9999)
		{
			// Invalid Long Address, reset to a sane value
			locoAddress = 9999;
			eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, locoAddress);
		}
	}

	// Function configs. readFunctionConfiguration() reads the raw bytes with no self-heal, so seed a
	// fresh EMRG FN byte (0x15) and COMPRSR2 byte (0x5B) to FN_OFF first - otherwise CONFIG FUNC shows
	// "UNKNOWN" until edited. (The commit-time EEPROM_LAYOUT_VERSION migration will do this for every
	// slot.)
	if(0xFF == eeprom_read_byte((uint8_t*)EE_EMERGENCY_FUNCTION))
		eeprom_write_byte((uint8_t*)EE_EMERGENCY_FUNCTION, FN_OFF);
	if(0xFF == eeprom_read_byte((uint8_t*)EE_COMPRESSOR2_FUNCTION))
		eeprom_write_byte((uint8_t*)EE_COMPRESSOR2_FUNCTION, FN_OFF);
	readFunctionConfiguration();

	functionForceOn = eeprom_read_dword((uint32_t*)EE_FORCE_FUNC_ON);
	functionForceOff = eeprom_read_dword((uint32_t*)EE_FORCE_FUNC_OFF);

	// Thresholds
	hornThreshold = eeprom_read_byte((uint8_t*)EE_HORN_THRESHOLD);
	hornThreshold2 = eeprom_read_byte((uint8_t*)EE_HORN_THRESHOLD2);
	brakeThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_THRESHOLD);
	brakeLowThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_LOW_THRESHOLD);
	brakeHighThreshold = eeprom_read_byte((uint8_t*)EE_BRAKE_HIGH_THRESHOLD);
	
	// Options
	optionBits = readByteOrDefault((uint8_t*)EE_OPTIONBITS, OPTIONBITS_DEFAULT);

	brakePulseWidth = eeprom_read_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH);
	if(brakePulseWidth < BRAKE_PULSE_WIDTH_MIN)
	{
		brakePulseWidth = BRAKE_PULSE_WIDTH_MIN;
		eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
	}
	else if(brakePulseWidth > BRAKE_PULSE_WIDTH_MAX)
	{
		brakePulseWidth = BRAKE_PULSE_WIDTH_MAX;
		eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
	}

	// Notches
	eeprom_read_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);
	for(i=0; i<8; i++)
	{
		if(notchSpeedStep[i] > 126)
			notchSpeedStep[i] = 126;
		if(notchSpeedStep[i] < 1)
			notchSpeedStep[i] = 1;
	}

	// STACK band->combo mapping, both variants (band 0 is always fixed to "none" in each). Read byte by
	// byte via readByteOrDefault() rather than a raw eeprom_read_block(), so an unprogrammed/never-written
	// byte (0xFF - true for EE_STACK_BAND_COMBOS_3STEP on any device that pre-dates the 3-STEP feature)
	// falls back to the real named default instead of "all 3 combo bits set" (0xFF masked down to the 3
	// valid bits is always all-ones, regardless of which band it belongs to).
	stackBandCombos5Step[0] = 0x00;
	stackBandCombos5Step[1] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS + 0), STACK_5STEP_DEFAULT_1);
	stackBandCombos5Step[2] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS + 1), STACK_5STEP_DEFAULT_2);
	stackBandCombos5Step[3] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS + 2), STACK_5STEP_DEFAULT_3);
	stackBandCombos5Step[4] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS + 3), STACK_5STEP_DEFAULT_4);
	stackBandCombos5Step[5] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS + 4), STACK_5STEP_DEFAULT_5);
	for(i=1; i<STACK_BAND_COUNT_5STEP; i++)
	{
		// Mask off anything but the 3 valid bits, in case of other (non-0xFF) corrupt EEPROM - this
		// value gets OR'd directly into controls, so stray bits here would corrupt unrelated bits.
		stackBandCombos5Step[i] &= STACK_COMBO_MASK;
	}
	stackBandCombos3Step[0] = 0x00;
	stackBandCombos3Step[1] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS_3STEP + 0), STACK_3STEP_DEFAULT_1);
	stackBandCombos3Step[2] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS_3STEP + 1), STACK_3STEP_DEFAULT_2);
	stackBandCombos3Step[3] = readByteOrDefault((uint8_t*)(EE_STACK_BAND_COMBOS_3STEP + 2), STACK_3STEP_DEFAULT_3);
	for(i=1; i<STACK_BAND_COUNT_3STEP; i++)
		stackBandCombos3Step[i] &= STACK_COMBO_MASK;

	// Scale-speed simulation config - raw 0-255 values mirroring the loco's decoder CVs directly.
	// ACCEL / DECEL are genuine 0-255 (a decoder's literal CV3 / CV4 can be 255), so they are read raw:
	// a stored 0xFF is a real 255, not "unset". The layout -> 4 seed above initialised any never-written
	// 0x28 / 0x2E byte to the default so a blank chip does not read 255. BRK1 (and the rest) stay on
	// readByteOrDefault - for a brake CV 254 and 255 are indistinguishable (the brake sum caps at 255).
	speedSet(SPEED_ITEM_ACCEL,            eeprom_read_byte((uint8_t*)EE_MOMENTUM_ACCEL_CV3));
	speedSet(SPEED_ITEM_DECEL,            eeprom_read_byte((uint8_t*)EE_MOMENTUM_DECEL_CV4));
	speedSet(SPEED_ITEM_BRAKE1,           readByteOrDefault((uint8_t*)EE_MOMENTUM_BRAKE1_CV179, MOMENTUM_BRAKE1_CV179_DEFAULT));
	speedSet(SPEED_ITEM_BRAKE2,           readByteOrDefault((uint8_t*)EE_MOMENTUM_BRAKE2_CV180, MOMENTUM_BRAKE2_CV180_DEFAULT));
	speedSet(SPEED_ITEM_BRAKE3,           readByteOrDefault((uint8_t*)EE_MOMENTUM_BRAKE3_CV181, MOMENTUM_BRAKE3_CV181_DEFAULT));
	speedSet(SPEED_ITEM_START_DELAY,      readByteOrDefault((uint8_t*)EE_MOMENTUM_START_DELAY, MOMENTUM_START_DELAY_DEFAULT));
	speedSet(SPEED_ITEM_MAX_MPH,          readByteOrDefault((uint8_t*)EE_SPEED_MAX_MPH, SPEED_MAX_MPH_DEFAULT));
	speedSet(SPEED_ITEM_UNIT,             readByteOrDefault((uint8_t*)EE_SPEED_UNIT_KMH, SPEED_UNIT_KMH_DEFAULT));
	speedSet(SPEED_ITEM_STOP_FN,          readByteOrDefault((uint8_t*)EE_SPEED_STOP_WATCH_FN, SPEED_STOP_WATCH_FN_DEFAULT));
	speedSet(SPEED_ITEM_TYPE,             readByteOrDefault((uint8_t*)EE_SPEED_TYPE, SPEED_TYPE_DEFAULT));
	speedSet(SPEED_ITEM_OPLOAD,           readByteOrDefault((uint8_t*)EE_SPEED_OPLOAD, SPEED_OPLOAD_DEFAULT));
	speedSet(SPEED_ITEM_PRLOAD,           readByteOrDefault((uint8_t*)EE_SPEED_PRLOAD, SPEED_PRLOAD_DEFAULT));
	speedSet(SPEED_ITEM_OPLOAD_FN,        readByteOrDefault((uint8_t*)EE_SPEED_OPLOAD_FN, SPEED_OPLOAD_FN_DEFAULT));
	speedSet(SPEED_ITEM_PRLOAD_FN,        readByteOrDefault((uint8_t*)EE_SPEED_PRLOAD_FN, SPEED_PRLOAD_FN_DEFAULT));
	// HOLDFN read raw so OFF (0xFF) sticks - its readByteOrDefault default is F09, not OFF, so the heal
	// would silently revert a user-set OFF. The layout -> 4 seed initialised any never-written 0x57.
	speedSet(SPEED_ITEM_HOLD_FN,          eeprom_read_byte((uint8_t*)EE_SPEED_HOLD_WATCH_FN));
	speedSet(SPEED_ITEM_DECEL_THRESHOLD,  readByteOrDefault((uint8_t*)EE_SPEED_DECEL_THRESHOLD, SPEED_DECEL_THRESHOLD_DEFAULT));
	speedSet(SPEED_ITEM_DECEL_PCT,        readByteOrDefault((uint8_t*)EE_SPEED_DECEL_PCT, SPEED_DECEL_PCT_DEFAULT));
	speedSet(SPEED_ITEM_ACCEL_PCT,        readByteOrDefault((uint8_t*)EE_SPEED_ACCEL_PCT, SPEED_ACCEL_PCT_DEFAULT));
	speedSet(SPEED_ITEM_ACCEL_TARGET,     readByteOrDefault((uint8_t*)EE_SPEED_ACCEL_TARGET, SPEED_ACCEL_TARGET_DEFAULT));
	// ACCELADJ/DECELADJ read raw: -127 is byte 0xFF (sign-magnitude), which collides with the
	// readByteOrDefault sentinel. The layout -> 4 seed initialised any never-written 0x61/0x62 to 0.
	speedSet(SPEED_ITEM_ACCEL_ADJ,       eeprom_read_byte((uint8_t*)EE_SPEED_ACCEL_ADJ));
	speedSet(SPEED_ITEM_DECEL_ADJ,       eeprom_read_byte((uint8_t*)EE_SPEED_DECEL_ADJ));
	speedApplyTypeInert();  // V4 has no CV23/CV24/CV180/CV181/CV103/CV104 - force those inert whatever is stored

	// AIRBRAKE per-profile model config (src/cst-pressure.c)
	airbrakeSet(AIRBRAKE_CHARGED,     readByteOrDefault((uint8_t*)EE_AIRBRAKE_CHARGED,     AIRBRAKE_CHARGED_DEFAULT));
	airbrakeSet(AIRBRAKE_MR_CUTIN,    readByteOrDefault((uint8_t*)EE_AIRBRAKE_MR_CUTIN,    AIRBRAKE_MR_CUTIN_DEFAULT));
	airbrakeSet(AIRBRAKE_MR_CUTOUT,   readByteOrDefault((uint8_t*)EE_AIRBRAKE_MR_CUTOUT,   AIRBRAKE_MR_CUTOUT_DEFAULT));
	airbrakeSet(AIRBRAKE_CHARGE_RATE, readByteOrDefault((uint8_t*)EE_AIRBRAKE_CHARGE_RATE, AIRBRAKE_CHARGE_RATE_DEFAULT));
	airbrakeSet(AIRBRAKE_LEAK_RATE,   readByteOrDefault((uint8_t*)EE_AIRBRAKE_LEAK_RATE,   AIRBRAKE_LEAK_RATE_DEFAULT));
	airbrakeSet(AIRBRAKE_PUMP_RATE,   readByteOrDefault((uint8_t*)EE_AIRBRAKE_PUMP_RATE,   AIRBRAKE_PUMP_RATE_DEFAULT));
	airbrakeSet(AIRBRAKE_MR_LOAD,     readByteOrDefault((uint8_t*)EE_AIRBRAKE_MR_LOAD,     AIRBRAKE_MR_LOAD_DEFAULT));
	airbrakeSet(AIRBRAKE_DISPLAY,     readByteOrDefault((uint8_t*)EE_AIRBRAKE_DISPLAY,     AIRBRAKE_DISPLAY_DEFAULT));
	airbrakeSet(AIRBRAKE_COMP_MODE,   readByteOrDefault((uint8_t*)EE_AIRBRAKE_COMP_MODE,   AIRBRAKE_COMP_MODE_DEFAULT));
}

void copyConfig(uint8_t srcConfig, uint8_t destConfig)
{
	uint8_t configTemp[CONFIG_SIZE];
	eeprom_read_block((void *)configTemp, (void *)CONFIG_OFFSET(srcConfig), CONFIG_SIZE);
	eeprom_write_block((void *)configTemp, (void *)CONFIG_OFFSET(destConfig), CONFIG_SIZE);
}


#ifdef DEMO_8290
const unsigned char customConfigData[40] PROGMEM =
{
	0x62, 0x20, 0x02, 0x01, 0x1C, 0x09, 0x08, 0x80,
	0x03, 0x80, 0x00, 0x16, 0x06, 0x80, 0x05, 0x07, 
	0x58, 0x57, 0x09, 0x80, 0xFF, 0xFF, 0x02, 0xF4,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x06, 0x0E, 0x1A, 0x25, 0x2E, 0x38, 0x41, 0x46
};
#endif

#ifdef DEMO_3658
const unsigned char customConfigData[128] PROGMEM =
{
	0x4a, 0x0e, 0x02, 0x01, 0x1c, 0x09, 0x08, 0x80, 0x17, 0x80, 0x00, 0x19, 0x18, 0x80, 0x06, 0x1a,
	0x4c, 0x80, 0x09, 0x80, 0xff, 0xff, 0x02, 0xf4, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x07, 0x0e, 0x1a, 0x25, 0x2e, 0x38, 0x41, 0x46, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0x80, 0x80, 0x80, 0x80, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff
};
#endif


void resetConfig(void)
{
	uint8_t i;

	wdt_reset();

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, UPDATE_DECISECS_DEFAULT >> 8);
	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, UPDATE_DECISECS_DEFAULT & 0xFF);
	eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, TX_HOLDOFF_DEFAULT);

	eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, SLEEP_TMR_RESET_VALUE_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, ALERTER_TMR_RESET_VALUE_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, DEAD_RECKONING_TIME_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_CONFIGBITS, CONFIGBITS_DEFAULT);

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, MRBUS_DEV_ADDR_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_BASE_ADDR, MRBUS_BASE_ADDR_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS, TIME_SOURCE_ADDRESS_DEFAULT);

	setBatteryLevels(0xFF, 0xFF, 0xFF);  // Set to unprogrammed value, will be reset to default by setBatteryLevels
	eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
	eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
	eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());

	// Skip the following, since these are specific to each physical device:
	//    EE_HORN_THRESHOLD
	//    EE_HORN_THRESHOLD2
	//    EE_BRAKE_THRESHOLD
	//    EE_BRAKE_LOW_THRESHOLD
	//    EE_BRAKE_HIGH_THRESHOLD

	// Write working config first, then copy to all the others
	wdt_reset();
	eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, 0x0003 | LOCO_ADDRESS_SHORT);
	resetFunctionConfiguration();
	writeFunctionConfiguration();
	eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_ON, 0);
	eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_OFF, 0);
	eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, BRAKE_PULSE_WIDTH_DEFAULT);
	eeprom_write_byte((uint8_t*)EE_OPTIONBITS, OPTIONBITS_DEFAULT);
	notchSpeedStep[0] = 7;
	notchSpeedStep[1] = 23;
	notchSpeedStep[2] = 39;
	notchSpeedStep[3] = 55;
	notchSpeedStep[4] = 71;
	notchSpeedStep[5] = 87;
	notchSpeedStep[6] = 103;
	notchSpeedStep[7] = 119;
	eeprom_write_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);

	// Per-profile SPEED / AIRBRAKE / STACK model defaults (eepromResetProfileModel() in cst-eeprom.c) -
	// the portion of a profile that grows as decoder families and simulation parameters are added.
	// Same values readByteOrDefault() falls back to in readConfig() on an unprogrammed (0xFF) byte.
	// Covered by `make eepromtest`. readConfig() at the end of this function reloads the RAM arrays.
	eepromResetProfileModel(CONFIG_OFFSET(WORKING_CONFIG));

	for (i=1; i<=MAX_CONFIGS; i++)
	{
		wdt_reset();
		lcd_gotoxy(3,1);
		printDec2Dig(MAX_CONFIGS-i+1);
		copyConfig(WORKING_CONFIG, i);
	}

#if defined(DEMO_8290) || defined(DEMO_3658)
	// Write working config first, then copy to all the others
	wdt_reset();

	eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, MRBUS_DEV_ADDR_DEFAULT + ('S' - 'A'));
	eeprom_write_byte((uint8_t*)EE_BASE_ADDR, MRBUS_BASE_ADDR_DEFAULT+28);

	for(i=0; i<sizeof(customConfigData); i++)
		eeprom_write_byte((uint8_t*)(i + CONFIG_OFFSET(WORKING_CONFIG)), pgm_read_byte(&(customConfigData[i])));
	copyConfig(WORKING_CONFIG, MAX_CONFIGS);
#endif

	wdt_reset();

	// Read everything again
	readConfig();
}


void init(void)
{
	// Clear watchdog (in the case of an 'X' packet reset)
	MCUSR = 0;
#ifdef ENABLE_WATCHDOG
	// If you don't want the watchdog to do system reset, remove this chunk of code
	wdt_reset();
	wdt_enable(WATCHDOG_TIMEOUT);
	wdt_reset();
#else
	wdt_reset();
	wdt_disable();
#endif

	pktTimeout = 0;  // Assume no base unit until we hear from one
	lastRSSI = 0xFF;

	clearDeadReckoningTime();
	
	readConfig();
	systemBits = SYSTEMBITS_DEFAULT;

	initPorts();
	initADC();
	enableThrottle();
	initialize100HzTimer();

	engineStatesQueueInitialize();
	initAirBrake();
	resetSpeed();

	DDRB |= _BV(PB3);
}

// Row-1 columns 4-7 for the shared-CNF picker's "working" animation: `dots` (1..4) dots, rest blank.
// Same dot idiom the local LOAD/SAVE screen already uses to look busy.
static void printCnfDots(uint8_t dots)
{
	uint8_t i;
	lcd_gotoxy(4, 1);
	for(i = 0; i < 4; i++)
		lcd_putc(i < dots ? '.' : ' ');
}

// Installed as cst-sync.c's progress callback around the blocking N-slot preview query, so the dots
// keep cycling through it (up to ~1.8s if the receiver is unreachable) instead of freezing.
static uint8_t cnfSpinnerPhase;
static void cnfSpinnerTick(void)
{
	printCnfDots(((cnfSpinnerPhase++ >> 3) & 0x03) + 1);   // advance one dot ~every 160ms (callback ~20ms)
}

// Outcome screen for a shared-CNF push/pull. Called from the LOAD/SAVE CNF confirm path and the
// SAVE-to-SHARED two-stage upgrade path (UPGRADE BASE? / WIPE N01-20?). Success shows the house
// "SAVED!" / "LOADED!" confirm; every other outcome drops the LOAD/SAVE label (the operator knows
// which they started) and uses both rows for a plain-language reason. `slot` is the 1-based network
// slot number (1-20), used only by the SYNC_EMPTY screen.
static void displaySyncResult(SyncResult result, uint8_t isSave, uint8_t slot)
{
	lcd_clrscr();

	if(SYNC_OK == result)
	{
		lcd_gotoxy(1,0);
		lcd_puts(isSave ? "SAVED!" : "LOADED!");
		wait100ms(7);
		return;
	}

	if(SYNC_EMPTY == result)   // LOAD only - the slot has never been saved to
	{
		lcd_gotoxy(0,0);
		lcd_puts("SLOT N");
		printDec2DigWZero(slot);
		lcd_gotoxy(0,1);
		lcd_puts("EMPTY");
		wait100ms(30);
		return;
	}

	const char *line1, *line2;
	switch(result)
	{
		case SYNC_BUSY:             line1 = "BUSY";     line2 = "RETRY";    break;
		case SYNC_CHECKSUM_FAIL:    line1 = "CRC FAIL"; line2 = "RETRY";    break;
		case SYNC_TIMEOUT_BEGIN:    line1 = "TIMEOUT";  line2 = "NO REPLY"; break;
		case SYNC_TIMEOUT_DATA:     line1 = "TIMEOUT";  line2 = "TRANSFER"; break;
		case SYNC_TIMEOUT_COMMIT:   line1 = "TIMEOUT";  line2 = "COMMIT";   break;  // SAVE only
		case SYNC_VERSION_MISMATCH: line1 = "FIRMWARE"; line2 = "MISMATCH"; break;
		case SYNC_BAD_ENTRY:
		default:                    line1 = "ERROR";    line2 = "RETRY";    break;
	}
	lcd_gotoxy(0,0);
	lcd_puts(line1);
	lcd_gotoxy(0,1);
	lcd_puts(line2);
	// A failure is rare and the operator needs time to read it - much longer dwell than the confirm.
	wait100ms(30);

	if(SYNC_TIMEOUT_DATA == result)
	{
		// Which chunk offset never got an acked reply - kept as a bench-debug diagnostic.
		lcd_clrscr();
		lcd_gotoxy(0,0);
		lcd_puts("OFFSET");
		lcd_gotoxy(0,1);
		printDec3Dig(syncGetLastTimeoutOffset());
		wait100ms(30);
	}
}

// LOAD button (UP/DOWN/MENU/SEL BTN = LOAD): a persistent 3-way cycle (OFF -> OPLOAD -> PRLOAD ->
// OFF) advanced one step on each momentary press, asserting whichever DCC function SPEED CFG's
// OPLOADFN/PRLOADFN is currently configured to. One RAM-only state variable per button (file-scope,
// not a main() local, since both main()'s button-handling switch and the separate
// renderBaseScreen() need to read/write it) - resets to LOAD_MODE_OFF on power-up, not persisted to
// EEPROM, like optionButtonState.
static LoadMode loadModeUp = LOAD_MODE_OFF;
static LoadMode loadModeDown = LOAD_MODE_OFF;
static LoadMode loadModeMenu = LOAD_MODE_OFF;
static LoadMode loadModeSel = LOAD_MODE_OFF;

static LoadMode advanceLoadMode(LoadMode m)
{
	switch(m)
	{
		case LOAD_MODE_OFF:    return LOAD_MODE_OPLOAD;
		case LOAD_MODE_OPLOAD: return LOAD_MODE_PRLOAD;
		default:                return LOAD_MODE_OFF;
	}
}

// Live LOAD eligibility: SPEED enabled and the profile's TYPE models the load CVs (V5DCC/V5MULT,
// not V4). Checked at every runtime touch-point (not just CONFIG FUNC's value cycle) because
// CONFIGBITS_MAIN_SCREEN_SPEED is edited live in RAM from PREFS with no save required - it can flip
// on the very next render pass while a button still stores a stale FN_LOAD from before. Without
// re-checking here, setupLoadChar() would overwrite the CGRAM slot setupLCD() just correctly
// reloaded for the real PM glyph in that same pass, corrupting it persistently until some unrelated
// mode transition happened to reload clock chars again. isFunctionLoad(fn) alone is only safe to use
// at CONFIG FUNC's value-cycle gate (cst-functions.c), which is about *offering* LOAD as a choice,
// not about whether a slot is safe to write to right now.
static uint8_t loadEligible(void)
{
	return (configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)) && speedTypeHasLoad();
}
static uint8_t loadActive(Functions fn)
{
	return isFunctionLoad(fn) && loadEligible();
}

// The corner glyph for a configurable button (UP / DOWN / MENU / SEL): the "A" glyph
// (AIRBRAKE_GLYPH_CHAR, loaded by baseScreenLcdMode()'s LCD_MAIN* / LCD_OPS* - both screens' base
// palettes carry it) if the button opens the AIRBRAKE gauge - a screen jump, not a DCC function, so
// the softkey circle would be meaningless.
// The LOAD glyph (LOAD_CHAR) if the button is LOAD and currently eligible - loadActive(), not the
// bare isFunctionLoad(), so a now-ineligible stale LOAD assignment falls back to the ordinary circle
// instead of pointing at a slot that may no longer hold a LOAD bitmap. Otherwise the filled/hollow
// circle - filled while the function is asserting and configured, hollow otherwise (shown even when
// the function is OFF, matching the stock UP/DOWN glyphs).
static char buttonCornerGlyph(Functions fn, uint8_t asserting)
{
	if(isFunctionAirBrake(fn))
		return AIRBRAKE_GLYPH_CHAR;
	if(loadActive(fn))
		return LOAD_CHAR;
	return (asserting && !isFunctionOff(fn)) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR;
}

// MAIN_SCREEN and OPS_MODE_SCREEN each get their own CGRAM palette (LCD_MAIN* / LCD_OPS* - see
// cst-lcd.h/cst-common.h) - opsScreen picks which pair, CONFIGBITS_MAIN_SCREEN_SPEED picks the
// _SPEED variant within it: the SPEED readout needs the narrow-H unit glyph in slot 3, the CLOCK
// readout needs AMPM_CHAR there - and the two readouts are mutually exclusive (DISPLAY pref).
// renderBaseScreen() picks printSpeed()/printTime() off the same bit in the same pass, so the
// loaded CGRAM always matches what is drawn.
static LcdMode baseScreenLcdMode(uint8_t opsScreen)
{
	uint8_t speed = (configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)) ? 1 : 0;
	if(opsScreen)
		return speed ? LCD_OPS_SPEED : LCD_OPS;
	return speed ? LCD_MAIN_SPEED : LCD_MAIN;
}

// The AUX indicator glyph, or a blank. Suppressed when the AUX button drives the very DCC function
// HOLDFN watches for ESU Drive Hold: activating AUX then already replaces the loco address with
// "HOLD" (the holdFunctionActive branch in renderBaseScreen()), so the adjacent glyph is redundant.
// A static config comparison, not a runtime holdFunctionActive check - whenever AUX is both active
// and mapped to the HOLDFN number, Drive Hold is by construction asserted too.
static char auxIndicatorChar(void)
{
	if(!(controls & AUX_CONTROL))
		return ' ';
	uint8_t holdFn = speedGet(SPEED_ITEM_HOLD_FN);   // 0-28 = F##, > 28 = OFF
	if((holdFn <= 28) && (getFunctionMask(AUX_FN) == ((uint32_t)1 << holdFn)))
		return ' ';
	return AUX_CHAR;
}

// Draws the base-screen status area, shared by MAIN_SCREEN and OPS_MODE_SCREEN. The two differ only
// in three places, all gated by CONFIGBITS_OPS_MODE (opsLayout) and, for the column-0 glyphs, by
// which screen is showing (opsScreen):
//   - CONFIGBITS_OPS_MODE clear: stock layout - battery at column 0, AUX/blank at (0,1), nothing at
//     (0,0)/(1,0). Byte-identical to the pre-OPS-MODE firmware.
//   - CONFIGBITS_OPS_MODE set: battery at column 6, AUX/blank relocated to (1,0), and column 0 of
//     both rows carries a status glyph - the MENU/SEL "circle" (hollow/filled, like the UP/DOWN
//     glyphs) on the OPS MODE screen, or the "Fn active" reminder glyph (blank unless the function
//     is still latched) on the plain base screen.
// Button handling stays in each screen's own case (it is what actually differs).
static void renderBaseScreen(uint8_t opsScreen, uint8_t backlight, uint8_t optionButtonState,
                             ReverserPosition activeReverserSetting, ReverserPosition reverserPosition_tmp)
{
	uint8_t opsLayout = (configBits & _BV(CONFIGBITS_OPS_MODE)) ? 1 : 0;

	lcd_gotoxy(2,0);
	if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
	{
		lcd_puts("EMRG");
		enableLCDBacklight();
	}
	else if(throttleStatus & THROTTLE_STATUS_ALERTER)
	{
		uint16_t alerter_tmp;
		lcd_puts("ALRT");
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			alerter_tmp = alerterTimeout_decisecs;
		}
		if((alerter_tmp/1) % 2)
			disableLCDBacklight();
		else
			enableLCDBacklight();
	}
	else if(activeReverserSetting != reverserPosition_tmp)
	{
		lcd_puts("REV!");
		enableLCDBacklight();
	}
	else if(holdFunctionActive)
	{
		lcd_puts("HOLD");
		if(backlight || backlightTimeout_decisecs)
			enableLCDBacklight();
		else
			disableLCDBacklight();
	}
	else
	{
		printLocomotiveAddress(locoAddress);
		if(backlight || backlightTimeout_decisecs)
			enableLCDBacklight();
		else
			disableLCDBacklight();
	}

	lcd_gotoxy(1,1);
	if(configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED))
		printSpeed();
	else
		printTime();

	printBattery(opsLayout ? 6 : 0);

	// Rewrite LOAD_CHAR for whichever button is currently LOAD-active (at most one, firmware-enforced
	// - see loadUsedElsewhere() in cst-functions.c) before drawing any corner. Gating on loadActive()
	// rather than isFunctionLoad() is what actually keeps this safe: whenever eligibility drops (SPEED
	// disabled or TYPE=V4), none of these fire, so the shared slot is left alone for setupLCD()'s
	// ordinary clock-glyph reload to own without being overwritten in the same pass.
	if(loadActive(UP_FN))        setupLoadChar(loadModeUp);
	else if(loadActive(DOWN_FN)) setupLoadChar(loadModeDown);
	else if(loadActive(MENU_FN)) setupLoadChar(loadModeMenu);
	else if(loadActive(SEL_FN))  setupLoadChar(loadModeSel);

	lcd_gotoxy(7,0);
	lcd_putc(buttonCornerGlyph(UP_FN, optionButtonState & UP_OPTION_BUTTON));
	lcd_gotoxy(7,1);
	lcd_putc(buttonCornerGlyph(DOWN_FN, optionButtonState & DOWN_OPTION_BUTTON));

	if(opsLayout)
	{
		// LOAD-configured MENU_FN/SEL_FN never sets optionButtonState's bit (its press-edge handler
		// advances loadModeMenu/loadModeSel instead - see the press-edge sites), so the ordinary term
		// below is always false for it regardless of whether OPLOAD/PRLOAD is actively asserting. Treat
		// "currently OPLOAD or PRLOAD" as the LOAD analogue of "latched and on" - loadActive() already
		// falls back to false once the button becomes ineligible (SPEED disabled or TYPE=V4), so a
		// stale/inert LOAD assignment correctly shows nothing here either.
		uint8_t menuOn = ((optionButtonState & MENU_OPTION_BUTTON) && !isFunctionOff(MENU_FN)) ||
		                 (loadActive(MENU_FN) && (LOAD_MODE_OFF != loadModeMenu));
		uint8_t selOn  = ((optionButtonState & SEL_OPTION_BUTTON)  && !isFunctionOff(SEL_FN)) ||
		                 (loadActive(SEL_FN)  && (LOAD_MODE_OFF != loadModeSel));
		lcd_gotoxy(0,0);
		if(opsScreen)
			lcd_putc(buttonCornerGlyph(MENU_FN, optionButtonState & MENU_OPTION_BUTTON));
		else
			lcd_putc(menuOn ? OPS_FN_ACTIVE_CHAR : ' ');
		lcd_gotoxy(0,1);
		if(opsScreen)
			lcd_putc(buttonCornerGlyph(SEL_FN, optionButtonState & SEL_OPTION_BUTTON));
		else
			lcd_putc(selOn ? OPS_FN_ACTIVE_CHAR : ' ');
		lcd_gotoxy(1,0);
		lcd_putc(auxIndicatorChar());
	}
	else
	{
		lcd_gotoxy(0,1);
		lcd_putc(auxIndicatorChar());
	}
}

int main(void)
{
	uint16_t decisecs_tmp;
	uint8_t i;

	uint8_t inputButtons = 0;
	uint8_t txBuffer[MRBUS_BUFFER_SIZE];

	uint8_t activeThrottleSetting = throttlePosition;
	uint8_t lastActiveThrottleSetting = activeThrottleSetting;

	ReverserPosition activeReverserSetting = reverserPosition;
	ReverserPosition lastActiveReverserSetting = activeReverserSetting;
	
	uint8_t lastThrottleStatus = throttleStatus;
	
	uint8_t brakePcnt = 0;

	uint8_t optionButtonState = 0;

	uint8_t backlight = 0;
	uint8_t selectShortPressArmed = 0;  // Set on a SELECT press edge on the main screen; a release while
	                                    // still set = short press -> toggle backlight. Cleared when the
	                                    // power-down long-press fires, so power-down never toggles.

	Screens screenState = LAST_SCREEN;  // Initialize to the last one, since that's the only state guaranteed to be present
	uint8_t subscreenState = 0;
	uint8_t subscreenCount = 0;
	uint8_t systemBitsSnapshot = SYSTEMBITS_DEFAULT;  // Snapshot for reverting SYSTEM_SCREEN's systemBits on menu-cancel

	// OPS MODE. menuAdvancePending: a one-shot armed on a fresh MENU press that begins on the base
	// screen with OPS MODE enabled - it resolves into OPS MODE (the press became a long-press) or
	// into a normal menu advance on release (a short tap), so the base screen never flashes ENGINE
	// on the way into OPS MODE. opsMenuIgnoreUntilRelease: set on entry so the still-held MENU
	// doesn't immediately trip the exit long-press; cleared on release. airbrakeReturnToOps: AIRBRAKE
	// was opened from OPS MODE (via a MENU/SEL/UP/DOWN button set to AIRBRAKE) - any of those four
	// returns there instead of the main screen / menu. airbrakeReturnToMain: AIRBRAKE was opened from
	// the base screen via a UP/DOWN button set to AIRBRAKE (NOT via the menu cycle, NOT from OPS
	// MODE) - any of the four buttons dismisses it straight back to the main screen.
	uint8_t menuAdvancePending = 0;
	uint8_t opsMenuIgnoreUntilRelease = 0;
	uint8_t airbrakeReturnToOps = 0;
	uint8_t airbrakeReturnToMain = 0;

	BrakeStates brakeState = BRAKE_LOW_BEGIN;

	uint8_t decimalNumberIndex = 0;
	uint8_t decimalNumber[4];

	uint8_t functionNumber = 0;

	EngineState engineState = ENGINE_OFF;

	uint32_t functionMask = 0;
	uint32_t lastFunctionMask = 0;

	ReverserPosition direction = FORWARD;

	// Check if the EEPROM is initialized, check first config for all 0xFF
	for(i=0; i< 0x80; i++)
	{
		if(0xFF != eeprom_read_byte((uint8_t *)CONFIG_OFFSET(1) + i) )
			break;
	}
	if(0x80 == i)
	{
		screenState = DIAG_SCREEN;
		subscreenState = 12;
	}
	
	init();

	// Assign after init() so values are read from EEPROM first
	uint8_t newConfigNumber = 1;
	// newConfigNumber's most recent non-SHARED value - restored to whenever the CNF picker screen is left
	// while resting on SHARED, so SHARED can never become a "sticky" default that fires a network query
	// just from cycling back through the menu (see the screenJustChanged block below).
	uint8_t lastLocalConfigNumber = 1;
	// Cached preview of the currently-selected shared entry's loco address in the LOAD/SAVE CNF picker -
	// queried once per fresh arrival at that entry (see the UP/DOWN cases), not re-queried every render pass.
	uint8_t sharedLocoQueryValid = 0;
	uint16_t sharedLocoAddress = 0;
	SyncResult sharedLocoQueryResult = SYNC_OK;
	uint16_t newLocoAddress = locoAddress;
	uint8_t newDevAddr = mrbus_dev_addr;
	uint8_t newBaseAddr = mrbus_base_addr;
	uint8_t newTimeAddr = timeSourceAddress;
	uint8_t newSleepTimeout = sleep_tmr_reset_value / 600;
	uint8_t newAlerterTimeout = alerter_tmr_reset_value / 150;
	uint8_t newUpdate_seconds = update_decisecs / 10;

	setXbeeActive();

	ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
	{
		sleepTimeout_decisecs = sleep_tmr_reset_value;
		alerterTimeout_decisecs = alerter_tmr_reset_value;
	}
	
	enableLCD();

	wdt_reset();

	// Initialize MRBus core
	mrbusPktQueueInitialize(&mrbeeTxQueue, mrbusTxPktBufferArray, MRBUS_TX_BUFFER_DEPTH);
	mrbusPktQueueInitialize(&mrbeeRxQueue, mrbusRxPktBufferArray, MRBUS_RX_BUFFER_DEPTH);
	mrbeeInit();

	sei();	

	wdt_reset();

	// Queue up initial reset version packet (not sent until end of first main loop)
	createVersionPacket(0xFF, txBuffer);
	mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);

	wdt_reset();

	led = LED_OFF;

	enableButtons();
	enableSwitches();

	initLCD();

	// Initialize the buttons so there are no startup artifacts when we actually use them
	inputButtons = PINB & (0xF6);

	BatteryState lastBatteryState = getBatteryState();

	while(1)
	{
		wdt_reset();

		// Heartbeat (or loop timer)
		PORTB |= _BV(PB3);
		PORTB &= ~_BV(PB3);

		if (status & STATUS_READ_SWITCHES)
		{
			// Read switches every 10ms
			status &= ~STATUS_READ_SWITCHES;
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
		}

		processADC();

		// Check for critical battery level
		if(CRITICAL == getBatteryState())
		{
			if(CRITICAL != lastBatteryState)
				lcd_clrscr();  // Clear if entering critical state
			led = LED_OFF;
			setXbeeSleep();
			disableLCDBacklight();
			disableSwitches();
			disableButtons();
			disableThrottle();
			lcd_gotoxy(2,0);
			lcd_puts("LOW");
			lcd_gotoxy(0,1);
			lcd_puts("BATTERY");
			lastBatteryState = getBatteryState();  // Do this here since we are going to skip the rest of the loop
			continue;
		}
		else if(lastBatteryState == CRITICAL)
		{
			lcd_clrscr();  // Clear if leaving critical state
			setXbeeActive();
			enableSwitches();
			enableButtons();
			enableThrottle();
			// Initialize the buttons so there are no startup artifacts when we actually use them
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
			previousButton = button;  // Prevent extraneous menu advances
			clearDeadReckoningTime();
		}
		lastBatteryState = getBatteryState();

		// Convert horn to on/off control
		if(hornPosition <= (hornThreshold - HORN_HYSTERESIS))
		{
			controls &= ~(HORN_CONTROL);
		}
		else if(hornPosition >= hornThreshold)
		{
			controls |= HORN_CONTROL;
		}

		// Second horn stage - independent threshold+hysteresis, same shape as Horn1 above.
		// hornThreshold2 == 0xFF means "not calibrated". Horn2 calibration is optional, so treat that
		// as Horn2 disabled - without this, a full-lever reading (hornPosition can reach 255) would
		// satisfy "hornPosition >= 0xFF" and spuriously assert HORN2_CONTROL.
		if(0xFF == hornThreshold2)
		{
			controls &= ~(HORN2_CONTROL);
		}
		else if(hornPosition <= (hornThreshold2 - HORN_HYSTERESIS))
		{
			controls &= ~(HORN2_CONTROL);
		}
		else if(hornPosition >= hornThreshold2)
		{
			controls |= HORN2_CONTROL;
		}

		// HORN TYPE = Exclusive: Horn2, if active, suppresses Horn1. Stateless - recomputed fresh every
		// pass from the two independent checks above, so no mode-switch stale-state guard is needed.
		if((optionBits & _BV(OPTIONBITS_HORN_TYPE)) && (controls & HORN2_CONTROL))
		{
			controls &= ~(HORN_CONTROL);
		}

		// Sanity check brake position and calculate percentage. Guard the divisor: an uncalibrated
		// chip has both thresholds at 0xFF (resetConfig() deliberately never writes them), and a
		// mis-calibration can land them within BRAKE_DEAD_ZONE*2 of each other - either way
		// brakeHighThreshold <= brakeLowThreshold and the ratio would divide by zero (or go negative).
		if(brakePosition < brakeLowThreshold)
			brakePcnt = 0;
		else if(brakeHighThreshold > brakeLowThreshold)
			brakePcnt = 100 * (brakePosition - brakeLowThreshold) / (brakeHighThreshold - brakeLowThreshold);
		else
			brakePcnt = (brakePosition >= brakeLowThreshold) ? 100 : 0;   // degenerate/uncalibrated: all-or-nothing

		// Handle emergency on brake control.  Do this outside the main brake state machine so the effect is immediate
		if(optionBits & _BV(OPTIONBITS_ESTOP_ON_BRAKE))
		{
			if(brakePosition < brakeLowThreshold)
				estopStatus &= ~ESTOP_BRAKE;
			if(brakePosition > brakeHighThreshold)
				estopStatus |= ESTOP_BRAKE;
		}
		else
		{
			estopStatus &= ~ESTOP_BRAKE;
		}
		
		// Handle brake
		if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STEP == GET_BRK_TYPE(optionBits)) )
		{
			// This state machine handles the variable (stepped) brake.
			switch(brakeState)
			{
				case BRAKE_LOW_BEGIN:
					controls |= BRAKE_REL_CONTROL;  // Pulse the "brake off" control
					brakeState = BRAKE_LOW_WAIT;
					break;
				case BRAKE_LOW_WAIT:
					controls &= ~(BRAKE_REL_CONTROL);
					if(brakePcnt >= 20)
						brakeState = BRAKE_20PCNT_BEGIN;
					break;

				case BRAKE_20PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_20PCNT_WAIT;
					break;
				case BRAKE_20PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 40)
						brakeState = BRAKE_40PCNT_BEGIN;
					break;

				case BRAKE_40PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_40PCNT_WAIT;
					break;
				case BRAKE_40PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 60)
						brakeState = BRAKE_60PCNT_BEGIN;
					break;

				case BRAKE_60PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_60PCNT_WAIT;
					break;
				case BRAKE_60PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePcnt >= 80)
						brakeState = BRAKE_80PCNT_BEGIN;
					break;

				case BRAKE_80PCNT_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_80PCNT_WAIT;
					break;
				case BRAKE_80PCNT_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePosition > brakeHighThreshold)
						brakeState = BRAKE_FULL_BEGIN;
					break;

				case BRAKE_FULL_BEGIN:
					controls |= BRAKE_CONTROL;  // Pulse the "brake on" control
					brakeState = BRAKE_FULL_WAIT;
					break;
				case BRAKE_FULL_WAIT:
					controls &= ~(BRAKE_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					break;
			}
		}
		else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_PULSE == GET_BRK_TYPE(optionBits)) )
		{
			// This state machine handles the variable (pulse) brake.
			switch(brakeState)
			{
				// These two states get "brake off" set by first making sure "brake on" is clear (TCS decoders don't like these changing at the same time)
				case BRAKE_LOW_BEGIN:
					controls &= ~(BRAKE_CONTROL);
					brakeState = BRAKE_LOW_WAIT;
					break;
				case BRAKE_LOW_WAIT:
					controls |= BRAKE_REL_CONTROL;
					brakeState = BRAKE_20PCNT_BEGIN;
					break;

				// These states represent the pulse "brake off" period
				case BRAKE_20PCNT_BEGIN:
				case BRAKE_20PCNT_WAIT:
				case BRAKE_40PCNT_BEGIN:
				case BRAKE_40PCNT_WAIT:
					if( brakePcnt >= (((brakeCounter / brakePulseWidth)+1)*20) )
						brakeState = BRAKE_FULL_BEGIN;
					break;

				// These states represent the pulse "brake on" period
				case BRAKE_60PCNT_BEGIN:
				case BRAKE_60PCNT_WAIT:
				case BRAKE_80PCNT_BEGIN:
				case BRAKE_80PCNT_WAIT:
					if( brakePcnt < (((brakeCounter / brakePulseWidth)+1)*20) )
						brakeState = BRAKE_LOW_BEGIN;
					break;

				// These two states get "brake on" set by first making sure "brake off" is clear (TCS decoders don't like these changing at the same time)
				case BRAKE_FULL_BEGIN:
					controls &= ~(BRAKE_REL_CONTROL);
					brakeState = BRAKE_FULL_WAIT;
					break;
				case BRAKE_FULL_WAIT:
					controls |= BRAKE_CONTROL;
					brakeState = BRAKE_60PCNT_BEGIN;
					break;
			}
		}
		else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STACK == GET_BRK_TYPE(optionBits)) )
		{
			// STACK combo mode - stateless per loop, see evaluateStackBrake().
			evaluateStackBrake(brakePcnt);
		}
		else
		{
			// This state machine handles the basic on/off brake.  The "brake off" control is set when the handle is fully left.  The
			// "brake on" control is set when the handle is above the defined brake threshold.  Transitions always go through a middle
			// state where both controls are cleared.  This is because TCS decoders don't like these functions changing at the same
			// time in the same packet - one of the transitions is ignored.  The middle state forces the active function off before
			// turning on the other function.
			switch(brakeState)
			{
				case BRAKE_LOW_BEGIN:
				case BRAKE_LOW_WAIT:
					// Set "brake off" when below the low threshold
					controls |= BRAKE_REL_CONTROL;
					// Escape logic:
					//    Go to the middle state if above the brakeLowThreshold
					if(brakePosition >= brakeLowThreshold)
						brakeState = BRAKE_20PCNT_BEGIN;
					break;
				case BRAKE_20PCNT_BEGIN:
				case BRAKE_20PCNT_WAIT:
				case BRAKE_40PCNT_BEGIN:
				case BRAKE_40PCNT_WAIT:
				case BRAKE_60PCNT_BEGIN:
				case BRAKE_60PCNT_WAIT:
				case BRAKE_80PCNT_BEGIN:
				case BRAKE_80PCNT_WAIT:
					// Disable both "brake on" and "brake off" when between thresholds
					controls &= ~(BRAKE_CONTROL);
					controls &= ~(BRAKE_REL_CONTROL);
					if(brakePosition < brakeLowThreshold)
						brakeState = BRAKE_LOW_BEGIN;
					else if(brakePosition >= brakeThreshold)
						brakeState = BRAKE_FULL_BEGIN;
					break;
				case BRAKE_FULL_BEGIN:
				case BRAKE_FULL_WAIT:
					// Set "brake on" when above the brake threshold
					controls |= BRAKE_CONTROL;
					// Escape logic:
					//    Limit (brakeThreshold - BRAKE_HYSTERESIS) to non-negative values.  Compare the brake setting to the higher of
					//    the limited (brakeThreshold - BRAKE_HYSTERESIS) or brakeLowThreshold.  If below, go to the middle state.
					if(brakePosition < max( ((brakeThreshold > BRAKE_HYSTERESIS)?(brakeThreshold - BRAKE_HYSTERESIS):0), brakeLowThreshold ) )
						brakeState = BRAKE_20PCNT_BEGIN;
					break;
			}
		}

		// Make sure a stale combo doesn't stick if BRK TYPE is switched away from STACK mid-combo.
		// BRAKE_CONTROL/BRAKE_REL_CONTROL are left alone - already owned by whichever mode just ran.
		if(!( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STACK == GET_BRK_TYPE(optionBits)) ))
		{
			controls &= ~(BK2_CONTROL | BK3_CONTROL);
			currentStackBand = 0;
		}

		// Swap reverser if configured to do so
		ReverserPosition reverserPosition_tmp = reverserPosition;
		if( (optionBits & _BV(OPTIONBITS_REVERSER_SWAP)) || (functionMask & getFunctionMask(REV_SWAP_FN)) )
		{
			switch(reverserPosition_tmp)
			{
				case FORWARD:
					reverserPosition_tmp = REVERSE;
					break;
				case REVERSE:
					reverserPosition_tmp = FORWARD;
					break;
				case NEUTRAL:
					reverserPosition_tmp = NEUTRAL;
					break;
			}
		}
		
		// Calculate active reverser setting
		if(activeReverserSetting != reverserPosition_tmp)
		{
			if( !(configBits & _BV(CONFIGBITS_REVERSER_LOCK)) || (0 == throttlePosition) )
			// Only allow reverser to change when reverser lock disabled, or when throttle is in idle
			activeReverserSetting = reverserPosition_tmp;
		}
		
		// Calculate active throttle setting
		if(NEUTRAL == activeReverserSetting)
		{
			if( (functionMask & getFunctionMask(THR_UNLOCK_FN)) )
			{
				// If a function is assigned to the throttle unlock (e.g. Drive Hold) and the function is active, allow the throttle to change
				activeThrottleSetting = throttlePosition;
			}
			else
			{
				// Otherwise force the throttle to idle to prevent movement
				activeThrottleSetting = 0;
			}
		}
		else
		{
			activeThrottleSetting = throttlePosition;
		}

		// Commanded speed step for the local speed simulation (cst-speed.c). Deliberately
		// different from the outgoing DCC packet's speed byte, which does NOT zero for a centered
		// reverser - it relies on the decoder's separately-sent NEUTRAL_FN to actually stop the motor.
		// For the local mph display, though, a centered reverser means the loco is physically
		// stationary regardless of notch position.
		if((throttleStatus & THROTTLE_STATUS_EMERGENCY) || (NEUTRAL == activeReverserSetting) || (0 == activeThrottleSetting))
			commandedSpeedStep = 0;
		else
			commandedSpeedStep = notchSpeedStep[activeThrottleSetting - 1];

		if((ENGINE_START == engineState) && !engineTimer)
		{
			// START --> RUNNING
			engineState = ENGINE_RUNNING;
		}
		else if((ENGINE_NOT_IDLE == engineState) && (0 == activeThrottleSetting))
		{
			// NOT_IDLE --> RUNNING
			engineState = ENGINE_RUNNING;
		}
		else if((ENGINE_STOP == engineState) && !engineTimer)
		{
			// STOP --> OFF
			engineState = ENGINE_OFF;
		}
		
		updateTime();

		// Generic "did we just switch screens this pass" detector. Deliberately screenState-only, not
		// subscreenState - backing out of a screen's own subscreen (e.g. the CNF confirm screen's
		// MENU_BUTTON case) doesn't change screenState, so it doesn't retrigger this.
		static uint8_t lastRenderedScreenState = 0xFF;
		uint8_t screenJustChanged = (screenState != lastRenderedScreenState);
		if(screenJustChanged && IS_SHARED_CONFIG(newConfigNumber) &&
		   ((LOAD_CONFIG_SCREEN == lastRenderedScreenState) || (SAVE_CONFIG_SCREEN == lastRenderedScreenState)))
		{
			// Un-stick any shared slot as a resting default - it should only ever be revisited by
			// deliberate navigation, never by simply cycling back through the menu system while it
			// happened to be left selected (each such visit would otherwise re-trigger the blocking
			// network query for no reason).
			newConfigNumber = lastLocalConfigNumber;
		}
		lastRenderedScreenState = screenState;

		switch(screenState)
		{
			case MAIN_SCREEN:
				if(!subscreenState)
				{
					// The base screen uses its own LCD_MAIN / LCD_MAIN_SPEED CGRAM set (narrow battery,
					// the AIRBRAKE "A" glyph, the narrow-H unit glyph or AM/PM); the OPS MODE pref only
					// shifts the layout (renderBaseScreen). setupLCD()'s currentMode guard makes the
					// repeat call free.
					setupLCD(baseScreenLcdMode(0));
					renderBaseScreen(0, backlight, optionButtonState, activeReverserSetting, reverserPosition_tmp);
					switch(button)
					{
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								if(loadActive(UP_FN))
									loadModeUp = advanceLoadMode(loadModeUp);
								else if(isFunctionLatching(UP_FN))
									optionButtonState ^= UP_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= UP_OPTION_BUTTON;  // Momentary on
						}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								if(loadActive(DOWN_FN))
									loadModeDown = advanceLoadMode(loadModeDown);
								else if(isFunctionLatching(DOWN_FN))
									optionButtonState ^= DOWN_OPTION_BUTTON;  // Toggle
								else
									optionButtonState |= DOWN_OPTION_BUTTON;  // Momentary on
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								selectShortPressArmed = 1;
								ticks_autoincrement = 0;  // Reset to zero so a long press can be detected
							}
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								// Trigger power down menu on long press
								selectShortPressArmed = 0;  // Long press -> power-down, not a backlight toggle
								subscreenState = 1;
								lcd_clrscr();
							}
							// break;  // Roll through the other cases for cleanup
						case MENU_BUTTON:
						case NO_BUTTON:
							// A SELECT press released before the power-down long-press threshold is a short
							// press: toggle the LCD backlight on release, so starting a power-down never
							// flips it as a side effect. selectShortPressArmed gates out wake-from-sleep
							// SELECT (no press edge seen -> flag stays clear).
							if((NO_BUTTON == button) && (SELECT_BUTTON == previousButton) && selectShortPressArmed)
							{
								selectShortPressArmed = 0;
								if(backlight)
								{
									backlight = 0;
									backlightTimeout_decisecs = 0;  // Explicit off - drop the light now, don't let the menu hold linger
								}
								else
									backlight = 1;
							}
							// Release buttons if momentary
							if(!(isFunctionLatching(UP_FN)))
								optionButtonState &= ~UP_OPTION_BUTTON;
							if(!(isFunctionLatching(DOWN_FN)))
								optionButtonState &= ~DOWN_OPTION_BUTTON;
							break;
					}
					if(optionButtonState & UP_OPTION_BUTTON)
					{
						if(isFunctionAirBrake(UP_FN))
						{
							screenState = AIRBRAKE_SCREEN;
							subscreenState = 0;
							airbrakeReturnToMain = 1;   // dismissed back to the main screen on any button
							optionButtonState &= ~UP_OPTION_BUTTON;   // so a still-held UP does not re-open it on return
							lcd_clrscr();
						}
					}
					if(optionButtonState & DOWN_OPTION_BUTTON)
					{
						if(isFunctionAirBrake(DOWN_FN))
						{
							screenState = AIRBRAKE_SCREEN;
							subscreenState = 0;
							airbrakeReturnToMain = 1;   // dismissed back to the main screen on any button
							optionButtonState &= ~DOWN_OPTION_BUTTON;   // so a still-held DOWN does not re-open it on return
							lcd_clrscr();
						}
					}
				}
				else
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("POWER");
					lcd_gotoxy(0,1);
					lcd_puts("DOWN  -");
					lcd_putc(0x7E);
					switch(button)
					{
						case NO_BUTTON:
							if(DOWN_BUTTON == previousButton)  // Power off
							{
								// Force sleep.  Do this on the trailing edge so the release doesn't wake the throttle from sleep
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
								{
									throttleStatus |= THROTTLE_STATUS_SLEEP;
								}
								// Reset menu so things are clean when returning from sleep
								subscreenState = 0;
								screenState = LAST_SCREEN;
							}
							break;
						case MENU_BUTTON:
							// Escape power down menu and return to main screen
							screenState--;  // Back up one screen.  It will increment in the global MENU button handling code since we just pressed MENU.
							                // Yes, this may underflow, but there's a corresponding ++ in the MENU code.
							subscreenState = 0;
							backlight = 0;
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
							break;
					}
				}
				break;

			case OPS_MODE_SCREEN:
				// OPS MODE - the base screen with MENU/SELECT freed to drive MENU BTN / SEL BTN
				// (same momentary/latching options as UP BTN / DOWN BTN). Entered by a long-press of
				// MENU from the base screen, left by a long-press of MENU here. UP/DOWN behave exactly
				// as on the main screen. Its own LCD_OPS / LCD_OPS_SPEED CGRAM set (see
				// baseScreenLcdMode()) - one slot lighter than the base screen's, since this screen
				// never draws the "Fn active" reminder glyph.
				setupLCD(baseScreenLcdMode(1));
				renderBaseScreen(1, backlight, optionButtonState, activeReverserSetting, reverserPosition_tmp);

				// opsMenuIgnoreUntilRelease (set on entry, so the still-held MENU cannot immediately
				// trip the exit long-press) is cleared at the END of this case, after the switch - so
				// case NO_BUTTON below can still see its pre-release value to tell "released the entry
				// hold" from "released a genuine short MENU tap".

				switch(button)
				{
					case UP_BUTTON:
						if(UP_BUTTON != previousButton)
						{
							if(loadActive(UP_FN))
								loadModeUp = advanceLoadMode(loadModeUp);
							else if(isFunctionLatching(UP_FN))
								optionButtonState ^= UP_OPTION_BUTTON;
							else
								optionButtonState |= UP_OPTION_BUTTON;
						}
						break;
					case DOWN_BUTTON:
						if(DOWN_BUTTON != previousButton)
						{
							if(loadActive(DOWN_FN))
								loadModeDown = advanceLoadMode(loadModeDown);
							else if(isFunctionLatching(DOWN_FN))
								optionButtonState ^= DOWN_OPTION_BUTTON;
							else
								optionButtonState |= DOWN_OPTION_BUTTON;
						}
						break;
					case SELECT_BUTTON:
						// Purely a function button here - no power-down, no backlight toggle.
						if(SELECT_BUTTON != previousButton)
						{
							if(loadActive(SEL_FN))
								loadModeSel = advanceLoadMode(loadModeSel);
							else if(isFunctionLatching(SEL_FN))
								optionButtonState ^= SEL_OPTION_BUTTON;
							else
								optionButtonState |= SEL_OPTION_BUTTON;
						}
						break;
					case MENU_BUTTON:
						if(!opsMenuIgnoreUntilRelease)
						{
							if(MENU_BUTTON != previousButton)
							{
								ticks_autoincrement = 0;  // so the exit long-press can be timed
								// AIRBRAKE is a screen, not a DCC function - opening it on the press
								// edge would leave OPS MODE before the exit long-press below could ever
								// run, trapping the operator. It is opened on release of a short tap
								// instead (case NO_BUTTON). LOAD touches no screenState, so it is safe
								// to advance right here on the press edge - the long-press-exit check
								// just below still runs normally on a held MENU regardless. Latching /
								// momentary MENU_FN keep their press-edge behaviour.
								if(loadActive(MENU_FN))
								{
									loadModeMenu = advanceLoadMode(loadModeMenu);
								}
								else if(!isFunctionAirBrake(MENU_FN))
								{
									if(isFunctionLatching(MENU_FN))
										optionButtonState ^= MENU_OPTION_BUTTON;
									else
										optionButtonState |= MENU_OPTION_BUTTON;
								}
							}
							if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
							{
								// Long-press MENU -> leave OPS MODE for the base screen. A long-press
								// is "exit", not "toggle": undo this same press's latch toggle so a
								// still-latched MENU function is preserved and a not-latched one is not
								// spuriously turned on; drop every momentary bit on the way out.
								if(isFunctionLatching(MENU_FN))
									optionButtonState ^= MENU_OPTION_BUTTON;
								else
									optionButtonState &= ~MENU_OPTION_BUTTON;
								if(!isFunctionLatching(SEL_FN))  optionButtonState &= ~SEL_OPTION_BUTTON;
								if(!isFunctionLatching(UP_FN))   optionButtonState &= ~UP_OPTION_BUTTON;
								if(!isFunctionLatching(DOWN_FN)) optionButtonState &= ~DOWN_OPTION_BUTTON;
								opsMenuIgnoreUntilRelease = 0;
								menuAdvancePending = 0;
								screenState = LAST_SCREEN;
								lcd_clrscr();
							}
						}
						break;
					case NO_BUTTON:
						// Trailing edge of a MENU BTN press. If MENU BTN is AIRBRAKE and this was a
						// short tap (a long hold would already have exited OPS MODE, changing
						// screenState) that is not the still-held entry press (opsMenuIgnoreUntilRelease,
						// read here before the end-of-case clear), open the AIRBRAKE screen now.
						if((MENU_BUTTON == previousButton) && !opsMenuIgnoreUntilRelease && isFunctionAirBrake(MENU_FN))
						{
							screenState = AIRBRAKE_SCREEN;
							subscreenState = 0;
							airbrakeReturnToOps = 1;
							lcd_clrscr();
						}
						break;
				}

				// Release any momentary bit whose button is not currently held (the buttons are
				// mutually exclusive, so "not held" == "not the current button").
				if((UP_BUTTON != button)     && !isFunctionLatching(UP_FN))   optionButtonState &= ~UP_OPTION_BUTTON;
				if((DOWN_BUTTON != button)   && !isFunctionLatching(DOWN_FN)) optionButtonState &= ~DOWN_OPTION_BUTTON;
				if((MENU_BUTTON != button)   && !isFunctionLatching(MENU_FN)) optionButtonState &= ~MENU_OPTION_BUTTON;
				if((SELECT_BUTTON != button) && !isFunctionLatching(SEL_FN))  optionButtonState &= ~SEL_OPTION_BUTTON;

				// UP / DOWN / SEL set to AIRBRAKE open the screen on the press edge (they have no
				// long-press meaning here, and MAIN-screen UP/DOWN -> AIRBRAKE is press-edge too).
				// MENU is handled on release (case NO_BUTTON above) so its exit long-press still
				// works. Any of the four returns to OPS MODE via airbrakeReturnToOps. The triggering
				// button's (momentary) option bit is cleared here so that, on returning, a still-held
				// button does not immediately re-open AIRBRAKE.
				if(OPS_MODE_SCREEN == screenState)
				{
					uint8_t openAirbrake = 0;
					if((optionButtonState & UP_OPTION_BUTTON)   && isFunctionAirBrake(UP_FN))   { openAirbrake = 1; optionButtonState &= ~UP_OPTION_BUTTON; }
					if((optionButtonState & DOWN_OPTION_BUTTON) && isFunctionAirBrake(DOWN_FN)) { openAirbrake = 1; optionButtonState &= ~DOWN_OPTION_BUTTON; }
					if((optionButtonState & SEL_OPTION_BUTTON)  && isFunctionAirBrake(SEL_FN))  { openAirbrake = 1; optionButtonState &= ~SEL_OPTION_BUTTON; }
					if(openAirbrake)
					{
						screenState = AIRBRAKE_SCREEN;
						subscreenState = 0;
						airbrakeReturnToOps = 1;
						lcd_clrscr();
					}
				}

				// Clear the entry-hold ignore now that the switch is done (case NO_BUTTON above has
				// read its pre-release value): any button other than a still-held MENU means the
				// MENU press that entered OPS MODE has been released.
				if(MENU_BUTTON != button)
					opsMenuIgnoreUntilRelease = 0;
				break;

			case ENGINE_SCREEN:
				enableLCDBacklight();
				lcd_gotoxy(0,0);
				lcd_puts(" ENGINE");
				lcd_gotoxy(0,1);
				printEngineState(engineState);
				switch(button)
				{
					case UP_BUTTON:
						if(isFunctionOff(ENGINE_OFF_FN))
						{
							// Level based start/stop
							engineState = ENGINE_ON;
						}
						else
						{
							// Edge triggered start/stop
							// Only send start pulse if in the off state
							if((ENGINE_OFF == engineState) || (ENGINE_RUNNING == engineState))
							{
								engineState = ENGINE_START;
								engineTimer = ENGINE_TIMER_DECISECS;
							}
						}
						engineScreenTimer = 1;
						break;
					case DOWN_BUTTON:
						if(isFunctionOff(ENGINE_OFF_FN))
						{
							// Level based start/stop
							engineState = ENGINE_OFF;
						}
						else
						{
							// Edge triggered start/stop
							// Always allow stop to be sent so we can resync the state with the locomotive if they ever get out of sync
							// But only if the locomotive is in idle
							if(0 == activeThrottleSetting)
							{
								engineState = ENGINE_STOP;
								engineTimer = ENGINE_TIMER_DECISECS;
							}
							else
							{
								// Go to special state if not in idle
								// Main loop will set engineState back to RUNNING once throttle goes to idle
								engineState = ENGINE_NOT_IDLE;
							}
						}
						engineScreenTimer = 1;
						break;
					case SELECT_BUTTON:
						// Do cleanup below
					case MENU_BUTTON:
						// Do cleanup below
					case NO_BUTTON:
						break;
				}

				if((SELECT_BUTTON == button) || (MENU_BUTTON == button) || (engineScreenTimer > ENGINE_SCREEN_TIMER))
				{
					// Clean up
					if(ENGINE_NOT_IDLE == engineState)
					{
						// Clear Not Idle state if exiting the menu
						engineState = ENGINE_RUNNING;
					}
					engineScreenTimer = 0;
					
					if(MENU_BUTTON != button)
					{
						// Exit to main screen
						screenState = LAST_SCREEN;
					}
				}

				break;




			case AIRBRAKE_SCREEN:
				// AIRBRAKE - the operator-facing air-brake display, a read-only viewport into the
				// always-running model. It blocks nothing: the brake lever drives real decoder
				// braking and the real e-stop from here exactly as from the main screen, and the
				// speed sim keeps running in parallel. No landing page / subscreen: renders straight
				// away, so the top-level MENU handler keeps cycling the menu past it.
				// Two display styles, set by the AIRBRAKE CFG "DISPLAY" item (per-profile):
				//   DUAL (default) - Row 0: "BP:" + 3-digit brake-pipe PSI + the 2-cell "PSI" glyph
				//     (cols 6-7). Row 1: "MR:" + 3-digit main-reservoir PSI + the same glyph.
				//   SINGLE - the original ISE analogue gauge dial (revived from git 3cde842, before
				//     BRAKESIM replaced it, now wired to BP): dial (cols 0-3, both rows) + a 3-digit
				//     BP readout + literal " PSI" text (cols 4-7) - the original's own layout.
				// UP/DOWN do nothing here. The full text/diagnostic readout (lever %, BRK REL /
				// BRK SET / COMPRESSOR / emergency letters) is the AIRBRAKE DIAGS page under DIAGS.
				if(airbrakeReturnToOps || airbrakeReturnToMain)
				{
					// Opened from a running screen (OPS MODE, or the base screen via an AIRBRAKE
					// button) rather than the menu cycle - honour the backlight toggle / hold like the
					// main screen and OPS MODE do, instead of the "menu screen = always lit" default.
					// EMRG (brake lever at max) still forces it on. The line-5883 hold re-arm also
					// skips this case so the light does not stick on after returning.
					if((throttleStatus & THROTTLE_STATUS_EMERGENCY) || backlight || backlightTimeout_decisecs)
						enableLCDBacklight();
					else
						disableLCDBacklight();
				}
				else
				{
					enableLCDBacklight();
				}
				if(AIRBRAKE_DISPLAY_SINGLE == airbrakeGet(AIRBRAKE_DISPLAY))
				{
					setupLCD(LCD_AIRBRAKE_ALT);   // no-op after the first call - currentMode bookkeeping only
					setupGaugeChars(airBrakePipePsi(), airbrakeGet(AIRBRAKE_CHARGED));
					lcd_gotoxy(0,0);
					lcd_putc(GAUGE_CHAR_A0);
					lcd_putc(GAUGE_CHAR_A1);
					lcd_putc(GAUGE_CHAR_A2);
					lcd_putc(GAUGE_CHAR_A3);
					lcd_putc(' ');
					printDec3Dig(airBrakePipePsi());
					lcd_gotoxy(0,1);
					lcd_putc(GAUGE_CHAR_B0);
					lcd_putc(GAUGE_CHAR_B1);
					lcd_putc(GAUGE_CHAR_B2);
					lcd_putc(GAUGE_CHAR_B3);
					lcd_puts(" PSI");
				}
				else
				{
					setupLCD(LCD_DEFAULT);   // CGRAM mode for the DUAL glyph view (PSI_CHAR_L/R et al)
					lcd_gotoxy(0,0);
					lcd_puts("BP:");
					printDec3Dig(airBrakePipePsi());
					lcd_putc(PSI_CHAR_L);
					lcd_putc(PSI_CHAR_R);
					lcd_gotoxy(0,1);
					lcd_puts("MR:");
					printDec3Dig(airMainResPsi());
					lcd_putc(PSI_CHAR_L);
					lcd_putc(PSI_CHAR_R);
				}
				if(airbrakeReturnToOps)
				{
					// AIRBRAKE was opened from OPS MODE (via a MENU/SEL/UP/DOWN button set to
					// AIRBRAKE). Any fresh press of those four returns to OPS MODE - not the main
					// screen, not the menu. The button that opened this screen is still held on the
					// next pass (button == previousButton), so it cannot bounce straight back.
					if((NO_BUTTON != button) && (button != previousButton))
					{
						airbrakeReturnToOps = 0;
						// If MENU was the button used to return, ignore it in OPS MODE until released
						// so a still-held MENU cannot immediately trip the OPS MODE exit long-press.
						opsMenuIgnoreUntilRelease = (MENU_BUTTON == button);
						setupLCD(baseScreenLcdMode(1));
						screenState = OPS_MODE_SCREEN;
						lcd_clrscr();
					}
					break;
				}
				if(airbrakeReturnToMain)
				{
					// AIRBRAKE was opened from the base screen via a UP/DOWN button set to AIRBRAKE
					// (not the menu cycle, not OPS MODE). Any of the four buttons dismisses it
					// straight back to the main screen - matching the OPS MODE dismiss. previousButton
					// is synced so the still-held dismiss button does not re-fire there (same idiom as
					// the wake-from-sleep "Prevent extraneous menu advances" line); the
					// button != previousButton guard keeps the still-held entry button from bouncing
					// straight back.
					if((NO_BUTTON != button) && (button != previousButton))
					{
						airbrakeReturnToMain = 0;
						screenState = LAST_SCREEN;   // case LAST_SCREEN restores CGRAM + drops to MAIN_SCREEN
						previousButton = button;
						lcd_clrscr();
					}
					break;
				}
				switch(button)
				{
					case SELECT_BUTTON:
						if(SELECT_BUTTON != previousButton)
						{
							// Quick exit to the main screen (MENU still cycles the menu normally).
							screenState = LAST_SCREEN;
							lcd_clrscr();
						}
						break;
					case UP_BUTTON:
					case DOWN_BUTTON:
					case MENU_BUTTON:
					case NO_BUTTON:
						break;
				}
				break;


			case LOAD_CONFIG_SCREEN:
			case SAVE_CONFIG_SCREEN:
				if(!subscreenState)
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(LOAD_CONFIG_SCREEN == screenState)
						lcd_puts("LOAD");
					else
						lcd_puts("SAVE");
					lcd_puts(" CNF");
					lcd_gotoxy(0,1);
					if(IS_SHARED_CONFIG(newConfigNumber))
					{
						uint8_t sharedEntry = SHARED_CONFIG_ENTRY(newConfigNumber);
						uint8_t sharedSlotNum = sharedEntry + 1;

						// A fresh arrival at any given shared slot always comes from the UP/DOWN cases below
						// (which invalidate the cache and reset sharedQuerySettleTicks on every move within
						// or into the shared range). The blocking query below only actually fires once
						// navigation has sat still on one slot for SHARED_QUERY_SETTLE_10MS_TICKS with no
						// further UP/DOWN press - otherwise a fast sweep across many N slots would freeze
						// the throttle for a beat (up to ~1.8s worst case) at every single one it crosses.
						if(!sharedLocoQueryValid && sharedQuerySettleTicks < SHARED_QUERY_SETTLE_10MS_TICKS)
						{
							// Settle wait: animate the dot field off the 10ms ISR tick (0->60 over 600ms).
							// >>4 at ~160ms/dot fills once ('.'->'..'->'...'->'....') across the settle with no
							// wrap; cnfSpinnerTick() then keeps cycling at the same 160ms/dot through the
							// blocking query below, so it reads as one continuous animation from "landed on
							// slot" to "result".
							lcd_puts("N");
							printDec2DigWZero(sharedSlotNum);
							lcd_putc(':');
							printCnfDots(((sharedQuerySettleTicks >> 4) & 0x03) + 1);
						}
						else
						{
							if(!sharedLocoQueryValid)
							{
								// The blocking query. cnfSpinnerTick(), installed as cst-sync.c's progress
								// callback, keeps the dot field cycling for its duration (tens of ms
								// typically, ~1.8s if cabbus is unreachable) so it doesn't look hung.
								lcd_puts("N");
								printDec2DigWZero(sharedSlotNum);
								lcd_putc(':');
								printCnfDots(1);
								cnfSpinnerPhase = 0;
								syncSetProgressCallback(cnfSpinnerTick);
								sharedLocoQueryResult = syncQuerySharedLocoAddress(sharedEntry, &sharedLocoAddress);
								syncSetProgressCallback(NULL);
								sharedLocoQueryValid = 1;
								lcd_gotoxy(0,1);
							}

							// All four cases below print exactly 8 characters total (N + 2-digit + ':' +
							// 4-char field) - see the LCD-width note in CLAUDE.md's Architecture section on
							// why that matters (lcd_puts doesn't clear the rest of the line, so a shorter
							// string would leave stale characters from whatever rendered here before).
							lcd_puts("N");
							printDec2DigWZero(sharedSlotNum);
							lcd_putc(':');
							switch(sharedLocoQueryResult)
							{
								case SYNC_OK:
									printLocomotiveAddress(sharedLocoAddress);
									break;
								case SYNC_BUSY:
									lcd_puts("BUSY");
									break;
								case SYNC_EMPTY:
									lcd_puts("NONE");
									break;
								default:
									// SYNC_TIMEOUT_BEGIN/_DATA/SYNC_BAD_ENTRY - shouldn't see
									// SYNC_CHECKSUM_FAIL/SYNC_TIMEOUT_COMMIT from a read-only query.
									lcd_puts("FAIL");
									break;
							}
						}
					}
					else
					{
						printDec2DigWZero(newConfigNumber);
						lcd_puts(": ");
						{
							uint16_t eepromAddressDelta = CONFIG_OFFSET(WORKING_CONFIG) - CONFIG_OFFSET(newConfigNumber);
							uint16_t tmpLocoAddress = eeprom_read_word((uint16_t*)(EE_LOCO_ADDRESS - eepromAddressDelta));  // Read loco address of newConfigNumber
							if(tmpLocoAddress & LOCO_ADDRESS_SHORT)
							{
								if((tmpLocoAddress & ~(LOCO_ADDRESS_SHORT)) > 127)
								{
									// Invalid Short Address, reset to a sane value
									tmpLocoAddress = 127 | LOCO_ADDRESS_SHORT;
								}
							}
							else
							{
								if(tmpLocoAddress > 9999)
								{
									// Invalid Long Address, reset to a sane value
									tmpLocoAddress = 9999;
								}
							}
							printLocomotiveAddress(tmpLocoAddress);
						}
					}
					switch(button)
					{
						// Zone-aware: local slots (1..MAX_CONFIGS) use plain ++/-- with a ceiling at
						// MAX_CONFIGS; shared slots (SHARED_CONFIG_BASE..SHARED_CONFIG_MAX) use plain ++/--
						// with a floor at SHARED_CONFIG_MAX; the two explicit crossing cases (local slot 1
						// DOWN -> N01, N01 UP -> local slot 1) are what actually keep N01 adjacent to local
						// slot 1 despite the shared range's raw values sitting above MAX_CONFIGS, not below 1
						// (uint8_t can't represent "below 1" - see the SHARED_CONFIG_BASE comment above).
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								if(newConfigNumber > SHARED_CONFIG_BASE)
								{
									newConfigNumber--;
									sharedLocoQueryValid = 0;
									sharedQuerySettleTicks = 0;
								}
								else if(SHARED_CONFIG_BASE == newConfigNumber)
								{
									newConfigNumber = 1;
								}
								else if(newConfigNumber < MAX_CONFIGS)
								{
									newConfigNumber++;
								}
							}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								if(1 == newConfigNumber)
								{
									newConfigNumber = SHARED_CONFIG_BASE;
									sharedLocoQueryValid = 0;
									sharedQuerySettleTicks = 0;
								}
								else if(newConfigNumber <= MAX_CONFIGS)
								{
									newConfigNumber--;
								}
								else if(newConfigNumber < SHARED_CONFIG_MAX)
								{
									newConfigNumber++;
									sharedLocoQueryValid = 0;
									sharedQuerySettleTicks = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case NO_BUTTON:
							break;
					}
					if(!IS_SHARED_CONFIG(newConfigNumber))
						lastLocalConfigNumber = newConfigNumber;
				}
				else if(1 == subscreenState)
				{
					enableLCDBacklight();
					if(IS_SHARED_CONFIG(newConfigNumber))
					{
						lcd_gotoxy(0,0);
						lcd_puts(LOAD_CONFIG_SCREEN == screenState ? "LOAD" : "SAVE TO");
						lcd_gotoxy(0,1);
						lcd_puts("N");
						printDec2DigWZero(SHARED_CONFIG_ENTRY(newConfigNumber) + 1);
						lcd_puts("?  -");
						lcd_putc(0x7E);
					}
					else
					{
						lcd_gotoxy(0,0);
						lcd_puts("CONFIRM");
						lcd_gotoxy(0,1);
						lcd_puts(LOAD_CONFIG_SCREEN == screenState ? "LOAD? -" : "SAVE? -");
						lcd_putc(0x7E);
					}
					switch(button)
					{
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								// A push (SAVE) to a shared entry can silently upgrade/wipe the whole
								// table (see mrbw-cabbus's version-guard design) if this throttle's own
								// EEPROM_LAYOUT_VERSION is newer than what's currently pinned there - peek
								// the pinned version first, and require two extra explicit confirmations
								// before that happens. The peek is a HARD gate for that destructive path:
								// if it fails we can't tell whether the push would wipe, so we refuse the
								// SAVE rather than risk a silent wipe.
								if(SAVE_CONFIG_SCREEN == screenState && IS_SHARED_CONFIG(newConfigNumber))
								{
									uint8_t peekedVersion;
									lcd_clrscr();
									lcd_gotoxy(0,0);
									lcd_puts("CHECKING");
									SyncResult peekResult = syncPeekSharedVersion(
										SHARED_CONFIG_ENTRY(newConfigNumber), &peekedVersion);
									if(SYNC_OK != peekResult)
									{
										lcd_clrscr();
										lcd_gotoxy(0,0);
										lcd_puts("CHECK");
										lcd_gotoxy(0,1);
										lcd_puts("RETRY");
										wait100ms(30);
										screenState = LAST_SCREEN;
										subscreenState = 0;
										lcd_clrscr();
										break;
									}
									if(0xFF == peekedVersion || peekedVersion < EEPROM_LAYOUT_VERSION)
									{
										subscreenState = 2;
										lcd_clrscr();
										break;
									}
								}

								lcd_clrscr();
								lcd_gotoxy(0,0);

								if(IS_SHARED_CONFIG(newConfigNumber))
								{
									// Network entry - pull/push over the air to mrbw-cabbus's shared CNF
									// table instead of a local copyConfig(). The engine-state-queue swap
									// that the local LOAD path does up front is done here after the pull
									// instead: the incoming loco address isn't known until syncPullSharedCnf()
									// + readConfig() have run, but by then it's the live locoAddress.
									SyncResult result;
									uint8_t sharedEntry = SHARED_CONFIG_ENTRY(newConfigNumber);
									lcd_puts(LOAD_CONFIG_SCREEN == screenState ? "LOADING" : "SAVING");
									if(LOAD_CONFIG_SCREEN == screenState)
									{
										// Snapshot the loco we're leaving before the pull overwrites WORKING_CONFIG / locoAddress.
										uint16_t prevLocoAddress = locoAddress;
										EngineState prevEngineState = engineState;
										result = syncPullSharedCnf(sharedEntry);
										if(SYNC_OK == result)
										{
											readConfig();  // locoAddress is now the pulled loco; engineState is left alone
											// Same hand-off the local LOAD path does: look up the incoming
											// loco first, then save the outgoing one (so the save can't
											// evict the lookup - see the local path's comment).
											engineState = engineStatesQueueGetState(locoAddress);
											if(ENGINE_NOT_INITIALIZED == engineState)
												engineState = prevEngineState;
											engineStatesQueueUpdate(prevLocoAddress, prevEngineState);
										}
									}
									else
									{
										result = syncPushSharedCnf(sharedEntry);
									}

									displaySyncResult(result, SAVE_CONFIG_SCREEN == screenState, sharedEntry + 1);
								}
								else
								{
									// Copy selected config into working config
									if(LOAD_CONFIG_SCREEN == screenState)
									{
										lcd_puts("LOADING");
										EngineState tmpEngineState = engineState;
										// Get new engine state before potentially bumping it off the queue when we save the old one
										uint16_t eepromAddressDelta = CONFIG_OFFSET(WORKING_CONFIG) - CONFIG_OFFSET(newConfigNumber);
										uint16_t tmpLocoAddress = eeprom_read_word((uint16_t*)(EE_LOCO_ADDRESS - eepromAddressDelta));  // Read loco address of newConfigNumber
										engineState = engineStatesQueueGetState(tmpLocoAddress);
										if(ENGINE_NOT_INITIALIZED == engineState)
											engineState = tmpEngineState;  // Restore old state if new locomotive not found
										engineStatesQueueUpdate(locoAddress, tmpEngineState);  // Save current engine state
										copyConfig(newConfigNumber, WORKING_CONFIG);
									}
									else
									{
										lcd_puts("SAVING");
										copyConfig(WORKING_CONFIG, newConfigNumber);
									}

									// Refresh.  Needed for load, not for save
									readConfig();

									lcd_gotoxy(0,1);
									for(i=0; i<8; i++)
									{
										// Do something to make it look active
										wait100ms(1);
										lcd_putc('.');
									}
									wait100ms(3);
								}
								screenState = LAST_SCREEN;
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							screenState--;  // Back up one screen.  It will increment in the global MENU button handling code since we just pressed MENU.
							subscreenState = 0;  // Escape submenu
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else if(2 == subscreenState)
				{
					// First of two explicit confirmations before a SAVE-to-SHARED push that would
					// upgrade/wipe the whole shared table (see the DOWN_BUTTON peek above and mrbw-cabbus's
					// version-guard design) - "N" (MENU_BUTTON) backs all the way out, matching this
					// screen's existing CONFIRM stage's own escape convention.
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("UPGRADE");
					lcd_gotoxy(0,1);
					lcd_puts("BASE? -");
					lcd_putc(0x7E);
					switch(button)
					{
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								subscreenState = 3;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							screenState--;  // Back up one screen - incremented by the global MENU handling.
							subscreenState = 0;  // Escape submenu
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else if(3 == subscreenState)
				{
					// Second confirmation - only reachable via stage 2's own DOWN_BUTTON, always a SAVE to a
					// SHARED entry (see the peek gate above; LOAD/local-slot paths never set subscreenState to
					// 2 or 3 in the first place).
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("WIPE");
					lcd_gotoxy(0,1);
					lcd_puts("N01-20?");
					lcd_putc(0x7E);
					switch(button)
					{
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								lcd_clrscr();
								lcd_gotoxy(0,0);
								// Shown immediately, before the blocking push below - without this the
								// screen sits blank for the several seconds a real upgrading push takes
								// (13 DATA chunks + a COMMIT that also wipes every other entry on cabbus).
								lcd_puts("SAVING");
								SyncResult result = syncPushSharedCnf(SHARED_CONFIG_ENTRY(newConfigNumber));
								displaySyncResult(result, 1, SHARED_CONFIG_ENTRY(newConfigNumber) + 1);
								screenState = LAST_SCREEN;
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							screenState--;  // Back up one screen - incremented by the global MENU handling.
							subscreenState = 0;  // Escape submenu
							lcd_clrscr();
							break;
						case SELECT_BUTTON:
						case UP_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				break;

			case LOCO_SCREEN:
				// A little explanation...  We store the information about a short address in the first digit (decimalNumber[0])
				//    If 0-9, then it's a long address.  If >9, and more specifically, ('s'-'0'), the it's a short address
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(5,0);
					lcd_puts("SET");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  LOCO");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								newLocoAddress = locoAddress;
								decimalNumberIndex = 0;
								if(newLocoAddress & LOCO_ADDRESS_SHORT)
								{
									decimalNumber[0] = 's' - '0';
									decimalNumber[1] = ((newLocoAddress & ~(LOCO_ADDRESS_SHORT)) / 100) % 10;
									decimalNumber[2] = ((newLocoAddress & ~(LOCO_ADDRESS_SHORT)) / 10) % 10;
									decimalNumber[3] = (newLocoAddress & ~(LOCO_ADDRESS_SHORT)) % 10;
								}
								else
								{
									decimalNumber[0] = (newLocoAddress / 1000) % 10;
									decimalNumber[1] = (newLocoAddress / 100) % 10;
									decimalNumber[2] = (newLocoAddress / 10) % 10;
									decimalNumber[3] = (newLocoAddress) % 10;
								}
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					for(i=0; i<4; i++)
					{
						lcd_gotoxy(2+i,0);
						lcd_putc('0' + decimalNumber[i]);
						lcd_gotoxy(2+i,1);
						if(i == decimalNumberIndex)
						{
							lcd_putc('^');
						}
						else
						{
							lcd_putc(' ');
						}
					}
					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(0 == decimalNumberIndex)
								{
									// First digit
									if(decimalNumber[0] > 9)
										decimalNumber[0] = 0;       // short to long
									else if(9 == decimalNumber[0])
									{
										// Check if valid short address
										if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
										{
											decimalNumber[0] = 's' - '0';   // Change to short
										}
										else
										{
											decimalNumber[0] = 0;
										}
									}
									else
										decimalNumber[0]++;
								}
								else if(decimalNumber[0] > 9)
								{
									// Short address, so do special checking
									switch(decimalNumberIndex)
									{
										case 1:
											if( (((decimalNumber[1]+1) * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
												decimalNumber[1] = 1;
											else
												decimalNumber[1] = 0;
											break;
										case 2:
											if( (((decimalNumber[1] * 100) + ((decimalNumber[2]+1) * 10) + decimalNumber[3]) < 128) && (decimalNumber[2] < 9) )
												decimalNumber[2]++;
											else
												decimalNumber[2] = 0;
											break;
										case 3:
											if( (((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3] + 1) < 128) && (decimalNumber[3] < 9) )
												decimalNumber[3]++;
											else
												decimalNumber[3] = 0;
											break;
									}
								}
								else
								{
									// Long address
									if(decimalNumber[decimalNumberIndex] < 9)
										decimalNumber[decimalNumberIndex]++;
									else
										decimalNumber[decimalNumberIndex] = 0;
								}
								
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(0 == decimalNumberIndex)
								{
									// First digit
									if(decimalNumber[0] > 9)
										decimalNumber[0] = 9;       // short to long
									else if(0 == decimalNumber[0])
									{
										// Check if valid short address
										if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
										{
											decimalNumber[0] = 's' - '0';   // Change to short
										}
										else
										{
											decimalNumber[0] = 9;
										}
									}
									else
										decimalNumber[0]--;
								}
								else if(decimalNumber[0] > 9)
								{
									// Short address, so do special checking
									switch(decimalNumberIndex)
									{
										case 1:
											if(decimalNumber[1] > 0)
												decimalNumber[1]--;
											else if( ((1 * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) < 128)
												decimalNumber[1] = 1;
											else
												decimalNumber[1] = 0;
											break;
										case 2:
											if(decimalNumber[2] > 0)
												decimalNumber[2]--;
											else
											{
												if( ((decimalNumber[1] * 100) + (9 * 10) + decimalNumber[3]) < 128)
													decimalNumber[2] = 9;
												else if( ((decimalNumber[1] * 100) + (2 * 10) + decimalNumber[3]) < 128)
													decimalNumber[2] = 2;
												else
													decimalNumber[2] = 1;
											}
											break;
										case 3:
											if(decimalNumber[3] > 0)
												decimalNumber[3]--;
											else
											{
												if( ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + 9) < 128)
													decimalNumber[3] = 9;
												else
													decimalNumber[3] = 7;
											}
											break;
									}
								}
								else
								{
									// Long address
									if(decimalNumber[decimalNumberIndex] > 0)
										decimalNumber[decimalNumberIndex]--;
									else
										decimalNumber[decimalNumberIndex] = 9;
								}

								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								if(decimalNumber[0] > 9)
									newLocoAddress = ((decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3]) | LOCO_ADDRESS_SHORT;
								else
									newLocoAddress = (decimalNumber[0] * 1000) + (decimalNumber[1] * 100) + (decimalNumber[2] * 10) + decimalNumber[3];

								EngineState tmpEngineState = engineState;
								// Get new engine state before potentially bumping it off the queue when we save the old one
								engineState = engineStatesQueueGetState(newLocoAddress);
								if(ENGINE_NOT_INITIALIZED == engineState)
									engineState = tmpEngineState;  // Restore old state if new locomotive not found
								engineStatesQueueUpdate(locoAddress, tmpEngineState);  // Save current engine state

								eeprom_write_word((uint16_t*)EE_LOCO_ADDRESS, newLocoAddress);
								readConfig();
								newLocoAddress = locoAddress;
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								decimalNumberIndex++;
								if(decimalNumberIndex > 3)
									decimalNumberIndex = 0;
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case FORCE_FUNC_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("FORCE");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								functionNumber = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					// Note: _BV() macro doesn't work on 32-bit numbers.  Thus why (1 << functionNumber) is used
					lcd_gotoxy(0,0);
					lcd_puts("F");
					printDec2DigWZero(functionNumber);
					lcd_gotoxy(5,0);
					if(functionForceOn & ((uint32_t)1 << functionNumber))
						lcd_puts(" ON");
					else if(functionForceOff & ((uint32_t)1 << functionNumber))
						lcd_puts("OFF");
					else
						lcd_puts("---");
					switch(button)
					{
						case UP_BUTTON:
							if(UP_BUTTON != previousButton) 
							{
								if( (functionForceOn & ((uint32_t)1 << functionNumber)) || (functionForceOff & ((uint32_t)1 << functionNumber)) )
								{
									// Function turned on, change to turned off (or already turned off)
									functionForceOn &= ~((uint32_t)1 << functionNumber);
									functionForceOff |= ((uint32_t)1 << functionNumber);
								}
								else
								{
									// Function disabled, turn on
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn |= ((uint32_t)1 << functionNumber);
								}
							}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								if(functionForceOff & ((uint32_t)1 << functionNumber))
								{
									// Function turned off, change to turned on
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn |= ((uint32_t)1 << functionNumber);
								}
								else
								{
									// Function turned on or disabled, disable
									functionForceOff &= ~((uint32_t)1 << functionNumber);
									functionForceOn &= ~((uint32_t)1 << functionNumber);
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_ON, functionForceOn);
								eeprom_write_dword((uint32_t*)EE_FORCE_FUNC_OFF, functionForceOff);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Advance through function settings
								if(++functionNumber > 28)
									functionNumber = 0;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case CONFIG_FUNC_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(2,0);
					lcd_puts("CONFIG");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-  FUNC");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								// Always start the walk fresh at HORN_FN. Without this, a specific
								// sequence (navigate to a now-conditionally-hidden function, back out
								// via long-press-cancel instead of SELECT-save, change the condition
								// elsewhere, re-enter here) could land directly on a function that
								// should be hidden - advanceCurrentFunction()'s skip guard only runs
								// on MENU, not on entry.
								resetCurrentFunction();
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{

					lcd_gotoxy(0,0);
					printCurrentFunctionName();
					lcd_gotoxy(0,1);
					printCurrentFunctionValue();
					
					switch(button)
					{
						//  |off|latch|0|Func[4:0]|
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								incrementCurrentFunctionValue(loadEligible());
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								decrementCurrentFunctionValue(loadEligible());
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								writeFunctionConfiguration();
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								resetCurrentFunction();
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								advanceCurrentFunction((configBits & _BV(CONFIGBITS_AIRBRAKE)) ? 1 : 0);
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case NOTCH_CONFIG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("NOTCH");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					lcd_puts("NOTCH ");
					uint8_t notch = subscreenState;
					lcd_putc('0' + notch);
					lcd_gotoxy(0,1);
					printDec4Dig(notchSpeedStep[notch-1]);
					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(notchSpeedStep[notch-1] < 126)
									notchSpeedStep[notch-1]++;
								else
									notchSpeedStep[notch-1] = 126;
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(notchSpeedStep[notch-1] > 1)
									notchSpeedStep[notch-1]--;
								else
									notchSpeedStep[notch-1] = 1;
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_block((void *)notchSpeedStep, (void *)EE_NOTCH_SPEEDSTEP, 8);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								if(subscreenState > 8)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case SPEED_CONFIG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("SPEED");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t speedAdvFunc = (systemBits & _BV(SYSTEMBITS_ADV_FUNC)) ? 1 : 0;
					uint8_t speedItem = speedItemAt(subscreenState, speedAdvFunc);
					if(SPEED_ITEM_COUNT == speedItem)
						speedItem = speedItemAt(1, speedAdvFunc);  // never render the past-end sentinel
					uint8_t speedVal = speedGet(speedItem);

					// ACCELADJ/DECELADJ render a "+/-" CGRAM glyph in their label - load it (and restore
					// the default glyphs for every other item). The menu-exit setupLCD(LCD_DEFAULT) calls
					// restore AUX afterward since currentMode changed.
					setupLCD(speedItemIsSignedAdjust(speedItem) ? LCD_SPEED_ADJ : LCD_DEFAULT);

					lcd_gotoxy(0,0);
					switch(speedItem)
					{
						case SPEED_ITEM_ACCEL:           lcd_puts("ACCEL "); break;  // CV3
						case SPEED_ITEM_DECEL:           lcd_puts("DECEL "); break;  // CV4
						case SPEED_ITEM_BRAKE1:          lcd_puts("BRK1  "); break;  // CV179
						case SPEED_ITEM_BRAKE2:          lcd_puts("BRK2  "); break;  // CV180
						case SPEED_ITEM_BRAKE3:          lcd_puts("BRK3  "); break;  // CV181
						case SPEED_ITEM_START_DELAY:     lcd_puts("DELAY "); break;  // CV167
						case SPEED_ITEM_MAX_MPH:         lcd_puts("MAXSPEED"); break;
						case SPEED_ITEM_UNIT:            lcd_puts("UNIT  "); break;
						case SPEED_ITEM_HOLD_FN:         lcd_puts("HOLDFN"); break;
						case SPEED_ITEM_STOP_FN:         lcd_puts("STOPFN"); break;
						case SPEED_ITEM_OPLOAD:          lcd_puts("OPLOAD"); break;  // CV103
						case SPEED_ITEM_OPLOAD_FN:       lcd_puts("OPLOADFN"); break;
						case SPEED_ITEM_PRLOAD:          lcd_puts("PRLOAD"); break;  // CV104
						case SPEED_ITEM_PRLOAD_FN:       lcd_puts("PRLOADFN"); break;
						case SPEED_ITEM_TYPE:            lcd_puts("TYPE"); break;
						case SPEED_ITEM_ACCEL_PCT:       lcd_puts("ACCPCT"); break;
						case SPEED_ITEM_ACCEL_TARGET:    lcd_puts("ACCTGT"); break;
						case SPEED_ITEM_DECEL_PCT:       lcd_puts("DECPCT"); break;
						case SPEED_ITEM_DECEL_THRESHOLD: lcd_puts("DECTHR"); break;
						case SPEED_ITEM_ACCEL_ADJ:       lcd_puts("ACCEL "); lcd_putc(PLUSMINUS_CHAR); break;  // CV23
						case SPEED_ITEM_DECEL_ADJ:       lcd_puts("DECEL "); lcd_putc(PLUSMINUS_CHAR); break;  // CV24
					}
					lcd_gotoxy(0,1);
					if(SPEED_ITEM_UNIT == speedItem)
						lcd_puts((SPEED_UNIT_KMH == speedVal) ? "KMH" : "MPH");
					else if(speedItemIsWatchFn(speedItem))
					{
						if(SPEED_STOP_WATCH_FN_OFF == speedVal)
							lcd_puts("OFF");
						else
						{
							lcd_putc('F');
							printDec2DigWZero(speedVal);
						}
					}
					else if(SPEED_ITEM_TYPE == speedItem)
						lcd_puts(speedTypeName(speedVal));  // 8-char padded, fills the row
					else if(speedItemIsSignedAdjust(speedItem))
					{
						// -127..+127 -> a fixed 4-char field ("   0" / "+063" / "-127") so a shrinking
						// magnitude can't leave a stale digit behind on an in-place edit.
						int8_t adj = speedAdjDecode(speedVal);
						if(0 == adj)
							lcd_puts("   0");
						else
						{
							lcd_putc((adj < 0) ? '-' : '+');
							printDec3DigWZero((uint16_t)((adj < 0) ? -adj : adj));
						}
					}
					else
						printDec3Dig(speedVal);

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(speedItemIsWatchFn(speedItem))
								{
									// OFF/255 sentinel below 0; step up 0..28, then hold at 28.
									if(SPEED_STOP_WATCH_FN_OFF == speedVal)
										speedVal = 0;
									else if(speedVal < 28)
										speedVal++;
									speedSet(speedItem, speedVal);
								}
								else if(SPEED_ITEM_TYPE == speedItem)
								{
									// Cycle 0..SPEED_TYPE_COUNT-1; a TYPE change re-inits the model params.
									if(speedVal < SPEED_TYPE_COUNT - 1)
									{
										speedSet(SPEED_ITEM_TYPE, speedVal + 1);
										speedResetModel(speedVal, speedVal + 1);
									}
								}
								else if(speedItemIsSignedAdjust(speedItem))
								{
									int8_t adj = speedAdjDecode(speedVal);
									if(adj < SPEED_ADJ_MAG_MAX)
										speedSet(speedItem, speedAdjEncode(adj + 1));
								}
								else
								{
									// UNIT is a 0/1 toggle; ACCEL/DECEL are genuine 0-255; every other
									// plain-numeric item self-heals from 0xFF so it caps at 254.
									uint8_t speedMax = (SPEED_ITEM_UNIT == speedItem) ? 1
									                 : speedItemIsFullRange(speedItem) ? 255 : 254;
									if(speedVal < speedMax)
										speedSet(speedItem, speedVal + 1);
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(speedItemIsWatchFn(speedItem))
								{
									// Step down 28..0, then wrap to the OFF/255 sentinel and hold there.
									if(0 == speedVal)
										speedVal = SPEED_STOP_WATCH_FN_OFF;
									else if(SPEED_STOP_WATCH_FN_OFF != speedVal)
										speedVal--;
									speedSet(speedItem, speedVal);
								}
								else if(SPEED_ITEM_TYPE == speedItem)
								{
									if(speedVal > 0)
									{
										speedSet(SPEED_ITEM_TYPE, speedVal - 1);
										speedResetModel(speedVal, speedVal - 1);
									}
								}
								else if(speedItemIsSignedAdjust(speedItem))
								{
									int8_t adj = speedAdjDecode(speedVal);
									if(adj > -SPEED_ADJ_MAG_MAX)
										speedSet(speedItem, speedAdjEncode(adj - 1));
								}
								else
								{
									// UNIT steps down toward 0 (MPH); MAXSPEED floors at 1 (it is a
									// divisor in the standing-start ramp math); the rest clamp at 0.
									uint8_t speedMin = (SPEED_ITEM_MAX_MPH == speedItem) ? 1 : 0;
									if(speedVal > speedMin)
										speedSet(speedItem, speedVal - 1);
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_ACCEL_CV3,     speedGet(SPEED_ITEM_ACCEL));
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_DECEL_CV4,     speedGet(SPEED_ITEM_DECEL));
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_BRAKE1_CV179,  speedGet(SPEED_ITEM_BRAKE1));
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_BRAKE2_CV180,  speedGet(SPEED_ITEM_BRAKE2));
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_BRAKE3_CV181,  speedGet(SPEED_ITEM_BRAKE3));
								eeprom_write_byte((uint8_t*)EE_MOMENTUM_START_DELAY,   speedGet(SPEED_ITEM_START_DELAY));
								eeprom_write_byte((uint8_t*)EE_SPEED_MAX_MPH,          speedGet(SPEED_ITEM_MAX_MPH));
								eeprom_write_byte((uint8_t*)EE_SPEED_UNIT_KMH,         speedGet(SPEED_ITEM_UNIT));
								eeprom_write_byte((uint8_t*)EE_SPEED_HOLD_WATCH_FN,    speedGet(SPEED_ITEM_HOLD_FN));
								eeprom_write_byte((uint8_t*)EE_SPEED_STOP_WATCH_FN,    speedGet(SPEED_ITEM_STOP_FN));
								eeprom_write_byte((uint8_t*)EE_SPEED_OPLOAD,           speedGet(SPEED_ITEM_OPLOAD));
								eeprom_write_byte((uint8_t*)EE_SPEED_OPLOAD_FN,        speedGet(SPEED_ITEM_OPLOAD_FN));
								eeprom_write_byte((uint8_t*)EE_SPEED_PRLOAD,           speedGet(SPEED_ITEM_PRLOAD));
								eeprom_write_byte((uint8_t*)EE_SPEED_PRLOAD_FN,        speedGet(SPEED_ITEM_PRLOAD_FN));
								eeprom_write_byte((uint8_t*)EE_SPEED_TYPE,             speedGet(SPEED_ITEM_TYPE));
								eeprom_write_byte((uint8_t*)EE_SPEED_ACCEL_PCT,        speedGet(SPEED_ITEM_ACCEL_PCT));
								eeprom_write_byte((uint8_t*)EE_SPEED_ACCEL_TARGET,     speedGet(SPEED_ITEM_ACCEL_TARGET));
								eeprom_write_byte((uint8_t*)EE_SPEED_DECEL_PCT,        speedGet(SPEED_ITEM_DECEL_PCT));
								eeprom_write_byte((uint8_t*)EE_SPEED_DECEL_THRESHOLD,  speedGet(SPEED_ITEM_DECEL_THRESHOLD));
								eeprom_write_byte((uint8_t*)EE_SPEED_ACCEL_ADJ,       speedGet(SPEED_ITEM_ACCEL_ADJ));
								eeprom_write_byte((uint8_t*)EE_SPEED_DECEL_ADJ,       speedGet(SPEED_ITEM_DECEL_ADJ));
								// If the saved TYPE no longer models the load CVs (V4), clear any
								// UP/DOWN/MENU/SEL BTN already set to LOAD - otherwise it would keep
								// showing "LOAD" in CONFIG FUNC despite being unreachable there now.
								if(!speedTypeHasLoad())
									clearLoadFunctions();
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu. speedItemAt() returns the SPEED_ITEM_COUNT
								// sentinel once past the last visible item for this TYPE and ADV FUNC
								// state (the four correction tunables are hidden unless ADV FUNC is on).
								subscreenState++;
								if(SPEED_ITEM_COUNT == speedItemAt(subscreenState, speedAdvFunc))
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case AIRBRAKE_CONFIG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(0,0);
					lcd_puts("AIRBRAKE");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t airbrakeItem = subscreenState - 1;
					uint8_t airbrakeVal = airbrakeGet(airbrakeItem);

					lcd_gotoxy(0,0);
					switch(airbrakeItem)
					{
						case AIRBRAKE_CHARGED:     lcd_puts("BPCHARGE"); break;
						case AIRBRAKE_MR_LOAD:     lcd_puts("MR LOAD "); break;
						case AIRBRAKE_MR_CUTIN:    lcd_puts("MR LOW  "); break;
						case AIRBRAKE_MR_CUTOUT:   lcd_puts("MR HIGH "); break;
						case AIRBRAKE_CHARGE_RATE: lcd_puts("RECHARGE"); break;
						case AIRBRAKE_LEAK_RATE:   lcd_puts("LEAKRATE"); break;
						case AIRBRAKE_PUMP_RATE:   lcd_puts("PUMPRATE"); break;
						case AIRBRAKE_DISPLAY:     lcd_puts("DISPLAY "); break;
						case AIRBRAKE_COMP_MODE:   lcd_puts("COMPMODE"); break;
					}
					// Right-justified to col 7 - gotoxy's column is 8 minus this item's content width.
					switch(airbrakeItem)
					{
						case AIRBRAKE_COMP_MODE:
							lcd_gotoxy(1,1);
							lcd_puts(airbrakeVal ? "CONSIST" : "NORMAL ");   // both 7 chars - clean overwrite either way
							break;
						case AIRBRAKE_DISPLAY:
							lcd_gotoxy(1,1);
							lcd_puts(airbrakeVal ? "SINGLE " : "DUAL   ");   // both 7 chars - clean overwrite either way
							break;
						case AIRBRAKE_CHARGED:
						case AIRBRAKE_MR_CUTIN:
						case AIRBRAKE_MR_CUTOUT:
							// Raw PSI values: 3 digits + the 2-cell PSI glyph = 5 chars.
							lcd_gotoxy(3,1);
							printDec3Dig(airbrakeVal);
							lcd_putc(PSI_CHAR_L);
							lcd_putc(PSI_CHAR_R);
							break;
						case AIRBRAKE_MR_LOAD:
							// A percentage, not PSI: 3 digits + '%' = 4 chars.
							lcd_gotoxy(4,1);
							printDec3Dig(airbrakeVal);
							lcd_putc('%');
							break;
						default:   // RECHARGE / LEAKRATE / PUMPRATE - PSI/min: 3 digits + glyph + "/m" = 7 chars.
							lcd_gotoxy(1,1);
							printDec3Dig(airbrakeVal);
							lcd_putc(PSI_CHAR_L);
							lcd_putc(PSI_CHAR_R);
							lcd_putc('/');
							lcd_putc('m');
							break;
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								// 254, not 255: a stored 0xFF is the "unset" sentinel readByteOrDefault() resets to
								// the default, so a value cranked to 255 would silently revert on the next load.
								uint8_t airbrakeMax = ((AIRBRAKE_COMP_MODE == airbrakeItem) || (AIRBRAKE_DISPLAY == airbrakeItem)) ? 1
								                     : (AIRBRAKE_CHARGED == airbrakeItem) ? AIRBRAKE_CHARGED_MAX
								                     : (AIRBRAKE_MR_LOAD == airbrakeItem) ? AIRBRAKE_MR_LOAD_MAX : 254;
								if(airbrakeVal < airbrakeMax)
									airbrakeSet(airbrakeItem, airbrakeVal + 1);
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								uint8_t airbrakeMin = (AIRBRAKE_CHARGED == airbrakeItem) ? AIRBRAKE_CHARGED_MIN : 0;
								if(airbrakeVal > airbrakeMin)
									airbrakeSet(airbrakeItem, airbrakeVal - 1);
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_CHARGED,     airbrakeGet(AIRBRAKE_CHARGED));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_MR_CUTIN,    airbrakeGet(AIRBRAKE_MR_CUTIN));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_MR_CUTOUT,   airbrakeGet(AIRBRAKE_MR_CUTOUT));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_CHARGE_RATE, airbrakeGet(AIRBRAKE_CHARGE_RATE));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_LEAK_RATE,   airbrakeGet(AIRBRAKE_LEAK_RATE));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_PUMP_RATE,   airbrakeGet(AIRBRAKE_PUMP_RATE));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_MR_LOAD,     airbrakeGet(AIRBRAKE_MR_LOAD));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_DISPLAY,     airbrakeGet(AIRBRAKE_DISPLAY));
								eeprom_write_byte((uint8_t*)EE_AIRBRAKE_COMP_MODE,   airbrakeGet(AIRBRAKE_COMP_MODE));
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								subscreenState++;
								if(subscreenState > AIRBRAKE_COUNT)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case OPTION_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(1,0);
					lcd_puts("OPTIONS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t optionBand;
					uint8_t optionItem = optionItemAt(subscreenState, &optionBand);
					if(OPTION_ITEM_NONE == optionItem)
					{
						subscreenState = 1;
						optionItem = OPTION_ITEM_VAR_BRK;
					}
					enableLCDBacklight();

					lcd_gotoxy(0,0);
					switch(optionItem)
					{
						case OPTION_ITEM_VAR_BRK:    lcd_puts("VAR BRK"); break;
						case OPTION_ITEM_BRK_TYPE:   lcd_puts("BRK TYPE"); break;
						case OPTION_ITEM_BRK_RATE:   lcd_puts("BRK RATE"); break;
						case OPTION_ITEM_STEPS:      lcd_puts("STEPS"); break;
						case OPTION_ITEM_STACK_BAND: lcd_puts("STEP"); lcd_putc('0' + optionBand); break;
						case OPTION_ITEM_BRK_ESTP:   lcd_puts("BRK ESTP"); break;
						case OPTION_ITEM_REV_SWAP:   lcd_puts("REV SWAP"); break;
						case OPTION_ITEM_HORNTYPE:   lcd_puts("HORNTYPE"); break;
					}

					switch(optionItem)
					{
						case OPTION_ITEM_VAR_BRK:
						case OPTION_ITEM_BRK_ESTP:
						case OPTION_ITEM_REV_SWAP:
							lcd_gotoxy(5,1);
							lcd_puts((optionBits & _BV(optionBitFor(optionItem))) ? "ON " : "OFF");
							break;
						case OPTION_ITEM_BRK_TYPE:
							lcd_gotoxy(3,1);
							switch(GET_BRK_TYPE(optionBits))
							{
								case BRK_TYPE_STEP:  lcd_puts(" STEP"); break;
								case BRK_TYPE_STACK: lcd_puts("STACK"); break;
								default:             lcd_puts("PULSE"); break;
							}
							break;
						case OPTION_ITEM_BRK_RATE:
							// brakePulseWidth, tenths of a second
							lcd_gotoxy(4,1);
							lcd_putc('0' + brakePulseWidth / 10);
							lcd_putc('.');
							lcd_putc('0' + brakePulseWidth % 10);
							lcd_gotoxy(7,1);
							lcd_puts("s");
							break;
						case OPTION_ITEM_STEPS:
							// Deterministic set on UP/DOWN (not a flip), so autorepeat does not flicker.
							lcd_gotoxy(0,1);
							lcd_puts(stackIs5Step() ? "5-STEP" : "3-STEP");
							break;
						case OPTION_ITEM_STACK_BAND:
						{
							uint8_t combo = stackCombos()[optionBand];
							lcd_gotoxy(0,1);
							lcd_puts("BRAKE");
							lcd_putc((combo & BRAKE_CONTROL) ? '1' : '-');
							lcd_putc((combo & BK2_CONTROL)   ? '2' : '-');
							lcd_putc((combo & BK3_CONTROL)   ? '3' : '-');
							break;
						}
						case OPTION_ITEM_HORNTYPE:
							// Fixed 8-char width fills the line so no stale characters remain. 0x7F/0x7E
							// are the controller's built-in left/right arrow glyphs, as used elsewhere in
							// the menu for navigation cues.
							lcd_gotoxy(0,1);
							lcd_putc('1');
							lcd_putc(' ');
							lcd_putc(0x7F);
							lcd_putc(0x7E);
							lcd_putc(' ');
							lcd_puts((optionBits & _BV(OPTIONBITS_HORN_TYPE)) ? "2  " : "1+2");
							break;
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								switch(optionItem)
								{
									case OPTION_ITEM_VAR_BRK:
									case OPTION_ITEM_BRK_ESTP:
									case OPTION_ITEM_REV_SWAP:
										optionBits |= _BV(optionBitFor(optionItem));
										break;  // plain bit - deliberately does not reset ticks_autoincrement
									case OPTION_ITEM_BRK_TYPE:
									{
										uint8_t brkType = GET_BRK_TYPE(optionBits);
										if(brkType < BRK_TYPE_STACK)
											brkType++;
										SET_BRK_TYPE(optionBits, brkType);
										ticks_autoincrement = 0;
										break;
									}
									case OPTION_ITEM_STEPS:
										if(!stackIs5Step())
										{
											optionBits |= _BV(OPTIONBITS_STACK_5STEP);
											currentStackBand = 0;  // avoid a stale out-of-range band mid-switch
										}
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_STACK_BAND:
										optionCycleBandCombo(optionBand, +1);
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_HORNTYPE:
										optionBits |= _BV(OPTIONBITS_HORN_TYPE);  // Exclusive
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_BRK_RATE:
										if(brakePulseWidth < BRAKE_PULSE_WIDTH_MAX)
											brakePulseWidth++;
										ticks_autoincrement = 0;
										break;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								switch(optionItem)
								{
									case OPTION_ITEM_VAR_BRK:
									case OPTION_ITEM_BRK_ESTP:
									case OPTION_ITEM_REV_SWAP:
										optionBits &= ~_BV(optionBitFor(optionItem));
										break;
									case OPTION_ITEM_BRK_TYPE:
									{
										uint8_t brkType = GET_BRK_TYPE(optionBits);
										if(brkType > BRK_TYPE_PULSE)
											brkType--;
										SET_BRK_TYPE(optionBits, brkType);
										ticks_autoincrement = 0;
										break;
									}
									case OPTION_ITEM_STEPS:
										if(stackIs5Step())
										{
											optionBits &= ~_BV(OPTIONBITS_STACK_5STEP);
											currentStackBand = 0;
										}
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_STACK_BAND:
										optionCycleBandCombo(optionBand, -1);
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_HORNTYPE:
										optionBits &= ~_BV(OPTIONBITS_HORN_TYPE);  // Additive, default
										ticks_autoincrement = 0;
										break;
									case OPTION_ITEM_BRK_RATE:
										if(brakePulseWidth > BRAKE_PULSE_WIDTH_MIN)
											brakePulseWidth--;
										ticks_autoincrement = 0;
										break;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_BRAKE_PULSE_WIDTH, brakePulseWidth);
								eeprom_write_byte((uint8_t*)EE_OPTIONBITS, optionBits);
								if(stackIs5Step())
									eeprom_write_block((void *)&stackBandCombos5Step[1], (void *)EE_STACK_BAND_COMBOS, 5);
								else
									eeprom_write_block((void *)&stackBandCombos3Step[1], (void *)EE_STACK_BAND_COMBOS_3STEP, 3);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;

								// Skip BRK TYPE and the item-3 slot (BRK RATE / STEPS) when they do not
								// apply: both need variable brake on, and there is no BRK RATE in STEP mode.
								// The STACK band editors above item 3 are shown, not skipped.
								while(	((2 == subscreenState) && !(optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE))) ||
										((3 == subscreenState) && !(optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE))) ||
										((3 == subscreenState) &&  (BRK_TYPE_STEP == GET_BRK_TYPE(optionBits)))
									)
								{
									subscreenState++;
								}

								// Wrap once past the last item (HORNTYPE - its position shifts with the
								// STACK band count, so ask the resolver rather than hardcode it).
								if(OPTION_ITEM_NONE == optionItemAt(subscreenState, &optionBand))
									subscreenState = 1;

								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case THRESHOLD_CAL_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(0,0);
					lcd_puts("THRSHOLD");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CAL");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t *positionPtr = &hornPosition;
					uint8_t *thresholdPtr = &hornThreshold;
					enableLCDBacklight();
					lcd_gotoxy(0,0);
					if(1 == subscreenState)
					{
						lcd_puts("HORN");
						positionPtr = &hornPosition;
						thresholdPtr = &hornThreshold;
					}
					else if(2 == subscreenState)
					{
						lcd_puts("HORN2");
						positionPtr = &hornPosition;
						thresholdPtr = &hornThreshold2;
						// Two-stage horn only works if Horn2's threshold sits above Horn1's (a harder
						// pull crosses Horn2 after Horn1). Flag it if not - non-blocking, just a cue.
						// 0xFF = uncalibrated, skip.
						lcd_gotoxy(0,1);
						if((0xFF != hornThreshold2) && (0xFF != hornThreshold) && (hornThreshold2 <= hornThreshold))
							lcd_puts("<H1");
						else
							lcd_puts("   ");
					}
					else if(3 == subscreenState)
					{
						lcd_puts("BRAKE");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeThreshold;
					}
					else if(4 == subscreenState)
					{
						lcd_puts("BRAKE");
						lcd_gotoxy(0,1);
						lcd_puts("LOW");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeLowThreshold;
					}
					else if(5 == subscreenState)
					{
						lcd_puts("BRAKE");
						lcd_gotoxy(0,1);
						lcd_puts("HIGH");
						positionPtr = &brakePosition;
						thresholdPtr = &brakeHighThreshold;
					}
					else
					{
						subscreenState = 1;
					}
					lcd_gotoxy(7,0);
					if(0xFF == *thresholdPtr)
						lcd_putc('-');
					else
						lcd_putc((*positionPtr >= *thresholdPtr) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
					lcd_gotoxy(5,1);
					printDec3DigWZero(*positionPtr);
					switch(button)
					{
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								// Update threshold to current position
								if(thresholdPtr == &brakeLowThreshold)
								{
									*thresholdPtr = *positionPtr + BRAKE_DEAD_ZONE;
								}
								else if(thresholdPtr == &brakeHighThreshold)
								{
									*thresholdPtr = *positionPtr - BRAKE_DEAD_ZONE;
								}
								else
								{
									*thresholdPtr = *positionPtr;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								// Reject a brake calibration where HIGH is not above LOW - it would
								// make the brakePcnt divisor zero/negative (see the brake-percentage
								// calc). A partial calibration (one still 0xFF) is allowed through.
								if((0xFF != brakeLowThreshold) && (0xFF != brakeHighThreshold)
								   && (brakeHighThreshold <= brakeLowThreshold))
								{
									lcd_clrscr();
									lcd_gotoxy(0,0);
									lcd_puts("BRK CAL");
									lcd_gotoxy(1,1);
									lcd_puts("HI<=LO");
									wait100ms(10);
									lcd_clrscr();
									break;
								}
								eeprom_write_byte((uint8_t*)EE_HORN_THRESHOLD, hornThreshold);
								eeprom_write_byte((uint8_t*)EE_HORN_THRESHOLD2, hornThreshold2);
								eeprom_write_byte((uint8_t*)EE_BRAKE_THRESHOLD, brakeThreshold);
								eeprom_write_byte((uint8_t*)EE_BRAKE_LOW_THRESHOLD, brakeLowThreshold);
								eeprom_write_byte((uint8_t*)EE_BRAKE_HIGH_THRESHOLD, brakeHighThreshold);
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								lcd_clrscr();
							}
							break;
						case DOWN_BUTTON:
							// DOWN does nothing
						case NO_BUTTON:
							break;
					}
				}
				break;

			case COMM_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(4,0);
					lcd_puts("COMM");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-   CFG");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t commItem = subscreenState - 1;
					enableLCDBacklight();

					lcd_gotoxy(0,0);
					switch(commItem)
					{
						case COMM_ITEM_THRTL_ID: lcd_puts("THRTL ID"); break;
						case COMM_ITEM_BASE_ADR: lcd_puts("BASE ADR"); break;
						case COMM_ITEM_TIME_ADR: lcd_puts("TIME ADR"); break;
						case COMM_ITEM_TX_INTVL: lcd_puts("TX INTVL"); break;
						case COMM_ITEM_TX_HLDOF: lcd_puts("TX HLDOF"); break;
					}

					switch(commItem)
					{
						case COMM_ITEM_THRTL_ID:
							lcd_gotoxy(4,1);
							lcd_putc('A' + (newDevAddr - MRBUS_DEV_ADDR_MIN));
							break;
						case COMM_ITEM_BASE_ADR:
							lcd_gotoxy(3,1);
							printDec2DigWZero(newBaseAddr - MRBUS_BASE_ADDR_MIN);
							break;
						case COMM_ITEM_TIME_ADR:
							lcd_gotoxy(2,1);
							if(0x00 == newTimeAddr)
								lcd_puts("BASE");
							else if(0xFF == newTimeAddr)
								lcd_puts(" ALL");
							else
							{
								lcd_puts("0x");
								printHex(newTimeAddr);
							}
							break;
						case COMM_ITEM_TX_INTVL:
							lcd_gotoxy(4,1);
							printDec3Dig(newUpdate_seconds);
							lcd_gotoxy(7,1);
							lcd_puts("s");
							break;
						case COMM_ITEM_TX_HLDOF:
							lcd_gotoxy(3,1);
							lcd_putc('0' + txHoldoff_centisecs / 100);
							lcd_putc('.');
							lcd_putc('0' + (txHoldoff_centisecs / 10) % 10);
							lcd_putc('0' + txHoldoff_centisecs % 10);
							lcd_gotoxy(7,1);
							lcd_puts("s");
							break;
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(!commItemIsAdvGated(commItem) || (systemBits & _BV(SYSTEMBITS_ADV_FUNC)))
								{
									switch(commItem)
									{
										case COMM_ITEM_THRTL_ID:
											if(newDevAddr < MRBUS_DEV_ADDR_MAX)
												newDevAddr++;
											break;
										case COMM_ITEM_BASE_ADR:
											if(newBaseAddr < MRBUS_BASE_ADDR_MAX)
												newBaseAddr++;
											break;
										case COMM_ITEM_TIME_ADR:
											if(newTimeAddr < 0xFF)
												newTimeAddr++;
											break;
										case COMM_ITEM_TX_INTVL:
											if(newUpdate_seconds < UPDATE_DECISECS_MAX / 10)
												newUpdate_seconds++;
											break;
										case COMM_ITEM_TX_HLDOF:
											if(txHoldoff_centisecs < TX_HOLDOFF_MAX)
												txHoldoff_centisecs++;
											break;
									}
								}
								ticks_autoincrement = 0;
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(commItemIsAdvGated(commItem))
								{
									if(systemBits & _BV(SYSTEMBITS_ADV_FUNC))
									{
										switch(commItem)
										{
											case COMM_ITEM_TX_INTVL:
												if(newUpdate_seconds > 1)
													newUpdate_seconds--;
												break;
											case COMM_ITEM_TX_HLDOF:
												if(txHoldoff_centisecs > TX_HOLDOFF_MIN)
													txHoldoff_centisecs--;
												break;
										}
									}
								}
								else
								{
									switch(commItem)
									{
										case COMM_ITEM_THRTL_ID:
											if(newDevAddr > MRBUS_DEV_ADDR_MIN)
												newDevAddr--;
											break;
										case COMM_ITEM_BASE_ADR:
											if(newBaseAddr > MRBUS_BASE_ADDR_MIN)
												newBaseAddr--;
											break;
										case COMM_ITEM_TIME_ADR:
											if(newTimeAddr > 0)
												newTimeAddr--;
											break;
									}
								}
								ticks_autoincrement = 0;
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_ADDR, newDevAddr);
								eeprom_write_byte((uint8_t*)EE_BASE_ADDR, newBaseAddr);
								eeprom_write_byte((uint8_t*)EE_TIME_SOURCE_ADDRESS, newTimeAddr);
								eeprom_write_byte((uint8_t*)EE_TX_HOLDOFF, txHoldoff_centisecs);
								update_decisecs = (uint16_t)newUpdate_seconds * 10;
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_H, update_decisecs >> 8);
								eeprom_write_byte((uint8_t*)MRBUS_EE_DEVICE_UPDATE_L, update_decisecs & 0xFF);
								readConfig();
								newDevAddr = mrbus_dev_addr;
								newBaseAddr = mrbus_base_addr;
								newTimeAddr = timeSourceAddress;
								newUpdate_seconds = update_decisecs / 10;
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								if(subscreenState > COMM_ITEM_COUNT)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case PREFS_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("PREFS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t prefsItem = subscreenState - 1;
					enableLCDBacklight();

					lcd_gotoxy(0,0);
					switch(prefsItem)
					{
						case PREFS_ITEM_DISPLAY:      lcd_puts("DISPLAY"); break;
						case PREFS_ITEM_OPS_MODE:     lcd_puts("OPS MODE"); break;
						case PREFS_ITEM_AIRBRAKE:     lcd_puts("AIRBRAKE"); break;
						case PREFS_ITEM_SLEEP:        lcd_puts("SLEEP"); break;
						case PREFS_ITEM_ALERTER:      lcd_puts("ALERTER"); break;
						case PREFS_ITEM_TIMEOUT:      lcd_puts("TIMEOUT"); break;
						case PREFS_ITEM_LED_BLINK:    lcd_puts("LED BLNK"); break;
						case PREFS_ITEM_REV_LOCK:     lcd_puts("REV LOCK"); break;
						case PREFS_ITEM_STRICT_SLEEP: lcd_puts("STRICT"); break;
					}

					switch(prefsItem)
					{
						case PREFS_ITEM_DISPLAY:
							// No row-1 prefix here, so columns 0-7 are all free - room for the full word
							// rather than the other boolean items' 4-char " ON "/" OFF" cap.
							lcd_gotoxy(0,1);
							lcd_puts((configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)) ? "SPEED" : "CLOCK");
							break;
						case PREFS_ITEM_SLEEP:
							lcd_gotoxy(0,1);
							lcd_puts("DLY:");
							lcd_gotoxy(4,1);
							printDec3Dig(newSleepTimeout);
							lcd_puts("M");
							break;
						case PREFS_ITEM_ALERTER:
							lcd_gotoxy(0,1);
							lcd_puts("DLY:");
							lcd_gotoxy(4,1);
							if(newAlerterTimeout)
							{
								printDec3Dig(newAlerterTimeout * 15);
								lcd_puts("s");
							}
							else
							{
								lcd_puts(" OFF");
							}
							break;
						case PREFS_ITEM_TIMEOUT:
							lcd_gotoxy(0,1);
							lcd_puts("CLK:");
							lcd_gotoxy(4,1);
							printDec3Dig(convertMaxDeadReckoningToDecisecs() / 10);
							lcd_puts("s");
							break;
						case PREFS_ITEM_STRICT_SLEEP:
							lcd_gotoxy(0,1);
							lcd_puts("SLP");
							lcd_gotoxy(4,1);
							lcd_puts((configBits & _BV(CONFIGBITS_STRICT_SLEEP)) ? " ON " : " OFF");
							break;
						case PREFS_ITEM_OPS_MODE:
						case PREFS_ITEM_AIRBRAKE:
						case PREFS_ITEM_LED_BLINK:
						case PREFS_ITEM_REV_LOCK:
							lcd_gotoxy(4,1);
							lcd_puts((configBits & _BV(prefsItemBit(prefsItem))) ? " ON " : " OFF");
							break;
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(prefsItemIsBit(prefsItem))
								{
									configBits |= _BV(prefsItemBit(prefsItem));
								}
								else
								{
									switch(prefsItem)
									{
										case PREFS_ITEM_SLEEP:
											if(newSleepTimeout < SLEEP_TMR_RESET_VALUE_MAX)
												newSleepTimeout++;
											break;
										case PREFS_ITEM_ALERTER:
											if(newAlerterTimeout < ALERTER_TMR_RESET_VALUE_MAX)
												newAlerterTimeout++;
											break;
										case PREFS_ITEM_TIMEOUT:
											incrementMaxDeadReckoningTime();
											break;
									}
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(prefsItemIsBit(prefsItem))
								{
									configBits &= ~_BV(prefsItemBit(prefsItem));
								}
								else
								{
									switch(prefsItem)
									{
										case PREFS_ITEM_SLEEP:
											if(newSleepTimeout > SLEEP_TMR_RESET_VALUE_MIN)
												newSleepTimeout--;
											break;
										case PREFS_ITEM_ALERTER:
											if(newAlerterTimeout > ALERTER_TMR_RESET_VALUE_MIN)
												newAlerterTimeout--;
											break;
										case PREFS_ITEM_TIMEOUT:
											decrementMaxDeadReckoningTime();
											break;
									}
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_DEVICE_SLEEP_TIMEOUT, newSleepTimeout);
								eeprom_write_byte((uint8_t*)EE_ALERTER_TIMEOUT, newAlerterTimeout);
								eeprom_write_byte((uint8_t*)EE_DEAD_RECKONING_TIME, getMaxDeadReckoningTime());
								eeprom_write_byte((uint8_t*)EE_CONFIGBITS, configBits);
								// If the saved DISPLAY setting no longer shows SPEED, clear any UP/DOWN/
								// MENU/SEL BTN already set to LOAD - otherwise it would keep showing
								// "LOAD" in CONFIG FUNC despite being unreachable there now (same
								// reasoning as SPEED_CONFIG_SCREEN's TYPE-drops-to-V4 clearLoadFunctions()
								// call).
								if(!(configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)))
									clearLoadFunctions();
								readConfig();
								// Resync the new* staging locals from the (readConfig()-restored) real
								// values. Also done by the long-press-Menu cancel handler in the top-level
								// Menu logic - the two paths that leave this screen. new* values exist
								// because the on-screen format differs from the program's.
								newSleepTimeout = sleep_tmr_reset_value / 600;
								newAlerterTimeout = alerter_tmr_reset_value / 150;
								// Reset alerter here so it doesn't trigger the alerter down below when changing from off
								ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
								{
									alerterTimeout_decisecs = alerter_tmr_reset_value;
								}
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								if(subscreenState > PREFS_ITEM_COUNT)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;

			case SYSTEM_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(2,0);
					lcd_puts("SYSTEM");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_puts("-");
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					uint8_t systemItem = subscreenState - 1;
					enableLCDBacklight();

					// Battery thresholds are edited as a set (setBatteryLevels() takes all three), so
					// read all three fresh each pass and index by systemItem - SYSTEM_ITEM_BAT_OKAY.
					uint8_t decivolts[3] = { getBatteryOkay(), getBatteryWarn(), getBatteryCritical() };

					lcd_gotoxy(0,0);
					switch(systemItem)
					{
						case SYSTEM_ITEM_MENU_LOCK: lcd_puts("MENU LCK"); break;
						case SYSTEM_ITEM_ADV_FUNC:  lcd_puts("ADV FUNC"); break;
						case SYSTEM_ITEM_BAT_OKAY:  lcd_puts("BAT OKAY"); break;
						case SYSTEM_ITEM_BAT_WARN:  lcd_puts("BAT WARN"); break;
						case SYSTEM_ITEM_BAT_CRIT:  lcd_puts("BAT CRIT"); break;
					}

					if(systemItemIsBit(systemItem))
					{
						lcd_gotoxy(4,1);
						lcd_puts((systemBits & _BV(systemItemBit(systemItem))) ? " ON " : " OFF");
					}
					else
					{
						uint8_t dv = decivolts[systemItem - SYSTEM_ITEM_BAT_OKAY];
						lcd_gotoxy(4,1);
						lcd_putc('0' + dv / 10);
						lcd_putc('.');
						lcd_putc('0' + dv % 10);
						lcd_gotoxy(7,1);
						lcd_puts("V");
					}

					switch(button)
					{
						case UP_BUTTON:
							if((UP_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(systemItemIsBit(systemItem))
								{
									systemBits |= _BV(systemItemBit(systemItem));
								}
								else if(systemBits & _BV(SYSTEMBITS_ADV_FUNC))
								{
									uint8_t idx = systemItem - SYSTEM_ITEM_BAT_OKAY;
									if(decivolts[idx] < 0xFF)
										decivolts[idx]++;
									setBatteryLevels(decivolts[0], decivolts[1], decivolts[2]);
									ticks_autoincrement = 0;
								}
							}
							break;
						case DOWN_BUTTON:
							if((DOWN_BUTTON != previousButton) || (ticks_autoincrement >= button_autoincrement_10ms_ticks))
							{
								if(systemItemIsBit(systemItem))
								{
									systemBits &= ~_BV(systemItemBit(systemItem));
								}
								else if(systemBits & _BV(SYSTEMBITS_ADV_FUNC))
								{
									uint8_t idx = systemItem - SYSTEM_ITEM_BAT_OKAY;
									if(decivolts[idx] > 0)
										decivolts[idx]--;
									setBatteryLevels(decivolts[0], decivolts[1], decivolts[2]);
									ticks_autoincrement = 0;
								}
							}
							break;
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								eeprom_write_byte((uint8_t*)EE_BATTERY_OKAY, getBatteryOkay());
								eeprom_write_byte((uint8_t*)EE_BATTERY_WARN, getBatteryWarn());
								eeprom_write_byte((uint8_t*)EE_BATTERY_CRITICAL, getBatteryCritical());
								readConfig();
								lcd_clrscr();
								lcd_gotoxy(1,0);
								lcd_puts("SAVED!");
								wait100ms(7);
								subscreenState = 0;  // Escape submenu
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance menu
								subscreenState++;
								if(subscreenState > SYSTEM_ITEM_COUNT)
									subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case NO_BUTTON:
							break;
					}
				}
				break;
				
			case DIAG_SCREEN:
				enableLCDBacklight();
				if(!subscreenState)
				{
					lcd_gotoxy(3,0);
					lcd_puts("DIAGS");
					lcd_gotoxy(0,1);
					lcd_putc(0x7F);
					lcd_putc('-');
					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								setupLCD(LCD_DIAGS);
								subscreenState = 1;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
						case UP_BUTTON:
						case DOWN_BUTTON:
						case NO_BUTTON:
							break;
					}
				}
				else
				{
					if(1 == subscreenState)
					{
						if(!subscreenCount)
						{
							enableLCDBacklight();
							lcd_gotoxy(5,0);
							if(0 == activeThrottleSetting)
							{
								lcd_putc('I');
							}
							else
							{
								lcd_putc('0' + activeThrottleSetting);
							}
							lcd_gotoxy(0,0);
							if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
							{
								lcd_puts("EMRG");
							}
							else
							{
								if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STEP == GET_BRK_TYPE(optionBits)) )
								{
									// Stepped-brake level shown as "STP1".."STP5", not "BRK*" - keeps it clear of
									// the BRAKE / BRAKE2 / BRAKE3 function names (Configure Function) and the
									// "BRAKE1".."BRAKE123" combo readout (STACK step editor). Same "STP" label
									// STACK mode uses on this status field (below).
									switch(brakeState)
									{
										// These two states get "brake off" set by first making sure "brake on" is clear (TCS decoders don't like these changing at the same time)
										case BRAKE_LOW_BEGIN:
										case BRAKE_LOW_WAIT:
											lcd_puts("OFF ");
											break;
										case BRAKE_20PCNT_BEGIN:
										case BRAKE_20PCNT_WAIT:
											lcd_puts("STP1");
											break;
										case BRAKE_40PCNT_BEGIN:
										case BRAKE_40PCNT_WAIT:
											lcd_puts("STP2");
											break;
										case BRAKE_60PCNT_BEGIN:
										case BRAKE_60PCNT_WAIT:
											lcd_puts("STP3");
											break;
										case BRAKE_80PCNT_BEGIN:
										case BRAKE_80PCNT_WAIT:
											lcd_puts("STP4");
											break;
										case BRAKE_FULL_BEGIN:
										case BRAKE_FULL_WAIT:
											lcd_puts("STP5");
											break;
									}
								}
								else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STACK == GET_BRK_TYPE(optionBits)) )
								{
									if(0 == currentStackBand)
									{
										lcd_puts("OFF ");
									}
									else
									{
										lcd_puts("STP");
										lcd_putc('0' + currentStackBand);
									}
								}
								else
								{
									if( !(controls & BRAKE_CONTROL) && !(controls & BRAKE_REL_CONTROL) )
										lcd_putc(FUNCTION_INACTIVE_CHAR);
									else if( (controls & BRAKE_CONTROL) && !(controls & BRAKE_REL_CONTROL) )
										lcd_putc(FUNCTION_ACTIVE_CHAR);
									else if( !(controls & BRAKE_CONTROL) && (controls & BRAKE_REL_CONTROL) )
										lcd_putc('*');
									else if( (controls & BRAKE_CONTROL) && (controls & BRAKE_REL_CONTROL) )
										lcd_putc('!');  // Invalid condition
									printDec2Dig((brakePcnt>99)?99:brakePcnt);
									lcd_putc('%');
								}
							}
						
							lcd_gotoxy(7,0);
							switch(activeReverserSetting)
							{
								case FORWARD:
									lcd_putc('F');
									break;
								case NEUTRAL:
									lcd_putc('N');
									break;
								case REVERSE:
									lcd_putc('R');
									break;
							}

							lcd_gotoxy(2, 1);
							lcd_putc((controls & AUX_CONTROL) ? FUNCTION_ACTIVE_CHAR : FUNCTION_INACTIVE_CHAR);
							lcd_gotoxy(4, 1);
							lcd_putc((controls & BELL_CONTROL) ? BELL_CHAR : ' ');
							lcd_gotoxy(5, 1);
							lcd_putc((controls & HORN_CONTROL) ? HORN_CHAR : ' ');
							lcd_gotoxy(6, 1);
							lcd_putc((controls & HORN2_CONTROL) ? HORN_CHAR : ' ');

							lcd_gotoxy(7, 1);
							switch(frontLight)
							{
								case LIGHT_OFF:
									lcd_putc('-');
									break;
								case LIGHT_DIM:
									lcd_putc('D');
									break;
								case LIGHT_BRIGHT:
									lcd_putc('B');
									break;
								case LIGHT_BRIGHT_DITCH:
									lcd_putc('*');
									break;
							}

							lcd_gotoxy(0, 1);
							switch(rearLight)
							{
								case LIGHT_OFF:
									lcd_putc('-');
									break;
								case LIGHT_DIM:
									lcd_putc('D');
									break;
								case LIGHT_BRIGHT:
									lcd_putc('B');
									break;
								case LIGHT_BRIGHT_DITCH:
									lcd_putc('*');
									break;
							}
						}
						else
						{
							if(subscreenCount > 3)
								subscreenCount = 3;
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("FN:");
							lcd_gotoxy(0,1);
							lcd_putc('0'+(subscreenCount-1));
							lcd_puts("0+");
							for(i=0; i<10; i++)
							{
								uint8_t fnum = (10*(subscreenCount-1))+i;
								lcd_gotoxy((i%5)+3, i/5);
								if(functionMask & ((uint32_t)1 << (fnum)))
								{
									lcd_putc('0' + i);
								}
								else
								{
									lcd_putc(' ');
								}
							}
						}
					}
					else if(2 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("SLEEP");
						lcd_gotoxy(0,1);
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = sleepTimeout_decisecs;
						}
						printDec4Dig((decisecs_tmp+9)/10);
						lcd_gotoxy(5,1);
						lcd_puts("sec");
					}
					else if(3 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("ALERTER");
						lcd_gotoxy(0,1);
						ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
						{
							decisecs_tmp = alerterTimeout_decisecs;
						}
						if(IS_ALERTER_ENABLED)
						{
							printDec4Dig((decisecs_tmp+9)/10);
							lcd_gotoxy(5,1);
							lcd_puts("sec");
						}
						else
						{
							lcd_gotoxy(5,1);
							lcd_puts("OFF");
						}
					}
					else if(4 == subscreenState)
					{
						if(!subscreenCount)
						{
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("ENGINE");
							lcd_gotoxy(0,1);
							lcd_puts("HISTORY");
						}
						else
						{
							if(subscreenCount > ENGINE_STATE_QUEUE_SIZE)
								subscreenCount = ENGINE_STATE_QUEUE_SIZE;
							lcd_gotoxy(0,0);
							printDec2Dig(subscreenCount);
							lcd_putc(':');
							printLocomotiveAddress(engineStatesQueuePeekLocoAddress(subscreenCount-1));
							lcd_gotoxy(0,1);
							printEngineState(engineStatesQueuePeekState(subscreenCount-1));
						}
					}
					else if(5 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("PKT TIME");
						lcd_gotoxy(0,1);
						lcd_putc('[');
						lcd_gotoxy(7,1);
						lcd_putc(']');
						lcd_gotoxy(1,1);
						uint8_t pktTimeout_tmp = (pktTimeout + 6) / 7;
						for(i = 0; i < 6; i++)
						{
							if(pktTimeout_tmp)
							{
								pktTimeout_tmp--;
								lcd_putc(0xFF);
							}
							else
								lcd_putc(' ');
						}
					}
					else if(6 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("RSSI");
						lcd_gotoxy(0,1);
						if(lastRSSI < 255)
						{
							if(lastRSSI < 10)
							{
								lcd_puts("  -");
								lcd_putc('0' + lastRSSI);
							}
							else if(lastRSSI < 100)
							{
								lcd_puts(" -");
								printDec2Dig(lastRSSI);
							}
							else
							{
								lcd_putc('-');
								printDec3Dig(lastRSSI);
							}
							lcd_gotoxy(4,1);
							lcd_puts("dBm ");
						}
						else
						{
							lcd_gotoxy(0,1);
							lcd_puts("NO SIGNL");
						}
					}
					else if(7 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("FT RATIO");
						lcd_gotoxy(1,1);
						uint8_t timeScaleFactor = getTimeScaleFactor();
						uint8_t timeScaleFactor_tmp = (timeScaleFactor/100)%10;
						if(timeScaleFactor_tmp)
							lcd_putc('0' + timeScaleFactor_tmp);
						else
							lcd_putc(' ');
						lcd_putc('0' + (timeScaleFactor/10)%10);
						lcd_putc('.');
						lcd_putc('0' + timeScaleFactor%10);
						lcd_puts(":1");
					}
					else if(8 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BATTERY");
						lcd_gotoxy(1,1);
						lcd_putc('0' + (((getBatteryVoltage()*2)/100)%10));
						lcd_putc('.');
						lcd_putc('0' + (((getBatteryVoltage()*2)/10)%10));
						lcd_putc('0' + ((getBatteryVoltage()*2)%10));
						lcd_putc('V');
					}
					else if(9 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("VERSION");
						lcd_gotoxy(0,1);
						lcd_puts(VERSION_STRING);
					}
					else if(10 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("GIT REV");
						lcd_gotoxy(1,1);
						printHex((GIT_REV >> 16) & 0xFF);
						printHex((GIT_REV >> 8) & 0xFF);
						printHex(GIT_REV & 0xFF);
					}
					else if(11 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE TYP");
						lcd_gotoxy(0,1);
						lcd_puts(baseString);
					}
					else if(12 == subscreenState)
					{
						enableLCDBacklight();
						lcd_gotoxy(0,0);
						lcd_puts("BASE REV");
						lcd_gotoxy(1,1);
						printHex((baseVersion >> 16) & 0xFF);
						printHex((baseVersion >> 8) & 0xFF);
						printHex(baseVersion & 0xFF);
					}
					else if(13 == subscreenState)
					{
						if(resetCounter)
						{
							enableLCDBacklight();
							lcd_gotoxy(0,0);
							lcd_puts("FACTORY");
							lcd_gotoxy(0,1);
							lcd_puts("RESET ");
							lcd_putc('0' + resetCounter);
							lcd_putc(0x7E);
						}
						else
						{
							lcd_clrscr();
							lcd_gotoxy(0,0);
							lcd_puts("RESET!!!");
							resetConfig();
							while(1);  // Force a watchdog reset
						}
					}
					else if((14 == subscreenState) && (configBits & _BV(CONFIGBITS_AIRBRAKE)))
					{
						// AIRBRAKE DIAGS - the raw air-brake text readout (was the AIRBRAKE screen's
						// second line before AIRBRAKE got its own glyph design). Page 14, but the MENU handler
						// visits it right after the throttle-status page (1), so it reads as the
						// second diag page. Plain ASCII, so the LCD_DIAGS CGRAM mode loaded on entry
						// is harmless. SELECT / MENU are handled by the shared DIAGS subscreen switch
						// below (SELECT -> landing; MENU -> SLEEP page).
						//   Row 0: P<brake-pipe> R<reservoir>, whole PSI. Pipe is normally 2 digits with a
						//     blank gap column before R; at 100+ the hundreds digit fills that gap column.
						//   Row 1: L<lever%> then a letter per AIRBRAKE function while it asserts -
						//          R = BRK REL, S = BRK SET pulse, C1/C2 = COMPRESSOR (synchronised /
						//          routine run - digit only shown when COMPMODE = CONSIST, bare C
						//          otherwise), E = emergency dump. While the compressor is off, the C
						//          column instead shows '*' (COMPMODE = CONSIST only) when the pending
						//          consist-sync credit already clears the deep/COMPRSR threshold - lets
						//          you watch it accumulate and leak away between runs.
						enableLCDBacklight();
						uint8_t bpPsi = airBrakePipePsi();
						lcd_gotoxy(0,0);
						lcd_putc('P');
						if(bpPsi >= 100)
						{
							// BP CHARGE can exceed 99 - the hundreds digit takes the P/R gap column.
							printDec3Dig(bpPsi);
						}
						else
						{
							printDec2Dig(bpPsi);
							lcd_putc(' ');
						}
						lcd_putc('R');
						printDec3Dig(airMainResPsi());
						lcd_gotoxy(0,1);
						lcd_putc('L');
						printDec2Dig(min(brakePcnt,99));
						lcd_putc(airBrakeReleased() ? 'R' : ' ');
						lcd_putc(airBrakeSetPulse() ? 'S' : ' ');
						if(airCompressorOn())
							lcd_putc('C');
						else if((AIRBRAKE_COMP_MODE_CONSIST == airbrakeGet(AIRBRAKE_COMP_MODE)) && airCompressorPendingRelease())
							lcd_putc('*');
						else
							lcd_putc(' ');
						lcd_putc((airCompressorOn() && (AIRBRAKE_COMP_MODE_CONSIST == airbrakeGet(AIRBRAKE_COMP_MODE)))
						             ? (airCompressorReleaseRun() ? '1' : '2') : ' ');
						lcd_putc(airEmergencyActive() ? 'E' : ' ');
					}
					else
					{
						subscreenState = 1;
					}

					switch(button)
					{
						case SELECT_BUTTON:
							if(SELECT_BUTTON != previousButton)
							{
								setupLCD(LCD_DEFAULT);   // Restore default characters
								subscreenState = 0;  // Escape submenu
								subscreenCount = 0;
								lcd_clrscr();
							}
							break;
						case MENU_BUTTON:
							if(MENU_BUTTON != previousButton)
							{
								// Menu pressed, advance to the next diag page. AIRBRAKE DIAGS (page 14) is
								// slotted in right after the throttle-status page (1) when AIRBRAKE is
								// enabled, and skipped entirely otherwise; page 13 (FACTORY RESET)
								// wraps back to 1.
								if(13 == subscreenState)
									subscreenState = 1;
								else if(14 == subscreenState)
									subscreenState = 2;
								else if((1 == subscreenState) && (configBits & _BV(CONFIGBITS_AIRBRAKE)))
									subscreenState = 14;
								else
									subscreenState++;
								subscreenCount = 0;
								lcd_clrscr();
								resetCounter = RESET_COUNTER_RESET_VALUE;
							}
							break;
						case DOWN_BUTTON:
							if(DOWN_BUTTON != previousButton)
							{
								// Decrement here blindly, but it's only used in the reset screen
								// It will be reset anyway prior to entering reset screen
								resetCounter--;
								
								if(subscreenCount)
								{
									subscreenCount--;
									lcd_clrscr();
								}
							}
							break;
						case UP_BUTTON:
							if(UP_BUTTON != previousButton)
							{
								// Decrement here blindly, but it's only used in the reset screen
								// It will be reset anyway prior to entering reset screen
								resetCounter--;
								
								if(subscreenCount < 255)
								{
									// Will be limited where used above
									subscreenCount++;
									lcd_clrscr();
								}
							}
						case NO_BUTTON:
							break;
					}
				}
				break;

			case LAST_SCREEN:
				// Clean up and reset
				lcd_clrscr();
				// Restore the default custom characters - any screen that reprogrammed CGRAM
				// (e.g. DIAG_SCREEN's LCD_DIAGS glyphs) funnels through here on exit, and
				// setupLCD()'s currentMode guard makes this a no-op when nothing changed.
				setupLCD(LCD_DEFAULT);
				screenState = 0;
				break;
		}
		// Process Menu button, but only if not in a subscreen.
		// Do this after main screen loop so screens can also do cleanup when menu is pressed.
		// OPS_MODE_SCREEN (and the AIRBRAKE screen reached from it or from a base-screen AIRBRAKE
		// button) own MENU entirely - MENU is a function button in OPS MODE and a screen-dismiss on
		// those AIRBRAKE sessions, not a menu-cycle key - so this whole block is bypassed for them.
		if((OPS_MODE_SCREEN != screenState) && !airbrakeReturnToOps && !airbrakeReturnToMain)
		{
		if(!subscreenState)
		{
			uint8_t doAdvance = 0;

			if(MENU_BUTTON == button)
			{
				if(MENU_BUTTON != previousButton)
				{
					ticks_autoincrement = 0;  // Reset to zero so a long press can be detected
					// A fresh MENU press from the base screen with OPS MODE enabled is deferred - it
					// may become the long-press that enters OPS MODE, and advancing to ENGINE first
					// would flash that screen. menuAdvancePending is a one-shot: it resolves either
					// into OPS MODE (long-press) or into a normal advance on release (short tap).
					// Every other screen, and the entire OPS-disabled build, advances immediately on
					// the press edge exactly as before.
					if((MAIN_SCREEN == screenState) && (configBits & _BV(CONFIGBITS_OPS_MODE)))
					{
						menuAdvancePending = 1;
					}
					else
					{
						menuAdvancePending = 0;
						doAdvance = 1;
					}
				}
				if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
				{
					if(menuAdvancePending)
					{
						// Long-press MENU from the base screen -> enter OPS MODE directly, without
						// ever advancing (no ENGINE flash). opsMenuIgnoreUntilRelease keeps this same
						// still-held MENU from immediately tripping OPS MODE's own exit long-press.
						menuAdvancePending = 0;
						screenState = OPS_MODE_SCREEN;
						opsMenuIgnoreUntilRelease = 1;
						lcd_clrscr();
					}
					// (MAIN_SCREEN != screenState): once a long-press has already landed back on the
					// main screen, stop re-firing every pass while MENU stays held - otherwise the
					// screen bounces main -> LAST_SCREEN -> main (a visible CGRAM reload flicker,
					// since the base screen uses its own LCD_MAIN / LCD_MAIN_SPEED CGRAM set, distinct
					// from LAST_SCREEN's LCD_DEFAULT).
					else if(MAIN_SCREEN != screenState)
					{
						// Reset menu on long press
						screenState = LAST_SCREEN;
					}
				}
			}
			else if((NO_BUTTON == button) && (MENU_BUTTON == previousButton) && menuAdvancePending)
			{
				// Trailing edge of a short MENU tap on the base screen - advance the menu now.
				menuAdvancePending = 0;
				doAdvance = 1;
			}

			if(doAdvance)
			{
				// Menu pressed, advance menu
				lcd_clrscr();
				// Restore the default CGRAM. AIRBRAKE_SCREEN's DISPLAY=SINGLE view leaves
				// LCD_AIRBRAKE_ALT active (all 8 slots = gauge artwork); advancing from it via MENU
				// would otherwise land on AIRBRAKE_CONFIG_SCREEN with slots 6/7 still holding gauge
				// fragments instead of the PSI glyph. currentMode guard makes this free on every
				// other menu advance (same backstop as case LAST_SCREEN).
				setupLCD(LCD_DEFAULT);
				screenState++;  // No range checking needed since LAST_SCREEN will reset the counter
				ticks_autoincrement = 0;  // Reset to zero so a long press can be detected

				// Check for conditional menus
				if(!(systemBits & _BV(SYSTEMBITS_ADV_FUNC)))
				{
					// Advanced functions NOT active
					if(THRESHOLD_CAL_SCREEN == screenState)
					{
						// Horn2's threshold is deliberately NOT in this gate - its calibration is
						// optional (hornThreshold2 == 0xFF just means Horn2 is disabled), so an
						// upgraded throttle isn't forced back through THRESHOLD CAL for it.
						if(	(0xFF != hornThreshold) &&
							(0xFF != brakeThreshold) &&
							(0xFF != brakeLowThreshold) &&
							(0xFF != brakeHighThreshold)
							)
						{
							// Skip threshold menu, but only if already calilbrated
							screenState++;
						}
					}
				}

				// Skip SPEED CFG screen when the main screen is showing the clock, not speed -
				// tuning these settings is meaningless if the throttle isn't displaying speed at all.
				if(SPEED_CONFIG_SCREEN == screenState)
				{
					if(!(configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)))
					{
						screenState++;
					}
				}

				// Skip AIRBRAKE / AIRBRAKE CFG when AIRBRAKE is off - the model still ticks but
				// drives nothing. (AIRBRAKE is still reachable via an AIRBRAKE-bound button.)
				if(AIRBRAKE_SCREEN == screenState)
				{
					if(!(configBits & _BV(CONFIGBITS_AIRBRAKE)))
					{
						screenState++;
					}
				}
				if(AIRBRAKE_CONFIG_SCREEN == screenState)
				{
					if(!(configBits & _BV(CONFIGBITS_AIRBRAKE)))
					{
						screenState++;
					}
				}

				// OPS_MODE_SCREEN is never a MENU-cycle target - it is only reached by a
				// long-press of MENU from the base screen (handled above). Cycling from
				// DIAG_SCREEN skips straight past it to LAST_SCREEN (-> main screen).
				if(OPS_MODE_SCREEN == screenState)
				{
					screenState++;
				}

				if(systemBits & _BV(SYSTEMBITS_MENU_LOCK))
				{
					// Menu lock active
					while( 	(ENGINE_SCREEN != screenState) &&
							(AIRBRAKE_SCREEN != screenState) &&
							(LOAD_CONFIG_SCREEN != screenState) &&
							(LOCO_SCREEN != screenState) &&
							(FORCE_FUNC_SCREEN != screenState) &&
							(SYSTEM_SCREEN != screenState) &&
							(LAST_SCREEN != screenState)
						)
					{
						// Skip menu(s)
						screenState++;
					}
				}
				if(SYSTEM_SCREEN == screenState)
				{
					// systemBits isn't stored in EEPROM, so it needs its own snapshot for the
					// menu-cancel handler (below) to revert it. Captured here, after every skip
					// path above (conditional-menu skips, menu-lock skip), so an indirect entry
					// to SYSTEM_SCREEN can't miss it.
					systemBitsSnapshot = systemBits;
				}
			}
		}
		else if(MENU_BUTTON == button)
		{
			// Long-press Menu while inside a subscreen: cancel and discard any
			// uncommitted edits, then exit all the way to the main screen -
			// same target as the long-press handler above.
			if(ticks_autoincrement >= button_autoincrement_10ms_ticks)
			{
				// Nothing reaches EEPROM without an explicit SELECT-save, so
				// reloading from EEPROM is a full undo of any in-progress edit.
				readConfig();

				// systemBits isn't stored in EEPROM, so readConfig() can't
				// revert it - restore the snapshot taken on entry to SYSTEM_SCREEN.
				if(SYSTEM_SCREEN == screenState)
				{
					systemBits = systemBitsSnapshot;
				}

				// PREFS_SCREEN's SLEEP DLY/ALERTER and COMM_SCREEN's THRTL ID/BASE
				// ADR/TIME ADR/TX INTVL items are staged in these new* locals and
				// only pushed to the real values on save - neither screen resyncs
				// them on entry. Resync from the (readConfig()-restored) real values
				// so a cancelled edit doesn't linger and get silently committed on
				// the next SELECT-save of that item.
				newSleepTimeout = sleep_tmr_reset_value / 600;
				newAlerterTimeout = alerter_tmr_reset_value / 150;
				newDevAddr = mrbus_dev_addr;
				newBaseAddr = mrbus_base_addr;
				newTimeAddr = timeSourceAddress;
				newUpdate_seconds = update_decisecs / 10;

				subscreenState = 0;
				screenState = LAST_SCREEN;
				lcd_clrscr();
			}
		}
		}  // end: OPS_MODE_SCREEN / airbrakeReturnToOps / airbrakeReturnToMain bypass of the top-level MENU handling

		previousButton = button;

		wdt_reset();

		// Handle any packets that may have come in
		if (mrbusPktQueueDepth(&mrbeeRxQueue))
		{
			PktHandler();
		}

		wdt_reset();

		if (0 == pktTimeout)
		{
			baseVersion = 0;
			lastRSSI = 0xFF;
			strcpy(baseString, "  NONE  ");
			led = LED_RED_FASTBLINK;
		}
		else if(configBits & _BV(CONFIGBITS_LED_BLINK))
		{
			//  Blink GREEN unless configred to be off
			led = LED_GREEN;
		}
		else
		{
			led = LED_OFF;
		}
		

		// AIRBRAKE model: once per 10Hz tick (flag from TIMER0_COMPA_vect), run here so it
		// takes the lever percentage as a parameter. Ticked just before the function mask is built so
		// its outputs (BRAKE_REL_FN / BRK SET / COMPRESSOR_FN, applied below) are this pass's values.
		//  arg 2 = independentBrakeAtRest: is the *independent* brake (whichever BRK TYPE is active)
		//          genuinely at rest right now - reuses each mode's own rest boundary instead of a
		//          separate AIRBRAKE-only percentage, so the automatic-brake pipe's apply/release
		//          point tracks how each mode really decides "released". brakeState/currentStackBand
		//          are already current for this pass (the brake-mode dispatch runs earlier in the same
		//          loop iteration, ~1645-1821). Step and Stack have real, wide rest zones of their own
		//          (brakeState holds at BRAKE_LOW_BEGIN/WAIT for the whole 0-20% range in Step;
		//          currentStackBand==0 for band 0, whose upper edge is 25%/17% for 3-STEP/5-STEP) -
		//          Standard's own low-threshold state has no usable width (its state machine leaves
		//          BRAKE_LOW_BEGIN/WAIT the instant brakePosition clears brakeLowThreshold, no margin),
		//          so it shares Pulse's raw fallback instead (Pulse's own brakeState tracks PWM duty-
		//          cycle phase, not lever rest, so it was never usable here either). See CLAUDE.md
		//          "Brake logic".
		//  arg 3 = the BRK ESTP option is on -> a full-lever slam is an emergency application (pipe
		//          dumps to 0); the option flag, not ESTOP_BRAKE, so it still models on AIRBRAKE.
		{
			uint8_t doBrakeTick;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { doBrakeTick = brake10HzTick; brake10HzTick = 0; }
			if(doBrakeTick)
			{
				uint8_t independentBrakeAtRest;
				if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STEP == GET_BRK_TYPE(optionBits)) )
					independentBrakeAtRest = (BRAKE_LOW_BEGIN == brakeState) || (BRAKE_LOW_WAIT == brakeState);
				else if( (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STACK == GET_BRK_TYPE(optionBits)) )
					independentBrakeAtRest = (0 == currentStackBand);
				else
					independentBrakeAtRest = (brakePcnt < 20);   // Standard + Pulse - matches Step's own 20% onset

				updateBrake10Hz(min(brakePcnt,100), independentBrakeAtRest,
				                (optionBits & _BV(OPTIONBITS_ESTOP_ON_BRAKE)) ? 1 : 0);
			}
		}

		// Figure out which functions should be on and which should be off
		functionMask = 0;
		estopStatus &= ~ESTOP_BUTTON;
		// FORCE FUNC audition (26199c7): while editing an F## (SELECT-ed into the subscreen), the horn
		// lever momentarily fires that function instead of HORN_FN/HORN2_FN, so you can try it on the
		// loco before committing. Both stages do it, so it works across the whole lever travel in
		// either HORNTYPE mode. The `&& subscreenState` keeps it off the FORCE FUNC landing page,
		// where functionNumber is stale and nothing on screen shows which function it is.
		if(controls & HORN_CONTROL)
		{
			if((FORCE_FUNC_SCREEN == screenState) && subscreenState)
			{
				functionMask |= (uint32_t)1 << (functionNumber);
			}
			else
			{
				functionMask |= getFunctionMask(HORN_FN);
			}
		}
		if(controls & HORN2_CONTROL)
		{
			if((FORCE_FUNC_SCREEN == screenState) && subscreenState)
			{
				functionMask |= (uint32_t)1 << (functionNumber);
			}
			else
			{
				functionMask |= getFunctionMask(HORN2_FN);
			}
		}
		if(controls & BELL_CONTROL)
			functionMask |= getFunctionMask(BELL_FN);
		if(controls & AUX_CONTROL)
		{
			functionMask |= getFunctionMask(AUX_FN);
			if(isFunctionEstop(AUX_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		if(controls & BRAKE_CONTROL)
			functionMask |= getFunctionMask(BRAKE_FN);
		if(configBits & _BV(CONFIGBITS_AIRBRAKE))
		{
			// AIRBRAKE owns the air functions: BRAKE_REL_FN from the applied latch (its ON edge, a
			// genuine full release, is the brake-release sound), BRAKE_SET_FN as a ~1 s pulse at the
			// start of every brake-pipe reduction (the trainline exhaust hiss), COMPRESSOR_FN/
			// COMPRESSOR2_FN from the reservoir governor. BK2/BK3 and BRAKE_FN stay with the BRK TYPE
			// machine.
			//
			// Heading into sleep: stop asserting the non-latching sound functions (compressor, vent)
			// a few ticks early, while packets still flow, so a final "off" reaches the loco - once
			// the radio sleeps the command station just holds the last state it heard and the
			// compressor sound would play forever. BRAKE_REL_FN is a released/applied state (edge-
			// triggered decoder sound, not continuous), so it's left as-is.
			uint16_t sleepLeft;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { sleepLeft = sleepTimeout_decisecs; }
			uint8_t sleepImminent = (throttleStatus & THROTTLE_STATUS_SLEEP) ||
			                        (sleepLeft <= AIRBRAKE_SLEEP_QUIET_DECISECS);

			if(airBrakeReleased())
				functionMask |= getFunctionMask(BRAKE_REL_FN);
			if(!sleepImminent && airBrakeSetPulse())
				functionMask |= getFunctionMask(BRAKE_SET_FN);
			if(!sleepImminent && airCompressorOn())
			{
				// COMPMODE = CONSIST: split by whether this run is servicing a deep recharge (see
				// airCompressorReleaseRun()/cst-pressure.c) - COMPRSR for that, COMPRSR2 for a
				// routine leak-driven cycle. COMPMODE = NORMAL (default): always COMPRSR, matching
				// pre-split behaviour exactly.
				if((AIRBRAKE_COMP_MODE_CONSIST == airbrakeGet(AIRBRAKE_COMP_MODE)) && !airCompressorReleaseRun())
					functionMask |= getFunctionMask(COMPRESSOR2_FN);
				else
					functionMask |= getFunctionMask(COMPRESSOR_FN);
			}
		}
		else if(controls & BRAKE_REL_CONTROL)
		{
			functionMask |= getFunctionMask(BRAKE_REL_FN);
		}
		if(controls & BK2_CONTROL)
			functionMask |= getFunctionMask(BK2_FN);
		if(controls & BK3_CONTROL)
			functionMask |= getFunctionMask(BK3_FN);
		if((ENGINE_ON == engineState)||(ENGINE_START == engineState))
			functionMask |= getFunctionMask(ENGINE_ON_FN);
		if(ENGINE_STOP == engineState)
			functionMask |= getFunctionMask(ENGINE_OFF_FN);
		if(optionButtonState & UP_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(UP_FN);
			if(isFunctionEstop(UP_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		if(optionButtonState & DOWN_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(DOWN_FN);
			if(isFunctionEstop(DOWN_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		// MENU BTN / SEL BTN (OPS MODE). Their optionButtonState bits are only ever set on the
		// OPS MODE screen, but a latched bit keeps asserting after OPS MODE is left - that persistence
		// is what the base screen's "Fn active" glyph reports.
		if(optionButtonState & MENU_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(MENU_FN);
			if(isFunctionEstop(MENU_FN))
				estopStatus |= ESTOP_BUTTON;
		}
		if(optionButtonState & SEL_OPTION_BUTTON)
		{
			functionMask |= getFunctionMask(SEL_FN);
			if(isFunctionEstop(SEL_FN))
				estopStatus |= ESTOP_BUTTON;
		}

		// LOAD (UP/DOWN/MENU/SEL BTN): asserts whichever DCC function OPLOADFN/PRLOADFN is configured
		// to in SPEED CFG, driven by the button's own persistent 3-way cycle state rather than by
		// optionButtonState - LOAD has no functionMask bit of its own (getFunctionMask() returns 0 for
		// it), so this asserts the target bit directly. Unconditional every pass, not gated on the
		// button currently being held - matches the "continuous-hold-while-condition-is-true" idiom
		// used elsewhere (e.g. NEUTRAL_FN) rather than a momentary/latching read. Runs before the
		// STOPFN/OPLOADFN/PRLOADFN/HOLDFN scan below, so that scan picks up LOAD's contribution
		// automatically, regardless of source, with no changes needed to it.
		if(loadActive(UP_FN))   functionMask |= speedLoadFunctionMask(loadModeUp);
		if(loadActive(DOWN_FN)) functionMask |= speedLoadFunctionMask(loadModeDown);
		if(loadActive(MENU_FN)) functionMask |= speedLoadFunctionMask(loadModeMenu);
		if(loadActive(SEL_FN))  functionMask |= speedLoadFunctionMask(loadModeSel);

		if(controls & THR_UNLK_CONTROL)
			functionMask |= getFunctionMask(THR_UNLOCK_FN);

		if(NEUTRAL == activeReverserSetting)
		{
			functionMask |= getFunctionMask(NEUTRAL_FN);
		}

		wdt_reset();

		switch(frontLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				functionMask |= getFunctionMask(FRONT_DIM1_FN);
				functionMask |= getFunctionMask(FRONT_DIM2_FN);
				break;
			case LIGHT_BRIGHT:
				functionMask |= getFunctionMask(FRONT_HEADLIGHT_FN);
				break;
			case LIGHT_BRIGHT_DITCH:
				functionMask |= getFunctionMask(FRONT_HEADLIGHT_FN);
				functionMask |= getFunctionMask(FRONT_DITCH_FN);
				break;
		}

		wdt_reset();

		switch(rearLight)
		{
			case LIGHT_OFF:
				break;
			case LIGHT_DIM:
				functionMask |= getFunctionMask(REAR_DIM1_FN);
				functionMask |= getFunctionMask(REAR_DIM2_FN);
				break;
			case LIGHT_BRIGHT:
				functionMask |= getFunctionMask(REAR_HEADLIGHT_FN);
				break;
			case LIGHT_BRIGHT_DITCH:
				functionMask |= getFunctionMask(REAR_HEADLIGHT_FN);
				functionMask |= getFunctionMask(REAR_DITCH_FN);
				break;
		}

		uint16_t alerterTimeout_decisecs_tmp;
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			alerterTimeout_decisecs_tmp = alerterTimeout_decisecs;
		}
		throttleStatus &= ~THROTTLE_STATUS_ALERTER;  // Clear here, (re)set below.
		estopStatus &= ~ESTOP_ALERTER;
		if(IS_ALERTER_ENABLED)
		{
			if(alerterTimeout_decisecs_tmp < 100)
			{
				functionMask |= getFunctionMask(ALERTER_FN);
				lastFunctionMask |= getFunctionMask(ALERTER_FN);  // Fake lastFunctionMask so this doesn't trigger inputsChanged below and reset alerter timer
				throttleStatus |= THROTTLE_STATUS_ALERTER;
			}  // Fall through if...
			if(0 == alerterTimeout_decisecs_tmp)
			{
				// Throttle down to idle and apply brakes
				activeThrottleSetting = 0;
				lastActiveThrottleSetting = 0;
				functionMask |= getFunctionMask(BRAKE_FN);
				if(isFunctionEstop(ALERTER_FN))
					estopStatus |= ESTOP_ALERTER;
				lastFunctionMask |= getFunctionMask(BRAKE_FN);  // Fake lastFunctionMask so this doesn't trigger inputsChanged below and reset alerter timer
			}
		}

		// Force specific functions on or off
		functionMask |= functionForceOn;
		functionMask &= ~functionForceOff;

		// Speed sim: are the user's watched DCC functions (STOPFN/OPLOADFN/PRLOADFN) currently part
		// of the outgoing mask, regardless of which physical control put them there?
		{
			uint8_t watchFn = speedGet(SPEED_ITEM_STOP_FN);
			stopFunctionActive = (watchFn <= 28) && (functionMask & ((uint32_t)1 << watchFn));
			uint8_t opFn = speedGet(SPEED_ITEM_OPLOAD_FN);
			oploadFunctionActive = (opFn <= 28) && (functionMask & ((uint32_t)1 << opFn));
			uint8_t prFn = speedGet(SPEED_ITEM_PRLOAD_FN);
			prloadFunctionActive = (prFn <= 28) && (functionMask & ((uint32_t)1 << prFn));
			uint8_t holdFn = speedGet(SPEED_ITEM_HOLD_FN);
			holdFunctionActive = (holdFn <= 28) && (functionMask & ((uint32_t)1 << holdFn));

			// Same mechanism, for Brake1/2/3 (BRAKE_FN/BK2_FN/BK3_FN) - forced false in Step mode, since
			// Step's brake pulses advance a TCS-style ratchet on the real decoder rather than meaning
			// "hold to brake", which the sim's held-while-active model can't represent regardless of
			// how the active state is detected.
			uint8_t stepBrakeMode = (optionBits & _BV(OPTIONBITS_VARIABLE_BRAKE)) && (BRK_TYPE_STEP == GET_BRK_TYPE(optionBits));
			brake1FunctionActive = (!stepBrakeMode && (functionMask & getFunctionMask(BRAKE_FN))) ? 1 : 0;
			brake2FunctionActive = (!stepBrakeMode && (functionMask & getFunctionMask(BK2_FN))) ? 1 : 0;
			brake3FunctionActive = (!stepBrakeMode && (functionMask & getFunctionMask(BK3_FN))) ? 1 : 0;
		}

		wdt_reset();

		// Process various E-Stop inputs to create single status bit
		if(estopStatus)
			throttleStatus |= THROTTLE_STATUS_EMERGENCY;
		else
			throttleStatus &= ~THROTTLE_STATUS_EMERGENCY;

		// EMRG FN: the operator's configured function, asserted for as long as the throttle is in
		// emergency (any source - brake slam, an FN_EMRG control, alerter-as-estop). Done here, after
		// the status bit is final and while functionMask is still being built.
		if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
			functionMask |= getFunctionMask(EMERGENCY_FN);

		// Scale-speed sim: once per 10Hz tick (flag set by TIMER0_COMPA_vect), but run from here, not
		// the ISR - its standing-start ramp does 64-bit math that must not stall the radio/encoder
		// interrupts. Placed after every input it reads is this pass's value (commandedSpeedStep, the
		// brake/watch mirrors above, and THROTTLE_STATUS_EMERGENCY just finalized).
		{
			uint8_t doSpeedTick;
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { doSpeedTick = speed10HzTick; speed10HzTick = 0; }
			if(doSpeedTick)
				updateSpeed10Hz(commandedSpeedStep, brake1FunctionActive, brake2FunctionActive,
				                brake3FunctionActive, throttleStatus & THROTTLE_STATUS_EMERGENCY,
				                stopFunctionActive, oploadFunctionActive, prloadFunctionActive,
				                holdFunctionActive);
		}

		uint8_t nonFnInputsChanged =	(activeReverserSetting != lastActiveReverserSetting) ||
									(activeThrottleSetting != lastActiveThrottleSetting) ||
									((throttleStatus & THROTTLE_STATUS_EMERGENCY) != (lastThrottleStatus & THROTTLE_STATUS_EMERGENCY));
									// Look at just EMERG bit since other bits are used for sleep and alerter

		uint8_t inputsChanged = nonFnInputsChanged || (functionMask != lastFunctionMask);

		// AIRBRAKE's compressor governor and vent one-shot flip their own function bits on internal
		// timers even while the throttle sits idle. Those changes must still be transmitted (inputsChanged,
		// above, gates TX), but they must not read as crew activity for the sleep / alerter timeouts, or a
		// throttle with AIRBRAKE on could never nod off (nor its alerter fire). Everything else - including
		// BRAKE_REL_FN, which only flips on a real brake apply/release - still counts.
		uint32_t idleFnMask = (configBits & _BV(CONFIGBITS_AIRBRAKE))
			? (getFunctionMask(COMPRESSOR_FN) | getFunctionMask(COMPRESSOR2_FN) | getFunctionMask(BRAKE_SET_FN)) : 0;
		uint8_t activityForTimeout = nonFnInputsChanged ||
			((functionMask & ~idleFnMask) != (lastFunctionMask & ~idleFnMask));

		// AIRBRAKE is a read-only viewport that suppresses nothing, so working the brake lever
		// there puts real brake bits into functionMask and already counts as activity above - it
		// needs no special sleep/alerter hold, and a genuinely idle AIRBRAKE screen times out to
		// sleep like every other menu screen.

		// Reset sleep timer
		// Using activeReverserSetting also guarantees the throttle (activeThrottleSetting) was in idle when entering sleep, so it will unsleep in the idle position.
		if(
			(NO_BUTTON != button) ||
			activityForTimeout ||
			((configBits & _BV(CONFIGBITS_STRICT_SLEEP)) && ((FORWARD == activeReverserSetting) || (REVERSE == activeReverserSetting)))
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
			}
		}

		// Reset alerter timer
		// Using activeReverserSetting also guarantees the throttle (activeThrottleSetting) was in idle when entering sleep, so it will unsleep in the idle position.
		if(
			(NO_BUTTON != button) ||
			activityForTimeout ||
			(NEUTRAL == activeReverserSetting)
			)
		{
			if(alerterTimeout_decisecs_tmp || ((!alerterTimeout_decisecs_tmp) && (NEUTRAL == activeReverserSetting)))
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				alerterTimeout_decisecs = alerter_tmr_reset_value;
			}
		}

		// Hold the LCD backlight on while navigating the menus, and for a short grace period after
		// returning to the main screen, so cycling back through for another pass doesn't strobe the
		// light off in between. Only MENU arms the hold from the main screen (UP/DOWN function taps
		// with the backlight off stay dark); every non-main screen keeps it armed continuously.
		// OPS_MODE_SCREEN is a base-screen variant where every button is a function tap, so it honours
		// the backlight toggle exactly like the main screen (the hold armed while entering covers the
		// transition). The AIRBRAKE screen reached from a running screen (OPS MODE / an AIRBRAKE
		// button) is the same running-screen character, so it is excluded here too.
		{
			uint8_t airbrakeFromRunning = (AIRBRAKE_SCREEN == screenState) && (airbrakeReturnToOps || airbrakeReturnToMain);
			if(((MENU_BUTTON == button) || (MAIN_SCREEN != screenState)) && (OPS_MODE_SCREEN != screenState) && !airbrakeFromRunning)
				backlightTimeout_decisecs = BACKLIGHT_HOLD_DECISECS;
		}

		wdt_reset();

		// New packet criteria: Stuff changed ...or... it's been more than the transmission timeout
		//    ...and there's room in the queue
		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = decisecs;
		}
		if (
				adcLoopInitialized() &&
				((inputsChanged) || (decisecs_tmp >= update_decisecs)) &&
				!(mrbusPktQueueFull(&mrbeeTxQueue))
			)
		{
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				decisecs = 0;
			}
			inputsChanged = 0;
			lastActiveReverserSetting = activeReverserSetting;
			lastActiveThrottleSetting = activeThrottleSetting;
			lastFunctionMask = functionMask;
			lastThrottleStatus = throttleStatus;
			
			txBuffer[MRBUS_PKT_DEST] = mrbus_base_addr;
			txBuffer[MRBUS_PKT_SRC] = mrbus_dev_addr;
			txBuffer[MRBUS_PKT_LEN] = 15;
			txBuffer[5] = 'S';

			txBuffer[6] = locoAddress >> 8;
			txBuffer[7] = locoAddress & 0xFF;

			if(throttleStatus & THROTTLE_STATUS_EMERGENCY)
			{
				txBuffer[8] = 1;  // E-stop				
			}
			else if(0 == activeThrottleSetting)
				txBuffer[8] = 0;
			else
				txBuffer[8] = notchSpeedStep[activeThrottleSetting-1] + 1;
			
			switch(activeReverserSetting)
			{
				case FORWARD:
					direction = FORWARD;
					break;
				case REVERSE:
					direction = REVERSE;
					break;
				case NEUTRAL:
					// Preserve previous direction for DCC purposes (mainly if auto-directional lighting is enabled)
					break;
			}
			
			if(FORWARD == direction)
				txBuffer[8] |= 0x80;
			
			txBuffer[9]  = functionMask >> 24;
			txBuffer[10] = functionMask >> 16;
			txBuffer[11] = functionMask >> 8;
			txBuffer[12] = functionMask & 0xFF;

			txBuffer[13] = throttleStatus;

			txBuffer[14] = getBatteryVoltage();	
			mrbusPktQueuePush(&mrbeeTxQueue, txBuffer, txBuffer[MRBUS_PKT_LEN]);
		}

		// Transmission criteria: something in the buffer ...and... it's been more than the minimum holdoff
		if (mrbusPktQueueDepth(&mrbeeTxQueue) && !txHoldoff)
		{
			wdt_reset();
			mrbeeTransmit();
			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				txHoldoff = txHoldoff_centisecs;
			}
		}

		ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
		{
			decisecs_tmp = sleepTimeout_decisecs;
		}
		if((0 == decisecs_tmp) || (throttleStatus & THROTTLE_STATUS_SLEEP))
		{
			wdt_reset();

			// Time to nod off
			led = LED_OFF;

			// Disable internal power-sucking peripherals (need to be enabled when awake)
			disableADC();
			setXbeeSleep();
			disableLCD();
			disableLCDBacklight();
			disableTimer();  // Disable 100Hz timer (to prevent LEDs from blinking on right before sleeping)
			ledGreenOff();
			ledRedOff();
			disableSwitches();  // Don't disable buttons
			disableThrottle();
			
			// Reinforce that these are off
			disablePots();
			disableReverser();
			disableLightSwitches();

			set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // set the type of sleep mode to use
			cli();
			wdt_reset();
			wdt_disable();                         // Disable watchdog so it doesn't reset us in sleep
			PCIFR |= _BV(PCIF1);                   // Clear any pending interrupts, maybe from the button release on the last sleep
			PCICR |= _BV(PCIE1);                   // Enable Pin Change Interrupt Bank 1 (softkey interrupts)
			PCMSK1 |= _BV(PCINT15) | _BV(PCINT14) | _BV(PCINT13) | _BV(PCINT12);  // Enable interrupts on softkeys
			sleep_enable();                        // Enable sleep mode
			sei();
			sleep_cpu();
			cli();
			sleep_disable();                       // Disable sleep mode
			PCICR &= ~_BV(PCIE1);                  // Disable Pin Change Interrupt Bank 1
			wdt_reset();
			wdt_enable(WATCHDOG_TIMEOUT);          // Reenable watchdog
			wdt_reset();
			sei();

			// Re-enable chip internal bits (ADC, pots, reverser, light switches done in ADC loop)
			setXbeeActive();
			enableLCD();
			initLCD();
			if(DIAG_SCREEN == screenState)
			{
				// Change LCD chars if in the DIAG screen
				setupLCD(LCD_DIAGS);
			}
			initialize100HzTimer();
			enableSwitches();
			enableThrottle();

			// Initialize the buttons so there are no startup artifacts when we actually use them
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			inputButtons = debounce(inputButtons, (PINB & (0xF6)));
			processButtons(inputButtons);
			processSwitches(inputButtons);
			previousButton = button;  // Prevent extraneous menu advances
			clearDeadReckoningTime();

			ATOMIC_BLOCK(ATOMIC_RESTORESTATE)
			{
				sleepTimeout_decisecs = sleep_tmr_reset_value;
				alerterTimeout_decisecs = alerter_tmr_reset_value;
			}
			throttleStatus &= ~THROTTLE_STATUS_SLEEP;
		}

	}

}


