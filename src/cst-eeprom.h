#ifndef _CST_EEPROM_H_
#define _CST_EEPROM_H_

#include <stdint.h>

#define EE_VERSION_MAJOR              0x0E
#define EE_VERSION_MINOR              0x0F

// EEPROM *layout* version, independent of EE_VERSION_MAJOR/MINOR above (those are parsed from
// VERSION_STRING/git describe --tags, which stays a degenerate ".0+" on a checkout with no tags - not a
// usable signal for this). EEPROM_LAYOUT_VERSION is a plain hand-bumped integer, incremented only when
// the EEPROM layout itself changes (new field, moved offset, repurposed byte) - written unconditionally
// into EE_LAYOUT_VERSION by readConfig(), same pattern as EE_VERSION_MAJOR/MINOR. Lets offline tooling
// (src/cst-cfgtransfer/) detect a layout mismatch against the connected chip and refuse rather than
// silently misdecode. Bump this alongside any cst-eeprom.h layout change - see CLAUDE.md.
#define EEPROM_LAYOUT_VERSION          5

//                                    0x10
#define EE_DEVICE_SLEEP_TIMEOUT       0x11
#define EE_DEAD_RECKONING_TIME        0x12
#define EE_CONFIGBITS                 0x13
#define EE_BATTERY_OKAY               0x14
#define EE_BATTERY_WARN               0x15
#define EE_BATTERY_CRITICAL           0x16
#define EE_TX_HOLDOFF                 0x1D
#define EE_TIME_SOURCE_ADDRESS        0x1E
#define EE_BASE_ADDR                  0x1F
#define EE_HORN_THRESHOLD             0x20
#define EE_BRAKE_THRESHOLD            0x21
#define EE_BRAKE_LOW_THRESHOLD        0x22
#define EE_BRAKE_HIGH_THRESHOLD       0x23
#define EE_PRESSURE_CONFIG            0x24  // legacy ISE brake-test gauge pump-rate; inert - unused by the firmware (AIRBRAKE superseded it) and no longer round-tripped by the PC tooling.
#define EE_ALERTER_TIMEOUT            0x25
#define EE_LAYOUT_VERSION             0x26
#define EE_HORN_THRESHOLD2            0x27

// 20 configs * 128 bytes = 2560 bytes
//  +128 bytes for global = 2688 bytes

//        Total available = 4096 bytes

#define MAX_CONFIGS      20

// Put working (scratchspace) config at the end of EEPROM space
#define WORKING_CONFIG   31

#define CONFIG_START                  0x80
#define CONFIG_SIZE                   0x80

#define CONFIG_OFFSET(cfgNum)         (((cfgNum - 1) * CONFIG_SIZE) + CONFIG_START)

#define EE_LOCO_ADDRESS               (0x00 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_LOCO_ADDRESS                0x01
#define EE_HORN_FUNCTION              (0x02 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BELL_FUNCTION              (0x03 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_FUNCTION             (0x04 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_AUX_FUNCTION               (0x05 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ENGINE_ON_FUNCTION         (0x06 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ENGINE_OFF_FUNCTION        (0x07 + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_FRONT_DIM1_FUNCTION        (0x08 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_DIM2_FUNCTION        (0x09 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_HEADLIGHT_FUNCTION   (0x0A + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_FRONT_DITCH_FUNCTION       (0x0B + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DIM1_FUNCTION         (0x0C + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DIM2_FUNCTION         (0x0D + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_HEADLIGHT_FUNCTION    (0x0E + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REAR_DITCH_FUNCTION        (0x0F + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_UP_BUTTON_FUNCTION         (0x10 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_DOWN_BUTTON_FUNCTION       (0x11 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_THR_UNLOCK_FUNCTION        (0x12 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_REL_FUNCTION         (0x13 + CONFIG_OFFSET(WORKING_CONFIG))  // "BRK REL" (was BRK OFF)
#define EE_REV_SWAP_FUNCTION          (0x14 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_EMERGENCY_FUNCTION         (0x15 + CONFIG_OFFSET(WORKING_CONFIG))  // "EMRG FN" - asserted while THROTTLE_STATUS_EMERGENCY
#define EE_BRAKE_PULSE_WIDTH          (0x16 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_OPTIONBITS                 (0x17 + CONFIG_OFFSET(WORKING_CONFIG))

#define EE_FORCE_FUNC_ON              (0x18 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_FORCE_FUNC_ON               0x19
//      EE_FORCE_FUNC_ON               0x1A
//      EE_FORCE_FUNC_ON               0x1B
#define EE_FORCE_FUNC_OFF             (0x1C + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_FORCE_FUNC_OFF              0x1D
//      EE_FORCE_FUNC_OFF              0x1E
//      EE_FORCE_FUNC_OFF              0x1F

#define EE_NOTCH_SPEEDSTEP            (0x20 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_NOTCH_SPEEDSTEP             0x21
//      EE_NOTCH_SPEEDSTEP             0x22
//      EE_NOTCH_SPEEDSTEP             0x23
//      EE_NOTCH_SPEEDSTEP             0x24
//      EE_NOTCH_SPEEDSTEP             0x25
//      EE_NOTCH_SPEEDSTEP             0x26
//      EE_NOTCH_SPEEDSTEP             0x27

//                                     0x28  (was EE_BK1_FUNCTION - STACK brake mode's Brake1 slot
//                                            now reuses EE_BRAKE_FUNCTION instead)
#define EE_BK2_FUNCTION               (0x29 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BK3_FUNCTION               (0x2A + CONFIG_OFFSET(WORKING_CONFIG))
//                                     0x2B  EE_MOMENTUM_BRAKE1_CV179 (SPEED, defined below)
#define EE_MENU_BUTTON_FUNCTION       (0x2C + CONFIG_OFFSET(WORKING_CONFIG))  // "MENU BTN" - OPS MODE function button (was a freed SPEED BRK2 scatter slot)
#define EE_SEL_BUTTON_FUNCTION        (0x2D + CONFIG_OFFSET(WORKING_CONFIG))  // "SEL BTN"  - OPS MODE function button (was a freed SPEED BRK3 scatter slot)
//                                     0x2E  EE_MOMENTUM_DECEL_CV4 (SPEED, defined below)
//                                     0x2F

#define EE_COMPRESSOR_FUNCTION        (0x30 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_SET_FUNCTION         (0x31 + CONFIG_OFFSET(WORKING_CONFIG))  // "BRK SET" - AIRBRAKE brake-pipe-reduction pulse (was BRK VENT / BRK TEST)
#define EE_NEUTRAL_FUNCTION           (0x32 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ALERTER_FUNCTION           (0x33 + CONFIG_OFFSET(WORKING_CONFIG))

// STACK band->combo storage encoding. A stored byte is OR'd straight into the mrbw-cst.c `controls`
// word, so STACK_COMBO_BRK1/2/3 MUST equal BRAKE_CONTROL/BK2_CONTROL/BK3_CONTROL there - mrbw-cst.c
// carries a _Static_assert to lock the two in step. Defined here (with the factory defaults) so the
// EEPROM_LAYOUT_VERSION 1->2 migration's repackDefault[] table can name them from cst-eeprom.c
// without depending on the monolith.
//   Both variants' steps are ordered by escalating *total* decoder brake force, not by which brake is
//   used - for the CV mix these target (Brake1 130, Brake2 70, Brake3 100) 5-STEP runs
//   100/130/170/200/230 and 3-STEP runs 70/100/130. All three brakes together (~300, past the
//   decoder's 255 cap = near-instant stop) is deliberately NOT a step - it is left for a dedicated
//   emergency-stop control, not normal lever travel.
#define STACK_COMBO_BRK1        0x08
#define STACK_COMBO_BRK2        0x20
#define STACK_COMBO_BRK3        0x40
#define STACK_COMBO_MASK        (STACK_COMBO_BRK1 | STACK_COMBO_BRK2 | STACK_COMBO_BRK3)
#define STACK_5STEP_DEFAULT_1   STACK_COMBO_BRK3
#define STACK_5STEP_DEFAULT_2   STACK_COMBO_BRK1
#define STACK_5STEP_DEFAULT_3   (STACK_COMBO_BRK2 | STACK_COMBO_BRK3)
#define STACK_5STEP_DEFAULT_4   (STACK_COMBO_BRK1 | STACK_COMBO_BRK2)
#define STACK_5STEP_DEFAULT_5   (STACK_COMBO_BRK1 | STACK_COMBO_BRK3)
#define STACK_3STEP_DEFAULT_1   STACK_COMBO_BRK2
#define STACK_3STEP_DEFAULT_2   STACK_COMBO_BRK3
#define STACK_3STEP_DEFAULT_3   STACK_COMBO_BRK1

// STACK brake mode's 5-STEP band->combo mapping, one byte per band 1-5 (band 0 is fixed to "none" and
// isn't stored) - the storage encoding is above. The 3-STEP variant (see EE_STACK_BAND_COMBOS_3STEP
// below) is stored completely separately, so toggling between the two never cross-contaminates one
// variant's configured combos with the other's.
#define EE_STACK_BAND_COMBOS          (0x34 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_STACK_BAND_COMBOS           0x35
//      EE_STACK_BAND_COMBOS           0x36
//      EE_STACK_BAND_COMBOS           0x37
//      EE_STACK_BAND_COMBOS           0x38

// Scale-speed simulation config (src/cst-speed.c), raw 0-255 values, split in two groups.
//
// Group 1 - decoder-type-agnostic (6 bytes): these mean the same thing for every SPEED_TYPE and keep
// their original scattered addresses. ACCEL/DECEL/BRK1 (CV3/CV4/CV179 mirrors - "momentum" is the
// correct NMRA/ESU term) reuse the isolated single-byte gaps left by EE_BK2_FUNCTION/EE_BK3_FUNCTION;
// MAX_MPH/UNIT/TYPE take the free bytes after EE_STACK_BAND_COMBOS. ACCEL/DECEL are read raw by
// readConfig() (a decoder's literal CV3/CV4 can be 255, so a stored 0xFF is a real 255, not "unset" -
// the layout -> 4 seed migration initialises any never-written byte). BRK1 and every other
// plain-numeric SPEED item still self-heal from 0xFF via readByteOrDefault(), so their max is 254.
#define EE_MOMENTUM_ACCEL_CV3         (0x28 + CONFIG_OFFSET(WORKING_CONFIG))  // CV3, agnostic, raw 0-255
#define EE_MOMENTUM_BRAKE1_CV179      (0x2B + CONFIG_OFFSET(WORKING_CONFIG))  // CV179, agnostic
#define EE_MOMENTUM_DECEL_CV4         (0x2E + CONFIG_OFFSET(WORKING_CONFIG))  // CV4, agnostic, raw 0-255
#define EE_SPEED_MAX_MPH              (0x39 + CONFIG_OFFSET(WORKING_CONFIG))  // scale mph @ speed step 126, agnostic
#define EE_SPEED_UNIT_KMH             (0x3A + CONFIG_OFFSET(WORKING_CONFIG))  // SPEED_UNIT_MPH/_KMH, agnostic
#define EE_SPEED_TYPE                 (0x3C + CONFIG_OFFSET(WORKING_CONFIG))  // SPEED_TYPE_* - tags the decoder family
//                                     0x2C 0x2D  reused by EE_MENU_BUTTON_FUNCTION / EE_SEL_BUTTON_FUNCTION (were BRK2/BRK3 - see EE_SPEED_MODEL_PAYLOAD)
//                                     0x2F       freed (was DELAY - see EE_SPEED_MODEL_PAYLOAD)
//                                     0x3B 0x3D-0x40  freed (were STOPFN/OPLOAD/PRLOAD/OPLOADFN/PRLOADFN)

// STACK brake mode's 3-STEP band->combo mapping, one byte per band 1-3 (band 0 is fixed to "none" and
// isn't stored) - stored separately from EE_STACK_BAND_COMBOS (the 5-STEP variant), see that comment above.
#define EE_STACK_BAND_COMBOS_3STEP    (0x41 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_STACK_BAND_COMBOS_3STEP     0x42
//      EE_STACK_BAND_COMBOS_3STEP     0x43
//                                     0x44-0x48  freed (were HOLDFN/DECTHR/DECPCT/ACCPCT/ACCTGT)

// Horn2 (two-stage horn), per-profile DCC function assignment. hornThreshold2 (the calibration point
// itself) is global, not per-profile - see EE_HORN_THRESHOLD2 above, alongside EE_HORN_THRESHOLD.
#define EE_HORN2_FUNCTION             (0x49 + CONFIG_OFFSET(WORKING_CONFIG))

// AIRBRAKE per-profile model config (src/cst-pressure.c), raw 0-255 bytes. Edited on-device via
// AIRBRAKE_CONFIG_SCREEN; read through readByteOrDefault() so a blank/upgraded chip self-heals to the
// AIRBRAKE_*_DEFAULT values. The rate items (CHARGE_RATE/LEAK_RATE/PUMP_RATE) are PSI/min, scaled to
// per-tick milliPSI in updateBrake10Hz(); the PSI/s items (VENT/EMERG) are hardcoded, not stored - see
// AIRBRAKE_VENT_RATE_PSI_S/AIRBRAKE_EMERG_VENT_RATE_PSI_S in cst-pressure.c - and FULLSVC is derived
// from CHARGED at runtime, also not stored. DISPLAY and COMP_MODE are NORMAL/CONSIST-style toggles.
// EE_COMPRESSOR2_FUNCTION (the "COMPRSR2" Functions-enum slot) lives inside this block because the
// 0x30 function region is fully packed. The old global EE_PRESSURE_CONFIG (0x24) is superseded/unused.
#define EE_AIRBRAKE_CHARGED          (0x4A + CONFIG_OFFSET(WORKING_CONFIG))  // brake-pipe charged pressure, PSI
#define EE_AIRBRAKE_MR_CUTIN         (0x4B + CONFIG_OFFSET(WORKING_CONFIG))  // reservoir governor cut-in, PSI
#define EE_AIRBRAKE_MR_CUTOUT        (0x4C + CONFIG_OFFSET(WORKING_CONFIG))  // reservoir governor cut-out, PSI
#define EE_AIRBRAKE_CHARGE_RATE      (0x4D + CONFIG_OFFSET(WORKING_CONFIG))  // brake-pipe recharge rate, PSI/min
#define EE_AIRBRAKE_LEAK_RATE        (0x4E + CONFIG_OFFSET(WORKING_CONFIG))  // reservoir base leak, PSI/min
#define EE_AIRBRAKE_PUMP_RATE        (0x4F + CONFIG_OFFSET(WORKING_CONFIG))  // compressor fill rate, PSI/min
#define EE_AIRBRAKE_MR_LOAD          (0x50 + CONFIG_OFFSET(WORKING_CONFIG))  // reservoir draw per PSI of pipe recharge, %
#define EE_AIRBRAKE_COMP_MODE        (0x51 + CONFIG_OFFSET(WORKING_CONFIG))  // NORMAL(0)/CONSIST(1) - see AIRBRAKE_COMP_MODE
#define EE_COMPRESSOR2_FUNCTION      (0x52 + CONFIG_OFFSET(WORKING_CONFIG))  // "COMPRSR2" - routine/staggered compressor event
#define EE_AIRBRAKE_DISPLAY          (0x53 + CONFIG_OFFSET(WORKING_CONFIG))  // DUAL(0)/SINGLE(1) - see AIRBRAKE_DISPLAY

// SPEED model payload (group 2): the decoder-type-specific parameters, in one contiguous 16-byte
// block (15 used, 0x63 reserved). Which of these a given SPEED_TYPE actually uses, and in what menu
// order, is decided by the per-family descriptor in cst-speed.c (speedItemAt()); this is only their
// fixed storage. The EEPROM_LAYOUT_VERSION 2->3 migration in readConfig() relocated 0x54-0x60 here
// from their former scattered offsets (preserving every value for a layout-2 throttle); the 3->4
// migration inits 0x61/0x62 (ACCELADJ/DECELADJ) to 0.
#define EE_SPEED_MODEL_PAYLOAD       (0x54 + CONFIG_OFFSET(WORKING_CONFIG))  // block base
#define EE_MOMENTUM_BRAKE2_CV180     (0x54 + CONFIG_OFFSET(WORKING_CONFIG))  // CV180 mirror
#define EE_MOMENTUM_BRAKE3_CV181     (0x55 + CONFIG_OFFSET(WORKING_CONFIG))  // CV181 mirror
#define EE_MOMENTUM_START_DELAY      (0x56 + CONFIG_OFFSET(WORKING_CONFIG))  // CV167 mirror
#define EE_SPEED_HOLD_WATCH_FN       (0x57 + CONFIG_OFFSET(WORKING_CONFIG))  // Drive Hold watched DCC fn, 255=OFF (default F09)
#define EE_SPEED_STOP_WATCH_FN       (0x58 + CONFIG_OFFSET(WORKING_CONFIG))  // stop-trigger watched DCC fn, 255=OFF
#define EE_SPEED_OPLOAD              (0x59 + CONFIG_OFFSET(WORKING_CONFIG))  // CV103 mirror (Optional Load)
#define EE_SPEED_OPLOAD_FN           (0x5A + CONFIG_OFFSET(WORKING_CONFIG))  // OPLOAD watched DCC fn, 255=OFF
#define EE_SPEED_PRLOAD              (0x5B + CONFIG_OFFSET(WORKING_CONFIG))  // CV104 mirror (Primary Load)
#define EE_SPEED_PRLOAD_FN           (0x5C + CONFIG_OFFSET(WORKING_CONFIG))  // PRLOAD watched DCC fn, 255=OFF
#define EE_SPEED_ACCEL_PCT           (0x5D + CONFIG_OFFSET(WORKING_CONFIG))  // 0-255=0-100%, standing-start head-start
#define EE_SPEED_ACCEL_TARGET        (0x5E + CONFIG_OFFSET(WORKING_CONFIG))  // target ticks-to-1mph (0.1s/tick)
#define EE_SPEED_DECEL_PCT           (0x5F + CONFIG_OFFSET(WORKING_CONFIG))  // 0-255=0-100%, steady-state decel-lag strength
#define EE_SPEED_DECEL_THRESHOLD     (0x60 + CONFIG_OFFSET(WORKING_CONFIG))  // raw speed step below which decel-lag does not apply
#define EE_SPEED_ACCEL_ADJ           (0x61 + CONFIG_OFFSET(WORKING_CONFIG))  // CV23 mirror (Adjust Acceleration), signed, V5 only
#define EE_SPEED_DECEL_ADJ           (0x62 + CONFIG_OFFSET(WORKING_CONFIG))  // CV24 mirror (Adjust Deceleration), signed, V5 only
//      reserved                      0x63

// 0x63-0x7F: per-slot space not yet in use - 0x63 reserved for a future SPEED model param, the rest
// padding. The old SPEED holes at 0x2F, 0x3B, 0x3D-0x40 and 0x44-0x48 (vacated by the layout 2->3
// model-parameter move) are also free to reuse (0x2C/0x2D of that set are now the MENU/SEL button
// function slots).

// One-shot EEPROM layout migrations (cst-eeprom.c) - rewrites the EEPROM when newer firmware boots
// over an older EEPROM_LAYOUT_VERSION. Called once from readConfig() with the pre-stamp
// EE_LAYOUT_VERSION byte; a no-op once oldLayoutVersion == EEPROM_LAYOUT_VERSION. Covered by
// `make eepromtest` (src/cst-eeprom-test/). See CLAUDE.md "EEPROM layout".
void applyEepromMigrations(uint8_t oldLayoutVersion);

// Factory defaults for the per-profile SPEED / AIRBRAKE / STACK "model" bytes at configBase (raw
// in-slot offsets). resetConfig() in mrbw-cst.c calls this for the working config; it writes the
// loco / force-func / brake-pulse / optionBits / notch bytes itself and cst-functions.c owns the
// function bytes. Covered by `make eepromtest`.
void eepromResetProfileModel(uint16_t configBase);

#endif
