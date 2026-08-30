#ifndef _CST_EEPROM_H_
#define _CST_EEPROM_H_

#define EE_VERSION_MAJOR              0x0E
#define EE_VERSION_MINOR              0x0F

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
#define EE_PRESSURE_CONFIG            0x24
#define EE_ALERTER_TIMEOUT            0x25
//                                    0x26
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
#define EE_BRAKE_OFF_FUNCTION         (0x13 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_REV_SWAP_FUNCTION          (0x14 + CONFIG_OFFSET(WORKING_CONFIG))
//                                     0x15
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
//                                     0x2B
//                                     0x2C
//                                     0x2D
//                                     0x2E
//                                     0x2F

#define EE_COMPRESSOR_FUNCTION        (0x30 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_BRAKE_TEST_FUNCTION        (0x31 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_NEUTRAL_FUNCTION           (0x32 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_ALERTER_FUNCTION           (0x33 + CONFIG_OFFSET(WORKING_CONFIG))

// STACK brake mode's 5-STEP band->combo mapping, one byte per band 1-5 (band 0 is fixed to "none" and
// isn't stored). Each byte reuses the BRAKE_CONTROL/BK2_CONTROL/BK3_CONTROL bit values from mrbw-cst.c.
// The 3-STEP variant (see EE_STACK_BAND_COMBOS_3STEP below) is stored completely separately, so toggling
// between the two never cross-contaminates one variant's configured combos with the other's.
#define EE_STACK_BAND_COMBOS          (0x34 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_STACK_BAND_COMBOS           0x35
//      EE_STACK_BAND_COMBOS           0x36
//      EE_STACK_BAND_COMBOS           0x37
//      EE_STACK_BAND_COMBOS           0x38

// Scale-speed simulation config (src/cst-speed.c), raw 0-255 values. The 6 CV-mirror fields directly
// mirror the loco's ESU LokSound V5 "momentum" CVs (CV3/CV4/CV179/CV180/CV181/CV167 - momentum is the
// correct NMRA/ESU term for these specific CVs) and reuse the isolated single-byte gaps left by
// EE_BK2_FUNCTION/EE_BK3_FUNCTION above; the other 3 fields (display max/unit/e-stop watch) are
// unrelated to momentum and take the next free bytes after EE_STACK_BAND_COMBOS.
#define EE_MOMENTUM_ACCEL_CV3         (0x28 + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV3
#define EE_MOMENTUM_BRAKE1_CV179      (0x2B + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV179
#define EE_MOMENTUM_BRAKE2_CV180      (0x2C + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV180
#define EE_MOMENTUM_BRAKE3_CV181      (0x2D + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV181
#define EE_MOMENTUM_DECEL_CV4         (0x2E + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV4
#define EE_MOMENTUM_START_DELAY       (0x2F + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV167
#define EE_SPEED_MAX_MPH              (0x39 + CONFIG_OFFSET(WORKING_CONFIG))  // scale mph @ speed step 126
#define EE_SPEED_UNIT_KMH             (0x3A + CONFIG_OFFSET(WORKING_CONFIG))  // SPEED_UNIT_MPH/_KMH
#define EE_SPEED_STOP_WATCH_FN        (0x3B + CONFIG_OFFSET(WORKING_CONFIG))  // watched DCC fn 0-28, 255=OFF
// 0x3C free (was EE_SPEED_RAMP_PCT/RAMPUP, itself a reuse of the freed PADDING byte - RAMPUP removed,
// fully inert all session, see CLAUDE.md).
#define EE_SPEED_TYPE                 (0x3D + CONFIG_OFFSET(WORKING_CONFIG))  // SPEED_TYPE_V5DCC/_V4V5MULT
#define EE_SPEED_OPLOAD               (0x3E + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV103
#define EE_SPEED_PRLOAD               (0x3F + CONFIG_OFFSET(WORKING_CONFIG))  // mirrors decoder CV104
#define EE_SPEED_OPLOAD_FN            (0x40 + CONFIG_OFFSET(WORKING_CONFIG))  // watched DCC fn 0-28, 255=OFF
#define EE_SPEED_PRLOAD_FN            (0x41 + CONFIG_OFFSET(WORKING_CONFIG))  // watched DCC fn 0-28, 255=OFF

// STACK brake mode's 3-STEP band->combo mapping, one byte per band 1-3 (band 0 is fixed to "none" and
// isn't stored) - stored separately from EE_STACK_BAND_COMBOS (the 5-STEP variant), see that comment above.
#define EE_STACK_BAND_COMBOS_3STEP    (0x42 + CONFIG_OFFSET(WORKING_CONFIG))
//      EE_STACK_BAND_COMBOS_3STEP     0x43
//      EE_STACK_BAND_COMBOS_3STEP     0x44

// ESU Drive Hold: watched DCC function (0-28, 255=OFF) that freezes the speed simulation while active -
// see cst-speed.c's updateSpeed10Hz(). Defaults to F09 (SPEED_HOLD_WATCH_FN_DEFAULT), not OFF like the
// other watched-function fields, since Drive Hold should work out of the box.
#define EE_SPEED_HOLD_WATCH_FN        (0x45 + CONFIG_OFFSET(WORKING_CONFIG))

// 0x46 free (was EE_SPEED_WINDUP_PCT/WINDUP - removed after hardware testing found DECTHR/DECPCT/SSFLOOR,
// applied unconditionally to any deceleration, already covers what WINDUP was for, see CLAUDE.md).

// Steady-state deceleration lag: raw speed-step threshold (0-126) below which no lag applies, and
// 0-255=0-100% scale for the lag above it - approximates PID wind-up when decelerating from a genuine
// steady state (no prior acceleration). See cst-speed.c's updateSpeed10Hz().
#define EE_SPEED_DECEL_THRESHOLD      (0x47 + CONFIG_OFFSET(WORKING_CONFIG))
#define EE_SPEED_DECEL_PCT            (0x48 + CONFIG_OFFSET(WORKING_CONFIG))

// 0x49/0x4A free (were EE_SPEED_SS_FLOOR/EE_SPEED_SS_FLOORCUT - removed).

// 0-255=0-100%, standing-start head-start correction - see cst-speed.c's updateSpeed10Hz().
#define EE_SPEED_ACCEL_PCT            (0x4B + CONFIG_OFFSET(WORKING_CONFIG))

// Target time-to-1mph, in ticks (0.1s/tick) - reuses the byte freed by LIFTOFF's removal above. See
// cst-speed.c's solveRampR0() and cst-speed.h's SPEED_ACCEL_TARGET_DEFAULT for the full mechanism (a
// nonzero initial ramp slope solved to hit this target exactly) and why this is a separate knob from ACCPCT.
#define EE_SPEED_ACCEL_TARGET         (0x4C + CONFIG_OFFSET(WORKING_CONFIG))

// Horn2 (two-stage horn), per-profile DCC function assignment. hornThreshold2 (the calibration point
// itself) is global, not per-profile - see EE_HORN_THRESHOLD2 above, alongside EE_HORN_THRESHOLD.
#define EE_HORN2_FUNCTION             (0x4D + CONFIG_OFFSET(WORKING_CONFIG))

#endif
