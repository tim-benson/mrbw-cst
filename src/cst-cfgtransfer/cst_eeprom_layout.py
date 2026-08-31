# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""EEPROM layout constants for the MRBW-CST ("ProtoThrottle") throttle.

This is a HAND-MAINTAINED MIRROR of the layout defined in the C firmware -
src/cst-eeprom.h, src/cst-functions.h, src/cst-speed.h, and the bitfield
#defines in src/mrbw-cst.c (OPTIONBITS_*, CONFIGBITS_*, BRAKE_CONTROL /
BK2_CONTROL / BK3_CONTROL). It is NOT generated from those files - the C
headers only give byte offsets, not the decode semantics (bitfields, enums,
multi-byte arrays) that live in mrbw-cst.c's control flow.

Whenever the firmware's EEPROM layout changes, the offset/enum/decode tables
below must be updated by hand to match. SUPPORTED_LAYOUT_VERSION is the one
exception - it is parsed straight from src/cst-eeprom.h at import (see
_read_firmware_layout_version below), so it can never drift out of sync with
the firmware's EEPROM_LAYOUT_VERSION. See CLAUDE.md's "PC tooling" section
("Maintenance checklist") for the full checklist.
"""

import os
import re

EEPROM_SIZE = 4096

# --- Layout-version guard ---
# The firmware header src/cst-eeprom.h is the single source of truth for the CNF format version. This
# value is read from it at import rather than hand-copied, so the two integers cannot disagree. The
# cst-cfgtransfer / cst-cfgnetwork tools are repo-resident by design (stdlib-only, run from a git
# checkout), so requiring the header one directory up is fine - a missing/unparseable header raises a
# loud RuntimeError here, never a silent wrong default.
EE_LAYOUT_VERSION = 0x26


def _read_firmware_layout_version():
    header = os.path.join(os.path.dirname(os.path.abspath(__file__)), os.pardir, "cst-eeprom.h")
    try:
        with open(header, encoding="utf-8") as f:
            contents = f.read()
    except OSError as e:
        raise RuntimeError(
            "cannot read %s to determine the CNF format version - the cst-cfgtransfer / cst-cfgnetwork "
            "tools must be run from a full git checkout (%s)" % (header, e)
        )
    match = re.search(r"^#define\s+EEPROM_LAYOUT_VERSION\s+(\d+)", contents, re.MULTILINE)
    if not match:
        raise RuntimeError("no '#define EEPROM_LAYOUT_VERSION' found in %s" % header)
    return int(match.group(1))


SUPPORTED_LAYOUT_VERSION = _read_firmware_layout_version()

# --- Slot geometry (src/cst-eeprom.h) ---
CONFIG_START = 0x80
CONFIG_SIZE = 0x80
MAX_CONFIGS = 20
WORKING_CONFIG = 31


def config_offset(cfg_num: int) -> int:
    """Absolute EEPROM address of the start of numbered slot cfg_num (1..20, or 31 for WORKING_CONFIG)."""
    return (cfg_num - 1) * CONFIG_SIZE + CONFIG_START


# --- Global block (device-level settings, not per-loco) ---
EE_MRBUS_DEVICE_ADDR = 0x00
EE_MRBUS_DEVICE_UPDATE_H = 0x02
EE_MRBUS_DEVICE_UPDATE_L = 0x03
EE_VERSION_MAJOR = 0x0E  # informational only (git-tag derived); never written by this tool
EE_VERSION_MINOR = 0x0F
EE_DEVICE_SLEEP_TIMEOUT = 0x11
EE_DEAD_RECKONING_TIME = 0x12
EE_CONFIGBITS = 0x13
EE_BATTERY_OKAY = 0x14
EE_BATTERY_WARN = 0x15
EE_BATTERY_CRITICAL = 0x16
EE_TX_HOLDOFF = 0x1D
EE_TIME_SOURCE_ADDRESS = 0x1E
EE_BASE_ADDR = 0x1F
EE_HORN_THRESHOLD = 0x20
EE_BRAKE_THRESHOLD = 0x21
EE_BRAKE_LOW_THRESHOLD = 0x22
EE_BRAKE_HIGH_THRESHOLD = 0x23
EE_PRESSURE_CONFIG = 0x24
EE_ALERTER_TIMEOUT = 0x25
EE_HORN_THRESHOLD2 = 0x27

MRBUS_DEV_ADDR_MIN = 0x30
MRBUS_DEV_ADDR_MAX = 0x49
MRBUS_BASE_ADDR_MIN = 0xD0
MRBUS_BASE_ADDR_MAX = 0xEF
UPDATE_DECISECS_MIN = 10
UPDATE_DECISECS_MAX = 100
SLEEP_TMR_MIN = 1
SLEEP_TMR_MAX = 99
ALERTER_TMR_MIN = 0
ALERTER_TMR_MAX = 60
TX_HOLDOFF_MIN = 10

CONFIGBITS_LED_BLINK = 0
CONFIGBITS_MAIN_SCREEN_SPEED = 1  # bit clear = clock (default), set = scale speed
CONFIGBITS_REVERSER_LOCK = 4
CONFIGBITS_STRICT_SLEEP = 5

# --- Per-slot fields (offsets relative to config_offset(n)) ---
EE_LOCO_ADDRESS = 0x00  # word
LOCO_ADDRESS_SHORT = 0x8000
LOCO_ADDRESS_SHORT_MAX = 127
LOCO_ADDRESS_LONG_MAX = 9999

# Function-assignment fields: (json_key, offset, attributes).
# attributes mirrors cst-functions.c's FunctionData.attributes bitfield:
#   FUNC_SPECIAL -> may be set to EMRG (FN_EMRG)
#   FUNC_MENU    -> may be set to BRKTEST (FN_BRKTEST)
# (FUNC_LATCH/SOFTWARE_LATCH only gates what the on-device increment/decrement UI can reach; it doesn't
# restrict what byte value is valid, so it's not enforced by this tool's validation.)
FUNC_SPECIAL = 0x02
FUNC_MENU = 0x04

# Listed in on-device CONFIG FUNC menu order - which is exactly the `Functions` enum in
# src/cst-functions.h, since advanceCurrentFunction() just does `currentFunction++` (the functions[]
# array in cst-functions.c uses designated initialisers, so its source-listing order is irrelevant).
# NOT EEPROM-offset order - decode/encode key off each tuple's explicit offset, never list position,
# so this order is purely for how the JSON file reads. Must match the key list in
# src/cst-cfgtransfer/README.md.
FUNCTION_FIELDS = [
    # (json_key,         offset, attributes)
    ("HORN",             0x02, 0),
    ("HORN2",            0x4D, 0),
    ("BELL",             0x03, 0),
    ("BRAKE",            0x04, 0),
    ("BRAKE2",           0x29, 0),
    ("BRAKE3",           0x2A, 0),
    ("BRAKE_OFF",        0x13, 0),
    ("AUX",              0x05, FUNC_SPECIAL),
    ("ENGINE_ON",        0x06, 0),
    ("ENGINE_OFF",       0x07, 0),
    ("THR_UNLOCK",       0x12, 0),
    ("REV_SWAP",         0x14, 0),
    ("NEUTRAL",          0x32, 0),
    ("ALERTER",          0x33, FUNC_SPECIAL),
    ("COMPRESSOR",       0x30, 0),
    ("BRAKE_TEST",       0x31, 0),
    ("FRONT_HEADLIGHT",  0x0A, 0),
    ("FRONT_DITCH",      0x0B, 0),
    ("FRONT_DIM1",       0x08, 0),
    ("FRONT_DIM2",       0x09, 0),
    ("REAR_HEADLIGHT",   0x0E, 0),
    ("REAR_DITCH",       0x0F, 0),
    ("REAR_DIM1",        0x0C, 0),
    ("REAR_DIM2",        0x0D, 0),
    ("UP_BUTTON",        0x10, FUNC_SPECIAL | FUNC_MENU),
    ("DOWN_BUTTON",      0x11, FUNC_SPECIAL | FUNC_MENU),
]

EE_BRAKE_PULSE_WIDTH = 0x16
BRAKE_PULSE_WIDTH_MIN = 2
BRAKE_PULSE_WIDTH_MAX = 10

EE_OPTIONBITS = 0x17
EE_FORCE_FUNC_ON = 0x18  # dword bitmask, bit N = DCC function N forced on
EE_FORCE_FUNC_OFF = 0x1C  # dword bitmask, bit N = DCC function N forced off
FORCE_FUNC_MAX_NUM = 28  # never assignable above F28 via the on-device menu

EE_NOTCH_SPEEDSTEP = 0x20  # 8 bytes
NOTCH_SPEEDSTEP_COUNT = 8
NOTCH_SPEEDSTEP_MIN = 1
NOTCH_SPEEDSTEP_MAX = 126

EE_MOMENTUM_ACCEL_CV3 = 0x28
EE_MOMENTUM_BRAKE1_CV179 = 0x2B
EE_MOMENTUM_BRAKE2_CV180 = 0x2C
EE_MOMENTUM_BRAKE3_CV181 = 0x2D
EE_MOMENTUM_DECEL_CV4 = 0x2E
EE_MOMENTUM_START_DELAY = 0x2F

EE_STACK_BAND_COMBOS = 0x34  # 5 bytes, bands 1-5 (band 0 fixed/not stored)
EE_SPEED_MAX_MPH = 0x39
EE_SPEED_UNIT_KMH = 0x3A
EE_SPEED_STOP_WATCH_FN = 0x3B
EE_SPEED_TYPE = 0x3D
EE_SPEED_OPLOAD = 0x3E
EE_SPEED_PRLOAD = 0x3F
EE_SPEED_OPLOAD_FN = 0x40
EE_SPEED_PRLOAD_FN = 0x41
EE_STACK_BAND_COMBOS_3STEP = 0x42  # 3 bytes, bands 1-3
EE_SPEED_HOLD_WATCH_FN = 0x45
EE_SPEED_DECEL_THRESHOLD = 0x47
EE_SPEED_DECEL_PCT = 0x48
EE_SPEED_ACCEL_PCT = 0x4B
EE_SPEED_ACCEL_TARGET = 0x4C

STACK_BAND_COUNT_3STEP = 4  # bands 0-3 (3 editable)
STACK_BAND_COUNT_5STEP = 6  # bands 0-5 (5 editable)

# optionBits bitfield (mrbw-cst.c OPTIONBITS_*)
OPTIONBITS_ESTOP_ON_BRAKE = 0
OPTIONBITS_REVERSER_SWAP = 1
OPTIONBITS_VARIABLE_BRAKE = 2
OPTIONBITS_BRK_TYPE_LSB = 3
OPTIONBITS_BRK_TYPE_MASK = 0x03 << OPTIONBITS_BRK_TYPE_LSB
OPTIONBITS_STACK_5STEP = 5
OPTIONBITS_HORN_TYPE = 6  # 0 = Additive (Horn2 stacks on Horn1, the default), 1 = Exclusive (Horn2 replaces Horn1)
OPTIONBITS_DEFAULT = 1 << OPTIONBITS_ESTOP_ON_BRAKE

BRK_TYPE_PULSE = 0
BRK_TYPE_STEP = 1
BRK_TYPE_STACK = 2
BRK_TYPE_TO_NAME = {BRK_TYPE_PULSE: "PULSE", BRK_TYPE_STEP: "STEP", BRK_TYPE_STACK: "STACK"}
BRK_TYPE_FROM_NAME = {v: k for k, v in BRK_TYPE_TO_NAME.items()}

HORN_TYPE_ADDITIVE = 0
HORN_TYPE_EXCLUSIVE = 1
HORN_TYPE_TO_NAME = {HORN_TYPE_ADDITIVE: "ADDITIVE", HORN_TYPE_EXCLUSIVE: "EXCLUSIVE"}
HORN_TYPE_FROM_NAME = {v: k for k, v in HORN_TYPE_TO_NAME.items()}

# controls-byte bits (mrbw-cst.c), reused directly as the STACK band-combo byte encoding
BRAKE_CONTROL = 0x08
BRAKE_OFF_CONTROL = 0x10
BK2_CONTROL = 0x20
BK3_CONTROL = 0x40
STACK_COMBO_MASK = BRAKE_CONTROL | BK2_CONTROL | BK3_CONTROL

STACK_5STEP_DEFAULTS = {
    1: BK3_CONTROL,
    2: BRAKE_CONTROL,
    3: BK2_CONTROL | BK3_CONTROL,
    4: BRAKE_CONTROL | BK2_CONTROL,
    5: BRAKE_CONTROL | BK3_CONTROL,
}
STACK_3STEP_DEFAULTS = {
    1: BK2_CONTROL,
    2: BK3_CONTROL,
    3: BRAKE_CONTROL,
}

# Speed/momentum field enums (cst-speed.h)
SPEED_UNIT_MPH = 0
SPEED_UNIT_KMH = 1
SPEED_UNIT_TO_NAME = {SPEED_UNIT_MPH: "MPH", SPEED_UNIT_KMH: "KMH"}
SPEED_UNIT_FROM_NAME = {v: k for k, v in SPEED_UNIT_TO_NAME.items()}

SPEED_TYPE_V5DCC = 0
SPEED_TYPE_V4V5MULT = 1
SPEED_TYPE_TO_NAME = {SPEED_TYPE_V5DCC: "V5DCC", SPEED_TYPE_V4V5MULT: "V4V5MULT"}
SPEED_TYPE_FROM_NAME = {v: k for k, v in SPEED_TYPE_TO_NAME.items()}

WATCHED_FN_OFF = 255  # 0-28 = a DCC function number, 255 = OFF/disabled

# Named defaults (cst-speed.h *_DEFAULT constants) - used for the "UNSET" (raw 0xFF) sentinel and by
# tests/test_slot_codec.py, mirroring readConfig()'s readByteOrDefault() fallback values exactly.
SPEED_FIELD_DEFAULTS = {
    "ACCEL": 60,
    "DECEL": 230,
    "BRK1": 130,
    "BRK2": 70,
    "BRK3": 100,
    "DELAY": 13,
    "MAXSPEED": 50,
    "UNIT": SPEED_UNIT_MPH,
    "HOLDFN": 9,
    "STOPFN": WATCHED_FN_OFF,
    "OPLOAD": 128,
    "OPLOADFN": WATCHED_FN_OFF,
    "PRLOAD": 128,
    "PRLOADFN": WATCHED_FN_OFF,
    "TYPE": SPEED_TYPE_V5DCC,
    "ACCPCT": 8,
    "ACCTGT": 5,
    "DECPCT": 22,
    "DECTHR": 11,
}

# (json_key, eeprom offset) for each SPEED field, in on-device menu order. All 19 are self-healing
# (readByteOrDefault) fields per readConfig() - a raw 0xFF decodes as "UNSET".
SPEED_FIELDS = [
    ("ACCEL", EE_MOMENTUM_ACCEL_CV3),
    ("DECEL", EE_MOMENTUM_DECEL_CV4),
    ("BRK1", EE_MOMENTUM_BRAKE1_CV179),
    ("BRK2", EE_MOMENTUM_BRAKE2_CV180),
    ("BRK3", EE_MOMENTUM_BRAKE3_CV181),
    ("DELAY", EE_MOMENTUM_START_DELAY),
    ("MAXSPEED", EE_SPEED_MAX_MPH),
    ("UNIT", EE_SPEED_UNIT_KMH),
    ("HOLDFN", EE_SPEED_HOLD_WATCH_FN),
    ("STOPFN", EE_SPEED_STOP_WATCH_FN),
    ("OPLOAD", EE_SPEED_OPLOAD),
    ("OPLOADFN", EE_SPEED_OPLOAD_FN),
    ("PRLOAD", EE_SPEED_PRLOAD),
    ("PRLOADFN", EE_SPEED_PRLOAD_FN),
    ("TYPE", EE_SPEED_TYPE),
    ("ACCPCT", EE_SPEED_ACCEL_PCT),
    ("ACCTGT", EE_SPEED_ACCEL_TARGET),
    ("DECPCT", EE_SPEED_DECEL_PCT),
    ("DECTHR", EE_SPEED_DECEL_THRESHOLD),
]
# Fields among the above whose value is a raw 0-255 number (as opposed to an enum/watched-fn field with
# its own string encoding) - i.e. everything except UNIT, TYPE, and the *FN watched-function fields.
SPEED_PLAIN_NUMERIC_FIELDS = {"ACCEL", "DECEL", "BRK1", "BRK2", "BRK3", "DELAY", "MAXSPEED",
                               "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"}
SPEED_WATCHED_FN_FIELDS = {"HOLDFN", "STOPFN", "OPLOADFN", "PRLOADFN"}
# Note: STOPFN/OPLOADFN/PRLOADFN's readByteOrDefault() default IS WATCHED_FN_OFF (255/0xFF) itself, so a
# raw 0xFF and an explicit "OFF" are the same byte and decode identically as "UNSET" either way - this is
# a real firmware property (the self-heal is a no-op for these three), not a codec bug. HOLDFN's default
# is F09 (non-OFF), so its "UNSET" is a genuinely distinct, meaningful state.

# --- FunctionValues encoding (cst-functions.h) ---
FN_OFF = 0x80
FN_EMRG = 0x81
FN_BRKTEST = 0xC0
FN_MAX_NUM = 28


def _build_function_value_maps():
    value_to_name = {}
    name_to_value = {}
    for n in range(FN_MAX_NUM + 1):
        mom_name = "F%02d_MOM" % n
        lat_name = "F%02d_LAT" % n
        value_to_name[n] = mom_name
        value_to_name[0x40 + n] = lat_name
        name_to_value[mom_name] = n
        name_to_value[lat_name] = 0x40 + n
    value_to_name[FN_OFF] = "OFF"
    value_to_name[FN_EMRG] = "EMRG"
    value_to_name[FN_BRKTEST] = "BRKTEST"
    name_to_value["OFF"] = FN_OFF
    name_to_value["EMRG"] = FN_EMRG
    name_to_value["BRKTEST"] = FN_BRKTEST
    return value_to_name, name_to_value


FUNCTION_VALUE_TO_NAME, FUNCTION_NAME_TO_VALUE = _build_function_value_maps()
