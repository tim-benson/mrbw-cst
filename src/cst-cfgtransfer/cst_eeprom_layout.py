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
EE_MENU_VIS_1 = 0x17  # menu-visibility bits, low byte (SYSTEM menu HIDE toggles) - layout 5->6
EE_MENU_VIS_2 = 0x18  # menu-visibility bits, high byte
EE_TX_HOLDOFF = 0x1D
EE_TIME_SOURCE_ADDRESS = 0x1E
EE_BASE_ADDR = 0x1F
EE_HORN_THRESHOLD = 0x20
EE_BRAKE_THRESHOLD = 0x21
EE_BRAKE_LOW_THRESHOLD = 0x22
EE_BRAKE_HIGH_THRESHOLD = 0x23
# 0x24 free (was EE_PRESSURE_CONFIG, the original stock ISE gauge's global pump-rate byte - inert
# since AIRBRAKE superseded it, dropped from device.json entirely rather than kept as a byte-exact
# backup exception).
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
TX_HOLDOFF_MAX = 254   # one below the 0xFF erased-byte sentinel (firmware heals 0xFF -> default)

CONFIGBITS_LED_BLINK = 0
CONFIGBITS_MAIN_SCREEN_SPEED = 1  # bit clear = clock (default), set = scale speed
CONFIGBITS_AIRBRAKE = 2  # bit clear = AIRBRAKE off (default); set = drives air sound functions
CONFIGBITS_OPS_MODE = 3  # bit clear = OPS MODE off (default); set = long-press MENU on the base screen enters the OPS MODE screen
CONFIGBITS_REVERSER_LOCK = 4
CONFIGBITS_STRICT_SLEEP = 5

# menuVisBits bitfield (mrbw-cst.c MENUVISBITS_*) - a 16-bit value split across EE_MENU_VIS_1 (low
# byte, bits 0-7) / EE_MENU_VIS_2 (high byte, bit 8 used, bits 9-15 reserved for future menus). Bit
# SET = the menu is shown in the top-level MENU cycle (the default - a fresh/upgrading throttle shows
# every menu), bit CLEAR = hidden via the SYSTEM menu's HIDE toggles. SYSTEM_SCREEN has no bit here -
# it can never be hidden.
MENUVISBITS_FORCE_FUNC = 0
MENUVISBITS_CONFIG_FUNC = 1
MENUVISBITS_NOTCH = 2
MENUVISBITS_SPEED = 3
MENUVISBITS_AIRBRAKE = 4
MENUVISBITS_OPTIONS = 5
MENUVISBITS_COMM = 6
MENUVISBITS_PREFS = 7
MENUVISBITS_DIAGS = 8

# --- Per-slot fields (offsets relative to config_offset(n)) ---
EE_LOCO_ADDRESS = 0x00  # word
LOCO_ADDRESS_SHORT = 0x8000
LOCO_ADDRESS_SHORT_MAX = 127
LOCO_ADDRESS_LONG_MAX = 9999

# Function-assignment fields: (json_key, offset, attributes).
# attributes mirrors cst-functions.c's FunctionData.attributes bitfield:
#   FUNC_SPECIAL -> may be set to EMRG (FN_EMRG)
#   FUNC_MENU    -> may be set to AIRBRAKE (FN_AIRBRAKE)
#   FUNC_LOAD    -> may be set to LOAD (FN_LOAD)
#   FUNC_CLOCK   -> may be set to CLOCK (FN_CLOCK)
# (FUNC_LATCH/SOFTWARE_LATCH only gates what the on-device increment/decrement UI can reach; it doesn't
# restrict what byte value is valid, so it's not enforced by this tool's validation.)
FUNC_SPECIAL = 0x02
FUNC_MENU = 0x04
FUNC_LOAD = 0x08
FUNC_CLOCK = 0x10

# Listed in on-device CONFIG FUNC menu order - which is exactly the `Functions` enum in
# src/cst-functions.h, since advanceCurrentFunction() just does `currentFunction++` (the functions[]
# array in cst-functions.c uses designated initialisers, so its source-listing order is irrelevant).
# NOT EEPROM-offset order - decode/encode key off each tuple's explicit offset, never list position,
# so this order is purely for how the JSON file reads. Must match the key list in
# src/cst-cfgtransfer/README.md.
FUNCTION_FIELDS = [
    # (json_key,         offset, attributes)
    ("HORN",             0x02, 0),
    ("HORN2",            0x49, 0),
    ("BELL",             0x03, 0),
    ("BRAKE",            0x04, 0),
    ("BRAKE2",           0x29, 0),
    ("BRAKE3",           0x2A, 0),
    ("AUX",              0x05, FUNC_SPECIAL),
    ("ENGINE_ON",        0x06, 0),
    ("ENGINE_OFF",       0x07, 0),
    ("THR_UNLOCK",       0x12, 0),
    ("REV_SWAP",         0x14, 0),
    ("NEUTRAL",          0x32, 0),
    ("COMPRESSOR",       0x30, 0),
    ("COMPRESSOR2",      0x52, 0),
    ("BRAKE_SET",        0x31, 0),
    ("BRAKE_REL",        0x13, 0),
    ("ALERTER",          0x33, FUNC_SPECIAL),
    ("EMERGENCY",        0x15, 0),
    ("FRONT_HEADLIGHT",  0x0A, 0),
    ("FRONT_DITCH",      0x0B, 0),
    ("FRONT_DIM1",       0x08, 0),
    ("FRONT_DIM2",       0x09, 0),
    ("REAR_HEADLIGHT",   0x0E, 0),
    ("REAR_DITCH",       0x0F, 0),
    ("REAR_DIM1",        0x0C, 0),
    ("REAR_DIM2",        0x0D, 0),
    ("UP_BUTTON",        0x10, FUNC_SPECIAL | FUNC_MENU | FUNC_LOAD | FUNC_CLOCK),
    ("DOWN_BUTTON",      0x11, FUNC_SPECIAL | FUNC_MENU | FUNC_LOAD | FUNC_CLOCK),
    ("MENU_BUTTON",      0x2C, FUNC_SPECIAL | FUNC_MENU | FUNC_LOAD | FUNC_CLOCK),  # OPS MODE - was a freed SPEED BRK2 scatter slot
    ("SEL_BUTTON",       0x2D, FUNC_SPECIAL | FUNC_MENU | FUNC_LOAD | FUNC_CLOCK),  # OPS MODE - was a freed SPEED BRK3 scatter slot
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

# SPEED group 1 - decoder-type-agnostic (same meaning for every TYPE, original scattered addresses).
EE_MOMENTUM_ACCEL_CV3 = 0x28
EE_MOMENTUM_BRAKE1_CV179 = 0x2B
EE_MOMENTUM_DECEL_CV4 = 0x2E

EE_STACK_BAND_COMBOS = 0x34  # 5 bytes, bands 1-5 (band 0 fixed/not stored)
EE_SPEED_MAX_MPH = 0x39
EE_SPEED_UNIT_KMH = 0x3A
EE_SPEED_TYPE = 0x3C
EE_STACK_BAND_COMBOS_3STEP = 0x41  # 3 bytes, bands 1-3

# SPEED group 2 - decoder-type-specific model parameters, contiguous EE_SPEED_MODEL_PAYLOAD block
# (0x54-0x63, 15 used + 0x63 reserved). 0x54-0x60 moved here from scattered holes in 0x2C-0x48 by the
# EEPROM_LAYOUT_VERSION 2->3 migration; ACCELADJ/DECELADJ (0x61/0x62) were added by the 3->4 migration.
# Which of these a TYPE uses is a firmware-descriptor concern (cst-speed.c); the codec just mirrors
# every slot.
EE_SPEED_MODEL_PAYLOAD = 0x54
EE_MOMENTUM_BRAKE2_CV180 = 0x54
EE_MOMENTUM_BRAKE3_CV181 = 0x55
EE_MOMENTUM_START_DELAY = 0x56
EE_SPEED_HOLD_WATCH_FN = 0x57
EE_SPEED_STOP_WATCH_FN = 0x58
EE_SPEED_OPLOAD = 0x59
EE_SPEED_OPLOAD_FN = 0x5A
EE_SPEED_PRLOAD = 0x5B
EE_SPEED_PRLOAD_FN = 0x5C
EE_SPEED_ACCEL_PCT = 0x5D
EE_SPEED_ACCEL_TARGET = 0x5E
EE_SPEED_DECEL_PCT = 0x5F
EE_SPEED_DECEL_THRESHOLD = 0x60
EE_SPEED_ACCEL_ADJ = 0x61   # CV23 mirror (signed, V5 only) - layout 3->4
EE_SPEED_DECEL_ADJ = 0x62   # CV24 mirror (signed, V5 only) - layout 3->4

# AIRBRAKE per-profile air-brake model config (src/cst-pressure.c / AIRBRAKE_CONFIG_SCREEN), raw
# 0-255 bytes, all self-healing via readByteOrDefault(). 0x49 is EE_HORN2_FUNCTION and 0x52 is
# EE_COMPRESSOR2_FUNCTION (both in FUNCTION_FIELDS - wedged in this block because the 0x30 function
# region is fully packed). AIRBRAKE occupies 0x4A-0x53, the SPEED model payload 0x54-0x63 (0x63
# reserved); the SPEED group-1 holes at 0x2C/0x2D/0x2F, 0x3B, 0x3D-0x40 and 0x44-0x48 are free.
# 0x64-0x7F is per-slot padding.
EE_AIRBRAKE_CHARGED = 0x4A
EE_AIRBRAKE_MR_CUTIN = 0x4B
EE_AIRBRAKE_MR_CUTOUT = 0x4C
EE_AIRBRAKE_CHARGE_RATE = 0x4D
EE_AIRBRAKE_LEAK_RATE = 0x4E
EE_AIRBRAKE_PUMP_RATE = 0x4F
EE_AIRBRAKE_MR_LOAD = 0x50
EE_AIRBRAKE_COMP_MODE = 0x51  # NORMAL(0)/CONSIST(1) - gates the COMPRSR/COMPRSR2 split
EE_AIRBRAKE_DISPLAY = 0x53    # DUAL(0)/SINGLE(1) - which AIRBRAKE_SCREEN rendering

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
OPTIONBITS_DITCH_TYPE = 7  # 0 = Additive (F.DITCH stacks on F.HEAD, the default), 1 = Exclusive (F.DITCH replaces F.HEAD)
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

DITCH_TYPE_ADDITIVE = 0
DITCH_TYPE_EXCLUSIVE = 1
DITCH_TYPE_TO_NAME = {DITCH_TYPE_ADDITIVE: "ADDITIVE", DITCH_TYPE_EXCLUSIVE: "EXCLUSIVE"}
DITCH_TYPE_FROM_NAME = {v: k for k, v in DITCH_TYPE_TO_NAME.items()}

# controls-byte bits (mrbw-cst.c), reused directly as the STACK band-combo byte encoding
BRAKE_CONTROL = 0x08
BRAKE_REL_CONTROL = 0x10
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
SPEED_TYPE_V5MULT = 1   # was SPEED_TYPE_V4V5MULT - same byte value, same behaviour
SPEED_TYPE_V4 = 2
SPEED_TYPE_TO_NAME = {SPEED_TYPE_V5DCC: "V5DCC", SPEED_TYPE_V5MULT: "V5MULT", SPEED_TYPE_V4: "V4"}
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
    "ACCELADJ": 0,
    "DECELADJ": 0,
}

# json_key -> EEPROM offset for every SPEED field (agnostic + every family's model params). All are
# self-healing (readByteOrDefault) fields per readConfig() - a raw 0xFF decodes as "UNSET".
SPEED_FIELD_OFFSET = {
    "TYPE": EE_SPEED_TYPE,
    "MAXSPEED": EE_SPEED_MAX_MPH,
    "UNIT": EE_SPEED_UNIT_KMH,
    "ACCEL": EE_MOMENTUM_ACCEL_CV3,
    "DECEL": EE_MOMENTUM_DECEL_CV4,
    "BRK1": EE_MOMENTUM_BRAKE1_CV179,
    "BRK2": EE_MOMENTUM_BRAKE2_CV180,
    "BRK3": EE_MOMENTUM_BRAKE3_CV181,
    "DELAY": EE_MOMENTUM_START_DELAY,
    "HOLDFN": EE_SPEED_HOLD_WATCH_FN,
    "STOPFN": EE_SPEED_STOP_WATCH_FN,
    "OPLOAD": EE_SPEED_OPLOAD,
    "OPLOADFN": EE_SPEED_OPLOAD_FN,
    "PRLOAD": EE_SPEED_PRLOAD,
    "PRLOADFN": EE_SPEED_PRLOAD_FN,
    "ACCPCT": EE_SPEED_ACCEL_PCT,
    "ACCTGT": EE_SPEED_ACCEL_TARGET,
    "DECPCT": EE_SPEED_DECEL_PCT,
    "DECTHR": EE_SPEED_DECEL_THRESHOLD,
    "ACCELADJ": EE_SPEED_ACCEL_ADJ,
    "DECELADJ": EE_SPEED_DECEL_ADJ,
}

# The 5 type-agnostic SPEED CFG fields, in on-device menu order (TYPE first). Present for every family.
# BRK1 (CV179) is present for every family too, but it is grouped with BRK2/BRK3 in each family's
# model list rather than here.
SPEED_AGNOSTIC_FIELDS = ["TYPE", "MAXSPEED", "UNIT", "ACCEL", "DECEL"]

# Per-family model fields. Mirrors the descriptors in cst-speed.c: V5DCC and V5MULT carry the identical
# 16; V4 drops ACCELADJ/DECELADJ (no CV23/CV24), BRK2/BRK3 (no CV180/CV181) and the load CVs (no
# CV103/CV104), keeping BRK1. ACCELADJ/DECELADJ lead this list for the "family has it" membership test,
# but speed_fields_for_type() splices them into the menu right after the ACCEL/DECEL they adjust; the
# rest follow the agnostic 5 in this order.
_ESU_V5_MODEL_FIELDS = ["ACCELADJ", "DECELADJ",
                        "BRK1", "BRK2", "BRK3", "DELAY", "HOLDFN", "STOPFN",
                        "OPLOAD", "OPLOADFN", "PRLOAD", "PRLOADFN",
                        "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"]
_ESU_V4_MODEL_FIELDS = ["BRK1", "DELAY", "HOLDFN", "STOPFN", "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"]
SPEED_MODEL_FIELDS = {
    "V5DCC": _ESU_V5_MODEL_FIELDS,
    "V5MULT": _ESU_V5_MODEL_FIELDS,
    "V4": _ESU_V4_MODEL_FIELDS,
}

# Value written to a model slot the family does not use - mirrors speedItemInert() in cst-speed.c, so
# a V4 EEPROM image matches what the firmware produces after speedResetModel()/speedApplyTypeInert().
SPEED_MODEL_INERT = {"ACCELADJ": 0, "DECELADJ": 0, "BRK2": 0, "BRK3": 0, "OPLOAD": 128, "PRLOAD": 128,
                     "OPLOADFN": WATCHED_FN_OFF, "PRLOADFN": WATCHED_FN_OFF}

# Fields shown/edited as a signed value (-127..127 - the full ESU CV23/CV24 magnitude range); the
# stored byte carries the decoder's sign-bit encoding (bit 7 = subtract, bits 0-6 = magnitude), so
# -127 is byte 0xFF. The firmware reads these two bytes raw (0xFF is a real -127, never "unset"), and
# so does _decode_speed. Decoded/encoded in slot_codec.py.
SPEED_SIGNED_FIELDS = {"ACCELADJ", "DECELADJ"}
SPEED_ADJ_MAG_MAX = 127

# ACCEL / DECEL are genuine 0-255 CVs (a decoder's literal CV3 / CV4 can be 255): the firmware reads
# them raw, so a stored 0xFF is a real 255, not "UNSET". Every other plain-numeric SPEED field still
# self-heals from 0xFF, so its max is 254. See _decode_speed / _encode_speed in slot_codec.py.
SPEED_FULL_RANGE_FIELDS = {"ACCEL", "DECEL"}


def speed_fields_for_type(type_name):
    """The json keys a `speed` object carries for the given decoder family, in on-device menu order:
    the 5 agnostic fields, with ACCELADJ/DECELADJ spliced in right after the ACCEL/DECEL they adjust
    for a family that has them, then the rest of that family's model params. Unknown type_name falls
    back to the V5DCC set."""
    model = SPEED_MODEL_FIELDS.get(type_name, _ESU_V5_MODEL_FIELDS)
    out = []
    for f in SPEED_AGNOSTIC_FIELDS:
        out.append(f)
        if f == "ACCEL" and "ACCELADJ" in model:
            out.append("ACCELADJ")
        elif f == "DECEL" and "DECELADJ" in model:
            out.append("DECELADJ")
    out += [f for f in model if f not in ("ACCELADJ", "DECELADJ")]
    return out


# Fields whose value is a raw 0-255 number (as opposed to UNIT, TYPE, or a *FN watched-function field
# with its own string encoding).
SPEED_PLAIN_NUMERIC_FIELDS = {"ACCEL", "DECEL", "BRK1", "BRK2", "BRK3", "DELAY", "MAXSPEED",
                               "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"}
SPEED_WATCHED_FN_FIELDS = {"HOLDFN", "STOPFN", "OPLOADFN", "PRLOADFN"}

# --- AIRBRAKE (AIRBRAKE CFG) per-profile fields ---
# Named defaults mirror cst-pressure.h's AIRBRAKE_*_DEFAULT constants (readByteOrDefault fallbacks).
# FULLSVC/VENT/EMRG/APPLY/DRV_LOAD no longer exist here - FULLSVC is derived from BP_CHARGE at
# runtime (never stored), VENT/EMRG are hardcoded constants, APPLY/DRV_LOAD were removed outright -
# see cst-pressure.c/cst-eeprom.h. DISPLAY (which AIRBRAKE_SCREEN rendering) and COMP_MODE (the
# COMPRSR/COMPRSR2 split toggle) are string enums, not plain numbers - see the *_TO_NAME/FROM_NAME
# maps below.
AIRBRAKE_FIELD_DEFAULTS = {
    "BP_CHARGE": 90,          # PSI, range AIRBRAKE_BP_CHARGE_MIN-MAX
    "MR_LOAD": 35,            # % of pipe recharge drawn from the reservoir, range 0-AIRBRAKE_MR_LOAD_MAX
    "MR_LOW": 130,            # PSI, reservoir governor cut-in
    "MR_HIGH": 140,           # PSI, reservoir governor cut-out
    "RECHARGE": 180,          # PSI/min, brake-pipe recharge rate (initial rate of the taper)
    "LEAK_RATE": 5,           # PSI/min, reservoir base leak
    "PUMP_RATE": 30,          # PSI/min, compressor fill rate
    "DISPLAY": "DUAL",        # "DUAL" (BP:/MR: glyph view) or "SINGLE" (analogue BP dial)
    "COMP_MODE": "NORMAL",    # "NORMAL" or "CONSIST"
}

AIRBRAKE_BP_CHARGE_MIN = 70   # on-device editing range
AIRBRAKE_BP_CHARGE_MAX = 110
AIRBRAKE_MR_LOAD_MAX = 100    # on-device editing range is 0-100

AIRBRAKE_COMP_MODE_NORMAL = 0
AIRBRAKE_COMP_MODE_CONSIST = 1
AIRBRAKE_COMP_MODE_TO_NAME = {AIRBRAKE_COMP_MODE_NORMAL: "NORMAL", AIRBRAKE_COMP_MODE_CONSIST: "CONSIST"}
AIRBRAKE_COMP_MODE_FROM_NAME = {v: k for k, v in AIRBRAKE_COMP_MODE_TO_NAME.items()}

AIRBRAKE_DISPLAY_DUAL = 0
AIRBRAKE_DISPLAY_SINGLE = 1
AIRBRAKE_DISPLAY_TO_NAME = {AIRBRAKE_DISPLAY_DUAL: "DUAL", AIRBRAKE_DISPLAY_SINGLE: "SINGLE"}
AIRBRAKE_DISPLAY_FROM_NAME = {v: k for k, v in AIRBRAKE_DISPLAY_TO_NAME.items()}

# (json_key, eeprom offset) for each AIRBRAKE CFG field, in on-device AIRBRAKE_CONFIG_SCREEN menu
# order. All are plain 0-254-or-"UNSET" numbers except DISPLAY and COMP_MODE (string enums).
AIRBRAKE_FIELDS = [
    ("BP_CHARGE", EE_AIRBRAKE_CHARGED),
    ("MR_LOAD",   EE_AIRBRAKE_MR_LOAD),
    ("MR_LOW",    EE_AIRBRAKE_MR_CUTIN),
    ("MR_HIGH",   EE_AIRBRAKE_MR_CUTOUT),
    ("RECHARGE",  EE_AIRBRAKE_CHARGE_RATE),
    ("LEAK_RATE", EE_AIRBRAKE_LEAK_RATE),
    ("PUMP_RATE", EE_AIRBRAKE_PUMP_RATE),
    ("DISPLAY",   EE_AIRBRAKE_DISPLAY),
    ("COMP_MODE", EE_AIRBRAKE_COMP_MODE),
]
# Note: STOPFN/OPLOADFN/PRLOADFN's readByteOrDefault() default IS WATCHED_FN_OFF (255/0xFF) itself, so a
# raw 0xFF and an explicit "OFF" are the same byte and decode identically as "UNSET" either way - this is
# a real firmware property (the self-heal is a no-op for these three), not a codec bug. HOLDFN's default
# is F09 (non-OFF), so its "UNSET" is a genuinely distinct, meaningful state.

# --- FunctionValues encoding (cst-functions.h) ---
FN_OFF = 0x80
FN_EMRG = 0x81
FN_LOAD = 0x82  # UP_BUTTON/DOWN_BUTTON/MENU_BUTTON/SEL_BUTTON only - see FUNC_LOAD above
FN_CLOCK = 0x83  # UP_BUTTON/DOWN_BUTTON/MENU_BUTTON/SEL_BUTTON only - see FUNC_CLOCK above
FN_AIRBRAKE = 0xC0  # was FN_BRKTEST - value unchanged
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
    value_to_name[FN_LOAD] = "LOAD"
    value_to_name[FN_CLOCK] = "CLOCK"
    value_to_name[FN_AIRBRAKE] = "AIRBRAKE"
    name_to_value["OFF"] = FN_OFF
    name_to_value["EMRG"] = FN_EMRG
    name_to_value["LOAD"] = FN_LOAD
    name_to_value["CLOCK"] = FN_CLOCK
    name_to_value["AIRBRAKE"] = FN_AIRBRAKE
    return value_to_name, name_to_value


FUNCTION_VALUE_TO_NAME, FUNCTION_NAME_TO_VALUE = _build_function_value_maps()
