# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Pure decode/encode/validate logic for one 128-byte EEPROM profile slot, or the 128-byte global block.

No hardware/subprocess dependency here - everything operates on `bytes`/`bytearray` already in memory,
which is what makes this module independently unit-testable without an ISP programmer attached
(see tests/test_slot_codec.py). See cst_eeprom_layout.py for the offset/constant ground truth this
module decodes against.
"""

import struct

import cst_eeprom_layout as layout

UNSET = "UNSET"

# 2: horn_type added; every section's keys in on-device-menu order; the JSON is grouped one object per
# config menu - slot: force_functions{on,off}, options (the OPTIONS menu, was `brake`; its meta-field
# `unset` was `options_unset`); device.json: system / comm / prefs (config_bits nested) / calibration.
# Informational only - import does not gate on it, and both encoders still accept the older flat/renamed
# shapes (see _normalize_legacy_slot / _flatten_global). v3 adds the `airbrake` slot section (AIRBRAKE
# CFG - BP_CHARGE/MR_LOAD/MR_LOW/MR_HIGH/RECHARGE/LEAK_RATE/PUMP_RATE/DISPLAY/COMP_MODE) - squashed from
# several uncommitted intermediate shapes (including a since-abandoned `brakesim` key/field-name scheme)
# into the one bump a v2 backup actually needs; a v2 backup lacking the section restores fine with
# --import-old. (DISPLAY was added to the airbrake section after v3 without a bump - the feature has
# only ever run on one test throttle, nothing in the wild; an older export whose airbrake object lacks
# DISPLAY needs --import-old on a plain import.)
# 4: the `speed` object is now decoder-family-shaped - it carries the 6 type-agnostic fields plus only
# the model fields the TYPE uses (V4 drops BRK2/BRK3 and the load CVs). A pre-4 flat backup (all 19
# fields) still imports without --import-old for a V5DCC/V5MULT TYPE (identical field set); a hand-set
# V4 TYPE on a flat backup needs --import-old to ignore the inapplicable fields.
SLOT_SCHEMA_VERSION = 4


class SlotValidationError(ValueError):
    """Raised by encode_slot()/encode_global() with every problem found, not just the first."""

    def __init__(self, errors):
        self.errors = list(errors)
        super().__init__("; ".join(self.errors))


# --- FunctionValues (functions.*) ---

def _decode_function_value(raw):
    # Any raw byte outside the known FunctionValues table (e.g. 0xFF on a never-visited slot) is
    # surfaced as a diagnostic passthrough rather than crashing the export - see module docstring.
    return layout.FUNCTION_VALUE_TO_NAME.get(raw, "RAW:0x%02X" % raw)


def _encode_function_value(name, errors, field_label):
    if isinstance(name, str) and name.startswith("RAW:0x"):
        try:
            return int(name[4:], 16) & 0xFF
        except ValueError:
            pass
    if name in layout.FUNCTION_NAME_TO_VALUE:
        return layout.FUNCTION_NAME_TO_VALUE[name]
    errors.append("%s: unrecognized function value %r" % (field_label, name))
    return 0


def _decode_functions(raw):
    return {key: _decode_function_value(raw[offset]) for key, offset, _attrs in layout.FUNCTION_FIELDS}


def _encode_functions(d, errors, allow_missing=False):
    """allow_missing=True (the `--import-old` path) defaults an absent function key to raw 0xFF - the
    same byte a genuinely never-written function reads as (confirmed by
    test_all_ff_slot_decodes_to_unset_not_crash), so this is the most faithful default available, not an
    invented one. A key that IS present but holds an unrecognized value is still a hard error either way -
    only genuine absence is relaxed."""
    out = {}
    keys_seen = set()
    for key, offset, attrs in layout.FUNCTION_FIELDS:
        keys_seen.add(key)
        if key not in d:
            if allow_missing:
                out[offset] = 0xFF
            else:
                errors.append("functions.%s: missing" % key)
            continue
        value = _encode_function_value(d[key], errors, "functions.%s" % key)
        if value == layout.FN_EMRG and not (attrs & layout.FUNC_SPECIAL):
            errors.append("functions.%s: EMRG is not valid here (only AUX, ALERTER, UP_BUTTON, "
                           "DOWN_BUTTON support it)" % key)
        if value == layout.FN_AIRBRAKE and not (attrs & layout.FUNC_MENU):
            errors.append("functions.%s: AIRBRAKE is not valid here (only UP_BUTTON, DOWN_BUTTON "
                           "support it)" % key)
        out[offset] = value
    unknown = set(d.keys()) - keys_seen
    for key in unknown:
        errors.append("functions.%s: unknown field" % key)
    return out


# --- loco_address ---

def _decode_loco_address(word):
    if word & layout.LOCO_ADDRESS_SHORT:
        return {"address": word & ~layout.LOCO_ADDRESS_SHORT, "type": "short"}
    return {"address": word, "type": "long"}


def _encode_loco_address(d, errors):
    if not isinstance(d, dict) or "address" not in d or "type" not in d:
        errors.append("loco_address: must be an object with 'address' and 'type'")
        return 0
    addr = d["address"]
    kind = d["type"]
    if kind == "short":
        if not (isinstance(addr, int) and 0 <= addr <= layout.LOCO_ADDRESS_SHORT_MAX):
            errors.append("loco_address: short address must be 0-%d" % layout.LOCO_ADDRESS_SHORT_MAX)
            return 0
        return addr | layout.LOCO_ADDRESS_SHORT
    elif kind == "long":
        if not (isinstance(addr, int) and 0 <= addr <= layout.LOCO_ADDRESS_LONG_MAX):
            errors.append("loco_address: long address must be 0-%d" % layout.LOCO_ADDRESS_LONG_MAX)
            return 0
        return addr
    else:
        errors.append("loco_address: type must be 'short' or 'long', got %r" % kind)
        return 0


# --- STACK band combos ---

def _decode_stack_combo(raw):
    if raw == 0xFF:
        return UNSET
    masked = raw & layout.STACK_COMBO_MASK
    c1 = "1" if masked & layout.BRAKE_CONTROL else "-"
    c2 = "2" if masked & layout.BK2_CONTROL else "-"
    c3 = "3" if masked & layout.BK3_CONTROL else "-"
    return "BRAKE" + c1 + c2 + c3


def _encode_stack_combo(value, errors, field_label):
    if value == UNSET:
        return 0xFF
    if isinstance(value, str) and len(value) == 8 and value.startswith("BRAKE"):
        c1, c2, c3 = value[5], value[6], value[7]
        val = 0
        ok = True
        if c1 == "1":
            val |= layout.BRAKE_CONTROL
        elif c1 != "-":
            ok = False
        if c2 == "2":
            val |= layout.BK2_CONTROL
        elif c2 != "-":
            ok = False
        if c3 == "3":
            val |= layout.BK3_CONTROL
        elif c3 != "-":
            ok = False
        if ok:
            return val
    errors.append("%s: invalid STACK combo %r (expected \"BRAKE---\"..\"BRAKE123\" or \"UNSET\")"
                  % (field_label, value))
    return 0


def _decode_stack_combo_list(raw_bytes, defaults_by_band):
    return [_decode_stack_combo(raw_bytes[i]) for i in range(len(defaults_by_band))]


def _encode_stack_combo_list(values, expected_len, errors, field_label, allow_missing=False):
    if values is None and allow_missing:
        return [0xFF] * expected_len  # UNSET for every band - the whole list was absent, not malformed
    if not isinstance(values, list) or len(values) != expected_len:
        errors.append("%s: must be a list of %d entries" % (field_label, expected_len))
        return [0] * expected_len
    return [_encode_stack_combo(v, errors, "%s[%d]" % (field_label, i)) for i, v in enumerate(values)]


# --- options (OPTIONS menu / optionBits) ---

def _decode_options(raw_option_bits, pulse_width, stack5_bytes, stack3_bytes):
    # Key order below follows the on-device OPTIONS menu: VAR BRK, BRK TYPE, BRK RATE | (STEPS + STEP1..n),
    # BRK ESTP, REV SWAP, HORNTYPE. `unset` is a meta-flag ("the option byte has never been written")
    # and stays first.
    if raw_option_bits == 0xFF:
        unset = True
        variable_brake = False
        brk_type_name = layout.BRK_TYPE_TO_NAME[layout.BRK_TYPE_PULSE]
        stack_5step = False
        estop_on_full_brake = True
        reverser_swap = False
        horn_type = layout.HORN_TYPE_TO_NAME[layout.HORN_TYPE_ADDITIVE]
    else:
        unset = False
        variable_brake = bool(raw_option_bits & (1 << layout.OPTIONBITS_VARIABLE_BRAKE))
        brk_type = (raw_option_bits & layout.OPTIONBITS_BRK_TYPE_MASK) >> layout.OPTIONBITS_BRK_TYPE_LSB
        brk_type_name = layout.BRK_TYPE_TO_NAME.get(brk_type, "RAW:%d" % brk_type)
        stack_5step = bool(raw_option_bits & (1 << layout.OPTIONBITS_STACK_5STEP))
        estop_on_full_brake = bool(raw_option_bits & (1 << layout.OPTIONBITS_ESTOP_ON_BRAKE))
        reverser_swap = bool(raw_option_bits & (1 << layout.OPTIONBITS_REVERSER_SWAP))
        horn_type = layout.HORN_TYPE_TO_NAME[(raw_option_bits >> layout.OPTIONBITS_HORN_TYPE) & 1]
    return {
        "unset": unset,
        "variable_brake": variable_brake,
        "type": brk_type_name,
        "pulse_width": pulse_width,
        "stack_5step": stack_5step,
        "stack_band_combos_5step": _decode_stack_combo_list(stack5_bytes, layout.STACK_5STEP_DEFAULTS),
        "stack_band_combos_3step": _decode_stack_combo_list(stack3_bytes, layout.STACK_3STEP_DEFAULTS),
        "estop_on_full_brake": estop_on_full_brake,
        "reverser_swap": reverser_swap,
        "horn_type": horn_type,
    }


def _encode_options(d, errors, allow_missing=False):
    """Returns (option_bits, pulse_width, stack5_bytes, stack3_bytes). allow_missing=True relaxes only
    genuinely ABSENT sub-keys (defaulted the same way each field already falls back on an invalid value -
    "BRAKE---"/no-combo for the stack lists, BRAKE_PULSE_WIDTH_MIN for pulse_width, etc. - reusing
    existing fallback values rather than inventing new ones); a key that IS present but holds a bad value
    is still a hard error regardless of this flag."""
    required = {"unset", "type", "estop_on_full_brake", "reverser_swap", "variable_brake",
                "stack_5step", "pulse_width", "stack_band_combos_5step", "stack_band_combos_3step",
                "horn_type"}
    if not isinstance(d, dict):
        errors.append("options: must be an object")
        return 0xFF, layout.BRAKE_PULSE_WIDTH_MIN, [0] * 5, [0] * 3
    unknown = set(d.keys()) - required
    for key in unknown:
        errors.append("options.%s: unknown field" % key)
    if not allow_missing:
        missing = required - set(d.keys())
        for key in missing:
            errors.append("options.%s: missing" % key)

    if d.get("unset"):
        option_bits = 0xFF
    else:
        brk_type_name = d.get("type")
        if brk_type_name not in layout.BRK_TYPE_FROM_NAME:
            if "type" in d or not allow_missing:
                errors.append("options.type: must be one of %s" % sorted(layout.BRK_TYPE_FROM_NAME))
            brk_type = layout.BRK_TYPE_PULSE
        else:
            brk_type = layout.BRK_TYPE_FROM_NAME[brk_type_name]
        option_bits = 0
        for key, bit in (("estop_on_full_brake", layout.OPTIONBITS_ESTOP_ON_BRAKE),
                          ("reverser_swap", layout.OPTIONBITS_REVERSER_SWAP),
                          ("variable_brake", layout.OPTIONBITS_VARIABLE_BRAKE),
                          ("stack_5step", layout.OPTIONBITS_STACK_5STEP)):
            val = d.get(key)
            if not isinstance(val, bool):
                if key in d or not allow_missing:
                    errors.append("options.%s: must be true/false" % key)
                val = False
            if val:
                option_bits |= (1 << bit)
        horn_type_name = d.get("horn_type")
        if horn_type_name not in layout.HORN_TYPE_FROM_NAME:
            if "horn_type" in d or not allow_missing:
                errors.append("options.horn_type: must be one of %s" % sorted(layout.HORN_TYPE_FROM_NAME))
        elif layout.HORN_TYPE_FROM_NAME[horn_type_name] == layout.HORN_TYPE_EXCLUSIVE:
            option_bits |= (1 << layout.OPTIONBITS_HORN_TYPE)
        option_bits |= (brk_type << layout.OPTIONBITS_BRK_TYPE_LSB)

    pulse_width = d.get("pulse_width")
    if not (isinstance(pulse_width, int) and
            layout.BRAKE_PULSE_WIDTH_MIN <= pulse_width <= layout.BRAKE_PULSE_WIDTH_MAX):
        if "pulse_width" in d or not allow_missing:
            errors.append("options.pulse_width: must be %d-%d" %
                           (layout.BRAKE_PULSE_WIDTH_MIN, layout.BRAKE_PULSE_WIDTH_MAX))
        pulse_width = layout.BRAKE_PULSE_WIDTH_MIN

    stack5 = _encode_stack_combo_list(d.get("stack_band_combos_5step"), 5, errors,
                                       "options.stack_band_combos_5step",
                                       allow_missing=(allow_missing and "stack_band_combos_5step" not in d))
    stack3 = _encode_stack_combo_list(d.get("stack_band_combos_3step"), 3, errors,
                                       "options.stack_band_combos_3step",
                                       allow_missing=(allow_missing and "stack_band_combos_3step" not in d))
    return option_bits, pulse_width, stack5, stack3


# --- speed ---

def _decode_speed(raw):
    # The TYPE byte picks the decoder family, which decides which model fields the object carries
    # (V4 drops BRK2/BRK3 and the load CVs). An unset or unrecognised TYPE falls back to the full V5
    # field set, matching speedType()'s clamp-to-default in the firmware.
    type_name = layout.SPEED_TYPE_TO_NAME.get(raw[layout.EE_SPEED_TYPE])
    out = {}
    for key in layout.speed_fields_for_type(type_name):
        val = raw[layout.SPEED_FIELD_OFFSET[key]]
        if val == 0xFF:
            out[key] = UNSET
        elif key == "UNIT":
            out[key] = layout.SPEED_UNIT_TO_NAME.get(val, "RAW:%d" % val)
        elif key == "TYPE":
            out[key] = layout.SPEED_TYPE_TO_NAME.get(val, "RAW:%d" % val)
        elif key in layout.SPEED_WATCHED_FN_FIELDS:
            out[key] = "OFF" if val == layout.WATCHED_FN_OFF else "F%02d" % val
        else:
            out[key] = val
    return out


def _encode_watched_fn(value, errors, field_label):
    if value == "OFF":
        return layout.WATCHED_FN_OFF
    if isinstance(value, str) and len(value) == 3 and value.startswith("F"):
        try:
            n = int(value[1:])
            if 0 <= n <= layout.FN_MAX_NUM:
                return n
        except ValueError:
            pass
    errors.append("%s: must be \"OFF\" or \"F00\"..\"F%02d\"" % (field_label, layout.FN_MAX_NUM))
    return layout.WATCHED_FN_OFF


def _encode_speed(d, errors, allow_missing=False):
    """allow_missing=True defaults an absent speed field to 0xFF/"UNSET" - the existing sentinel meaning
    "let the firmware's own readByteOrDefault() self-heal this on next load" (already the fallback value
    used below on any invalid field regardless of this flag). allow_missing also relaxes the "not a
    field of this TYPE" check, so a pre-split flat backup (all 19 fields, TYPE hand-set to V4)
    round-trips with --import-old - the inapplicable fields are ignored and the slots forced inert."""
    if not isinstance(d, dict):
        errors.append("speed: must be an object")
        d = {}

    # Resolve the decoder family from TYPE - it decides which model fields apply.
    type_raw = d.get("TYPE")
    if type_raw in layout.SPEED_TYPE_FROM_NAME:
        type_name = type_raw
    elif type_raw in (UNSET, None):
        type_name = None                       # validate against the full V5 field set
    else:
        errors.append("speed.TYPE: must be one of %s or \"UNSET\"" % sorted(layout.SPEED_TYPE_FROM_NAME))
        type_name = None

    expected = layout.speed_fields_for_type(type_name)
    expected_set = set(expected)
    for key in set(d.keys()) - expected_set:
        if key not in layout.SPEED_FIELD_OFFSET:
            errors.append("speed.%s: unknown field" % key)
        elif not allow_missing:
            errors.append("speed.%s: not a field of TYPE %s" % (key, type_raw))

    out = {}
    for key in expected:
        offset = layout.SPEED_FIELD_OFFSET[key]
        if key not in d:
            if not allow_missing:
                errors.append("speed.%s: missing" % key)
            out[offset] = 0xFF
            continue
        val = d[key]
        label = "speed.%s" % key
        if val == UNSET:
            out[offset] = 0xFF
        elif key == "UNIT":
            if val not in layout.SPEED_UNIT_FROM_NAME:
                errors.append("%s: must be one of %s" % (label, sorted(layout.SPEED_UNIT_FROM_NAME)))
                out[offset] = 0
            else:
                out[offset] = layout.SPEED_UNIT_FROM_NAME[val]
        elif key == "TYPE":
            out[offset] = layout.SPEED_TYPE_FROM_NAME.get(val, 0xFF)  # UNSET/invalid already flagged above
        elif key in layout.SPEED_WATCHED_FN_FIELDS:
            out[offset] = _encode_watched_fn(val, errors, label)
        else:
            if not (isinstance(val, int) and not isinstance(val, bool) and 0 <= val <= 254):
                # 254 not 255: 255/0xFF on a plain numeric field means UNSET (handled above), so a
                # literal 255 has no representable meaning here.
                errors.append("%s: must be an integer 0-254, or \"UNSET\"" % label)
                out[offset] = 0
            else:
                out[offset] = val

    # Model slots this family does not use -> inert (mirrors speedItemInert() in cst-speed.c), so a V4
    # image is byte-identical to what the firmware writes after speedResetModel().
    for key, inert in layout.SPEED_MODEL_INERT.items():
        if key not in expected_set:
            out[layout.SPEED_FIELD_OFFSET[key]] = inert
    return out


# --- airbrake (AIRBRAKE CFG menu) ---

def _decode_airbrake(raw):
    out = {}
    for key, offset in layout.AIRBRAKE_FIELDS:
        val = raw[offset]
        if val == 0xFF:
            out[key] = UNSET
        elif key == "COMP_MODE":
            out[key] = layout.AIRBRAKE_COMP_MODE_TO_NAME.get(val, "RAW:%d" % val)
        elif key == "DISPLAY":
            out[key] = layout.AIRBRAKE_DISPLAY_TO_NAME.get(val, "RAW:%d" % val)
        else:
            out[key] = val
    return out


def _encode_airbrake(d, errors, allow_missing=False):
    """AIRBRAKE CFG fields are plain 0-254 numbers or "UNSET" (0xFF - self-healed by the firmware's
    readByteOrDefault on next load), except DISPLAY (the string "DUAL"/"SINGLE", or "UNSET") and
    COMP_MODE (the string "NORMAL"/"CONSIST", or "UNSET").
    BP_CHARGE and MR_LOAD are additionally range-checked to match the on-device UP/DOWN ceiling/floor
    (the on-device UI cannot produce a byte outside these ranges once saved, so an import outside them
    could only come from a hand-edited file). allow_missing defaults an absent key to UNSET; see
    _encode_speed."""
    if not isinstance(d, dict):
        errors.append("airbrake: must be an object")
        d = {}
    known_keys = {key for key, _ in layout.AIRBRAKE_FIELDS}
    for key in set(d.keys()) - known_keys:
        errors.append("airbrake.%s: unknown field" % key)

    out = {}
    for key, offset in layout.AIRBRAKE_FIELDS:
        if key not in d:
            if not allow_missing:
                errors.append("airbrake.%s: missing" % key)
            out[offset] = 0xFF
            continue
        val = d[key]
        label = "airbrake.%s" % key
        if val == UNSET:
            out[offset] = 0xFF
        elif key == "COMP_MODE":
            if val not in layout.AIRBRAKE_COMP_MODE_FROM_NAME:
                errors.append("%s: must be one of %s, or \"UNSET\"" %
                               (label, sorted(layout.AIRBRAKE_COMP_MODE_FROM_NAME)))
                out[offset] = 0
            else:
                out[offset] = layout.AIRBRAKE_COMP_MODE_FROM_NAME[val]
        elif key == "DISPLAY":
            if val not in layout.AIRBRAKE_DISPLAY_FROM_NAME:
                errors.append("%s: must be one of %s, or \"UNSET\"" %
                               (label, sorted(layout.AIRBRAKE_DISPLAY_FROM_NAME)))
                out[offset] = 0
            else:
                out[offset] = layout.AIRBRAKE_DISPLAY_FROM_NAME[val]
        elif key == "BP_CHARGE":
            if not (isinstance(val, int) and not isinstance(val, bool) and
                    layout.AIRBRAKE_BP_CHARGE_MIN <= val <= layout.AIRBRAKE_BP_CHARGE_MAX):
                errors.append("%s: must be an integer %d-%d, or \"UNSET\"" %
                               (label, layout.AIRBRAKE_BP_CHARGE_MIN, layout.AIRBRAKE_BP_CHARGE_MAX))
                out[offset] = layout.AIRBRAKE_BP_CHARGE_MIN
            else:
                out[offset] = val
        elif key == "MR_LOAD":
            if not (isinstance(val, int) and not isinstance(val, bool) and
                    0 <= val <= layout.AIRBRAKE_MR_LOAD_MAX):
                errors.append("%s: must be an integer 0-%d, or \"UNSET\"" % (label, layout.AIRBRAKE_MR_LOAD_MAX))
                out[offset] = 0
            else:
                out[offset] = val
        elif isinstance(val, int) and not isinstance(val, bool) and 0 <= val <= 254:
            out[offset] = val
        else:
            errors.append("%s: must be an integer 0-254, or \"UNSET\"" % label)
            out[offset] = 0
    return out


# --- force_functions (FORCE FUNC menu) ---

def _decode_force_mask(raw_dword):
    return sorted(n for n in range(32) if raw_dword & (1 << n))


def _normalize_legacy_slot(d):
    """Accept a pre-schema-2 slot dict on import and present it in the current shape, so encode_slot
    only ever handles one shape. Two renames are bridged:
      - the flat `force_function_on` / `force_function_off` pair -> `force_functions: {on, off}`
      - the `brake` OPTIONS-menu object -> `options`, and its `options_unset` meta-field -> `unset`.
    A dict already in the current shape (or with none of these keys) passes through unchanged; the
    caller's dict is never mutated."""
    if not isinstance(d, dict):
        return d
    changed = False
    out = dict(d)
    if "force_functions" not in out and ("force_function_on" in out or "force_function_off" in out):
        ff = {}
        if "force_function_on" in out:
            ff["on"] = out.pop("force_function_on")
        if "force_function_off" in out:
            ff["off"] = out.pop("force_function_off")
        out["force_functions"] = ff
        changed = True
    opts = out.get("options", out.get("brake"))
    if "options" not in out and "brake" in out:
        out["options"] = out.pop("brake")
        opts = out["options"]
        changed = True
    if isinstance(opts, dict) and "options_unset" in opts and "unset" not in opts:
        opts = dict(opts)
        opts["unset"] = opts.pop("options_unset")
        out["options"] = opts
        changed = True
    return out if changed else d


def _encode_force_mask(values, errors, field_label):
    if not isinstance(values, list):
        errors.append("%s: must be a list of DCC function numbers" % field_label)
        return 0
    mask = 0
    for n in values:
        if not (isinstance(n, int) and not isinstance(n, bool) and 0 <= n <= layout.FORCE_FUNC_MAX_NUM):
            errors.append("%s: %r is not a valid DCC function number (0-%d)"
                          % (field_label, n, layout.FORCE_FUNC_MAX_NUM))
            continue
        mask |= (1 << n)
    return mask


# --- notch_speedstep ---

def _decode_notch(raw_bytes):
    return list(raw_bytes[:layout.NOTCH_SPEEDSTEP_COUNT])


def _encode_notch(values, errors):
    if not isinstance(values, list) or len(values) != layout.NOTCH_SPEEDSTEP_COUNT:
        errors.append("notch_speedstep: must be a list of %d entries" % layout.NOTCH_SPEEDSTEP_COUNT)
        return [layout.NOTCH_SPEEDSTEP_MIN] * layout.NOTCH_SPEEDSTEP_COUNT
    out = []
    for i, v in enumerate(values):
        if not (isinstance(v, int) and not isinstance(v, bool) and
                layout.NOTCH_SPEEDSTEP_MIN <= v <= layout.NOTCH_SPEEDSTEP_MAX):
            errors.append("notch_speedstep[%d]: must be %d-%d"
                          % (i, layout.NOTCH_SPEEDSTEP_MIN, layout.NOTCH_SPEEDSTEP_MAX))
            out.append(layout.NOTCH_SPEEDSTEP_MIN)
        else:
            out.append(v)
    return out


_OPTIONS_FIELDS = ("unset", "variable_brake", "type", "pulse_width", "stack_5step",
                    "stack_band_combos_5step", "stack_band_combos_3step", "estop_on_full_brake",
                    "reverser_swap", "horn_type")


def describe_missing_fields(d):
    """Returns a list of human-readable notices for the specific sub-fields encode_slot(d,
    allow_missing=True) will default, given the CURRENT contents of `d` (computed before encoding, so a
    caller can show this to the user before the confirmation prompt - see cst_cfgtransfer.py/
    cst_cfgnetwork.py's `--import-old` flag). Only looks at categories already present as an object in
    `d` - a category missing entirely isn't something allow_missing relaxes, so it's not reported here
    (encode_slot() will raise on it regardless of allow_missing)."""
    notices = []
    d = _normalize_legacy_slot(d)
    functions = d.get("functions")
    if isinstance(functions, dict):
        for key, _offset, _attrs in layout.FUNCTION_FIELDS:
            if key not in functions:
                notices.append("functions.%s: not in file, defaulting to RAW:0xFF" % key)
    speed = d.get("speed")
    if isinstance(speed, dict):
        type_raw = speed.get("TYPE")
        type_name = type_raw if type_raw in layout.SPEED_TYPE_FROM_NAME else None
        for key in layout.speed_fields_for_type(type_name):
            if key not in speed:
                notices.append("speed.%s: not in file, defaulting to UNSET" % key)
    airbrake = d.get("airbrake")
    if isinstance(airbrake, dict):
        for key, _offset in layout.AIRBRAKE_FIELDS:
            if key not in airbrake:
                notices.append("airbrake.%s: not in file, defaulting to UNSET" % key)
    elif "airbrake" not in d:
        notices.append("airbrake: whole section not in file (pre-v3 backup), every field defaulting to UNSET")
    options = d.get("options")
    if isinstance(options, dict):
        for key in _OPTIONS_FIELDS:
            if key not in options:
                notices.append("options.%s: not in file, defaulting" % key)
    ff = d.get("force_functions")
    if isinstance(ff, dict):
        for key in ("on", "off"):
            if key not in ff:
                notices.append("force_functions.%s: not in file, defaulting to [] (nothing forced)" % key)
    return notices


# --- top-level slot decode/encode ---

SLOT_TOP_LEVEL_KEYS = {"schema_version", "source", "loco_address", "force_functions", "functions",
                        "notch_speedstep", "speed", "airbrake", "options"}


def decode_slot(raw_128_bytes, source):
    """raw_128_bytes: a 128-byte bytes-like object for one slot. source: the dict to embed as "source"."""
    if len(raw_128_bytes) != layout.CONFIG_SIZE:
        raise ValueError("expected %d bytes, got %d" % (layout.CONFIG_SIZE, len(raw_128_bytes)))
    raw = raw_128_bytes
    loco_word = struct.unpack_from("<H", raw, layout.EE_LOCO_ADDRESS)[0]
    # Top-level key order follows the top-level menu cycle: LOCO -> FORCE FUNC -> CONFIG FUNC ->
    # NOTCH -> SPEED CFG -> AIRBRAKE CFG -> OPTIONS (schema_version/source are file metadata and lead).
    # One object per menu - FORCE FUNC is `force_functions` (distinct from CONFIG FUNC's `functions`),
    # and the OPTIONS menu is `options` (its object holds `reverser_swap`/`horn_type` too, not just
    # brake).
    return {
        "schema_version": SLOT_SCHEMA_VERSION,
        "source": source,
        "loco_address": _decode_loco_address(loco_word),
        "force_functions": {
            "on": _decode_force_mask(struct.unpack_from("<I", raw, layout.EE_FORCE_FUNC_ON)[0]),
            "off": _decode_force_mask(struct.unpack_from("<I", raw, layout.EE_FORCE_FUNC_OFF)[0]),
        },
        "functions": _decode_functions(raw),
        "notch_speedstep": _decode_notch(raw[layout.EE_NOTCH_SPEEDSTEP:layout.EE_NOTCH_SPEEDSTEP + 8]),
        "speed": _decode_speed(raw),
        "airbrake": _decode_airbrake(raw),
        "options": _decode_options(
            raw[layout.EE_OPTIONBITS],
            raw[layout.EE_BRAKE_PULSE_WIDTH],
            raw[layout.EE_STACK_BAND_COMBOS:layout.EE_STACK_BAND_COMBOS + 5],
            raw[layout.EE_STACK_BAND_COMBOS_3STEP:layout.EE_STACK_BAND_COMBOS_3STEP + 3],
        ),
    }


def encode_slot(d, base=None, allow_missing=False):
    """dict (as produced by decode_slot, or hand-edited) -> a 128-byte bytearray for one slot.

    `base`, if given, must be the slot's CURRENT 128 raw bytes on the device - fields this schema
    doesn't model (the 0x54-0x7F per-slot padding range, which nothing in firmware reads; 0x00-0x53 is
    fully packed - see cst_eeprom_layout.py) are preserved from it rather than zero-filled. Pass the
    real on-device bytes here before writing to hardware; omitting it (e.g. for offline validation/
    tests, where no "current" bytes exist) zero-fills unmodeled bytes instead.

    `allow_missing=True` (the `--import-old` CLI flag) is for restoring a JSON backup taken under an
    OLDER codec than this one, after a layout-version bump added new fields - a field ABSENT from a
    "functions"/"speed"/"airbrake"/"options" object present in `d` is defaulted rather than raising (see
    _encode_functions()/_encode_speed()/_encode_airbrake()/_encode_options() for each field's specific
    default). This does NOT relax SLOT_TOP_LEVEL_KEYS: if "functions"/"options"/"speed"/etc. is missing
    ENTIRELY, that's still
    a hard error regardless of this flag - those categories have all existed since long before any field
    was added to them, so a whole missing category means a genuinely bad file, not merely "predates a
    newer field."

    The older flat/renamed slot shapes (`force_function_on`/`off`, `brake`/`options_unset`) are
    normalised up-front by _normalize_legacy_slot() and remain importable.

    Raises SlotValidationError (with every problem found, not just the first) on any invalid input.
    """
    errors = []
    if not isinstance(d, dict):
        raise SlotValidationError(["slot: must be a JSON object"])
    d = _normalize_legacy_slot(d)
    unknown = set(d.keys()) - SLOT_TOP_LEVEL_KEYS
    for key in unknown:
        errors.append("%s: unknown top-level field" % key)
    missing = SLOT_TOP_LEVEL_KEYS - {"schema_version", "source"} - set(d.keys())
    if allow_missing:
        # `airbrake` is a whole section added by the EEPROM_LAYOUT_VERSION 1->2 bump, so a backup
        # taken under the older codec legitimately lacks it - default it to all-UNSET rather than raise.
        missing = missing - {"airbrake"}
    for key in missing:
        errors.append("%s: missing" % key)

    if base is not None and len(base) == layout.CONFIG_SIZE:
        out = bytearray(base)
    else:
        out = bytearray(layout.CONFIG_SIZE)

    if "loco_address" in d:
        loco_word = _encode_loco_address(d["loco_address"], errors)
        struct.pack_into("<H", out, layout.EE_LOCO_ADDRESS, loco_word)

    if "functions" in d:
        for offset, value in _encode_functions(d["functions"], errors, allow_missing).items():
            out[offset] = value

    if "options" in d:
        option_bits, pulse_width, stack5, stack3 = _encode_options(d["options"], errors, allow_missing)
        out[layout.EE_OPTIONBITS] = option_bits
        out[layout.EE_BRAKE_PULSE_WIDTH] = pulse_width
        out[layout.EE_STACK_BAND_COMBOS:layout.EE_STACK_BAND_COMBOS + 5] = bytes(stack5)
        out[layout.EE_STACK_BAND_COMBOS_3STEP:layout.EE_STACK_BAND_COMBOS_3STEP + 3] = bytes(stack3)

    if "force_functions" in d:
        ff = d["force_functions"]
        if not isinstance(ff, dict):
            errors.append("force_functions: must be an object with 'on' and 'off' lists")
        else:
            for key in set(ff) - {"on", "off"}:
                errors.append("force_functions.%s: unknown field" % key)
            for key, offset in (("on", layout.EE_FORCE_FUNC_ON), ("off", layout.EE_FORCE_FUNC_OFF)):
                if key in ff:
                    mask = _encode_force_mask(ff[key], errors, "force_functions.%s" % key)
                    struct.pack_into("<I", out, offset, mask)
                elif not allow_missing:
                    errors.append("force_functions.%s: missing" % key)

    if "notch_speedstep" in d:
        out[layout.EE_NOTCH_SPEEDSTEP:layout.EE_NOTCH_SPEEDSTEP + 8] = bytes(_encode_notch(
            d["notch_speedstep"], errors))

    if "speed" in d:
        for offset, value in _encode_speed(d["speed"], errors, allow_missing).items():
            out[offset] = value

    if "airbrake" in d or allow_missing:
        for offset, value in _encode_airbrake(d.get("airbrake", {}), errors, allow_missing).items():
            out[offset] = value

    if errors:
        raise SlotValidationError(errors)
    return out


# --- global block ---

# device.json groups the persisted device settings one object per config menu (see decode_global):
# SYSTEM -> COMM -> PREFS -> THRESHOLD CAL, top-level-menu-cycle order. encode_global() flattens both
# that shape and the older flat shape through _flatten_global() before doing anything else.
GLOBAL_CATEGORY_KEYS = ("system", "comm", "prefs", "calibration")

# Every individual device field name (+ config_bits). encode_global()'s unknown/missing check runs
# against this AFTER _flatten_global(), so it's the flat set regardless of which shape came in.
GLOBAL_FIELD_KEYS = {
    "schema_version", "source", "mrbus_device_address", "mrbus_base_address",
    "mrbus_update_interval_decisecs", "sleep_timeout_minutes", "alerter_timeout_minutes",
    "dead_reckoning_time", "time_source_address", "tx_holdoff_centisecs",
    "battery_okay_decivolts", "battery_warn_decivolts", "battery_critical_decivolts",
    "horn_threshold", "horn_threshold2", "brake_threshold", "brake_low_threshold",
    "brake_high_threshold", "config_bits",
}


def _flatten_global(d):
    """Accept the grouped (system/comm/prefs/calibration) device shape OR the older flat shape;
    return a flat {field_name: value} dict for the rest of encode_global. config_bits stays a dict
    (it just moves out from under prefs). A dict that is already flat passes straight through."""
    if not isinstance(d, dict) or not any(cat in d for cat in GLOBAL_CATEGORY_KEYS):
        return d
    flat = {k: v for k, v in d.items() if k not in GLOBAL_CATEGORY_KEYS}
    for cat in GLOBAL_CATEGORY_KEYS:
        sub = d.get(cat)
        if isinstance(sub, dict):
            flat.update(sub)
    return flat


# Order = on-device PREFS menu order (DISPLAY, then LED BLNK / REV LOCK / STRICT SLP; the SLEEP /
# ALERTER / TIMEOUT items in between are device-level, not config bits). Both decode and encode
# iterate this dict; encode keys off the explicit bit number, so the order only sets how the JSON reads.
CONFIGBITS_NAMED = {
    "main_screen_speed": layout.CONFIGBITS_MAIN_SCREEN_SPEED,
    "airbrake": layout.CONFIGBITS_AIRBRAKE,
    "led_blink": layout.CONFIGBITS_LED_BLINK,
    "reverser_lock": layout.CONFIGBITS_REVERSER_LOCK,
    "strict_sleep": layout.CONFIGBITS_STRICT_SLEEP,
}


def _decode_config_bits(raw):
    out = {name: bool(raw & (1 << bit)) for name, bit in CONFIGBITS_NAMED.items()}
    known_mask = 0
    for bit in CONFIGBITS_NAMED.values():
        known_mask |= (1 << bit)
    unknown_bits = raw & ~known_mask & 0xFF
    if unknown_bits:
        # No currently-named meaning for these bits, but preserved losslessly rather than silently
        # dropped, in case a future firmware version defines them.
        out["raw_unknown_bits"] = "0x%02X" % unknown_bits
    return out


def _encode_config_bits(d, errors):
    if not isinstance(d, dict):
        errors.append("config_bits: must be an object")
        return 0
    unknown = set(d.keys()) - set(CONFIGBITS_NAMED.keys()) - {"raw_unknown_bits"}
    for key in unknown:
        errors.append("config_bits.%s: unknown field" % key)
    raw = 0
    for name, bit in CONFIGBITS_NAMED.items():
        val = d.get(name, False)
        if not isinstance(val, bool):
            errors.append("config_bits.%s: must be true/false" % name)
            continue
        if val:
            raw |= (1 << bit)
    if "raw_unknown_bits" in d:
        try:
            raw |= int(d["raw_unknown_bits"], 16) & 0xFF
        except (TypeError, ValueError):
            errors.append("config_bits.raw_unknown_bits: must be a hex string like \"0x00\"")
    return raw


def decode_global(raw_128_bytes, source):
    if len(raw_128_bytes) != layout.CONFIG_START:
        raise ValueError("expected %d bytes, got %d" % (layout.CONFIG_START, len(raw_128_bytes)))
    raw = raw_128_bytes
    update_decisecs = struct.unpack_from(">H", raw, layout.EE_MRBUS_DEVICE_UPDATE_H)[0]
    # One object per config menu, in top-level-menu-cycle order: SYSTEM (BAT OKAY/WARN/CRIT, ADV-FUNC
    # gated) -> COMM (THRTL ID, BASE ADR, TIME ADR, TX INTVL, TX HLDOF) -> PREFS (DISPLAY+LED BLNK/REV
    # LOCK/STRICT SLP = config_bits, then SLEEP, ALERTER, TIMEOUT=dead_reckoning) -> THRESHOLD CAL
    # (HORN, HORN2, BRAKE, BRAKE LOW, BRAKE HIGH). Keys within each object are in that menu's item
    # order. encode_global() accepts this shape or the older flat one (via _flatten_global).
    return {
        "schema_version": SLOT_SCHEMA_VERSION,
        "source": source,
        "system": {
            "battery_okay_decivolts": raw[layout.EE_BATTERY_OKAY],
            "battery_warn_decivolts": raw[layout.EE_BATTERY_WARN],
            "battery_critical_decivolts": raw[layout.EE_BATTERY_CRITICAL],
        },
        "comm": {
            "mrbus_device_address": raw[layout.EE_MRBUS_DEVICE_ADDR],
            "mrbus_base_address": raw[layout.EE_BASE_ADDR],
            "time_source_address": raw[layout.EE_TIME_SOURCE_ADDRESS],
            "mrbus_update_interval_decisecs": update_decisecs,
            "tx_holdoff_centisecs": raw[layout.EE_TX_HOLDOFF],
        },
        "prefs": {
            "config_bits": _decode_config_bits(raw[layout.EE_CONFIGBITS]),
            "sleep_timeout_minutes": raw[layout.EE_DEVICE_SLEEP_TIMEOUT],
            "alerter_timeout_minutes": raw[layout.EE_ALERTER_TIMEOUT],
            "dead_reckoning_time": raw[layout.EE_DEAD_RECKONING_TIME],
        },
        "calibration": {
            "horn_threshold": raw[layout.EE_HORN_THRESHOLD],
            "horn_threshold2": raw[layout.EE_HORN_THRESHOLD2],
            "brake_threshold": raw[layout.EE_BRAKE_THRESHOLD],
            "brake_low_threshold": raw[layout.EE_BRAKE_LOW_THRESHOLD],
            "brake_high_threshold": raw[layout.EE_BRAKE_HIGH_THRESHOLD],
        },
    }


def _encode_u8_field(d, key, lo, hi, errors, out, offset):
    val = d.get(key)
    if not (isinstance(val, int) and not isinstance(val, bool) and lo <= val <= hi):
        errors.append("%s: must be an integer %d-%d" % (key, lo, hi))
        val = lo
    out[offset] = val


def encode_global(d, base=None):
    """dict -> a 128-byte bytearray for the global block. Raises SlotValidationError on invalid input.

    `base`, if given, must be the CURRENT 128 raw global-block bytes on the device - fields this schema
    doesn't model (MRBUS_EE_DEVICE_OPT_FLAGS, EE_VERSION_MAJOR/MINOR, EE_LAYOUT_VERSION, and other
    reserved/unmapped bytes) are preserved from it rather than zero-filled. This matters a lot more here
    than for encode_slot()'s padding: EE_LAYOUT_VERSION is the layout-version guard's own storage, and
    zeroing it would make the tool refuse to operate on this device on the next run. ALWAYS pass the
    real on-device bytes here before writing to hardware; omitting it (offline validation/tests only,
    where no "current" bytes exist) zero-fills unmodeled bytes instead.
    """
    errors = []
    if not isinstance(d, dict):
        raise SlotValidationError(["device config: must be a JSON object"])
    d = _flatten_global(d)
    unknown = set(d.keys()) - GLOBAL_FIELD_KEYS
    for key in unknown:
        errors.append("%s: unknown device field" % key)
    missing = GLOBAL_FIELD_KEYS - {"schema_version", "source"} - set(d.keys())
    for key in missing:
        errors.append("%s: missing" % key)

    if base is not None and len(base) == layout.CONFIG_START:
        out = bytearray(base)
    else:
        out = bytearray(layout.CONFIG_START)

    _encode_u8_field(d, "mrbus_device_address", layout.MRBUS_DEV_ADDR_MIN, layout.MRBUS_DEV_ADDR_MAX,
                      errors, out, layout.EE_MRBUS_DEVICE_ADDR)
    _encode_u8_field(d, "mrbus_base_address", layout.MRBUS_BASE_ADDR_MIN, layout.MRBUS_BASE_ADDR_MAX,
                      errors, out, layout.EE_BASE_ADDR)

    update_decisecs = d.get("mrbus_update_interval_decisecs")
    if not (isinstance(update_decisecs, int) and not isinstance(update_decisecs, bool) and
            layout.UPDATE_DECISECS_MIN <= update_decisecs <= layout.UPDATE_DECISECS_MAX):
        errors.append("mrbus_update_interval_decisecs: must be an integer %d-%d" %
                       (layout.UPDATE_DECISECS_MIN, layout.UPDATE_DECISECS_MAX))
        update_decisecs = layout.UPDATE_DECISECS_MIN
    struct.pack_into(">H", out, layout.EE_MRBUS_DEVICE_UPDATE_H, update_decisecs)

    _encode_u8_field(d, "sleep_timeout_minutes", layout.SLEEP_TMR_MIN, layout.SLEEP_TMR_MAX,
                      errors, out, layout.EE_DEVICE_SLEEP_TIMEOUT)
    _encode_u8_field(d, "alerter_timeout_minutes", layout.ALERTER_TMR_MIN, layout.ALERTER_TMR_MAX,
                      errors, out, layout.EE_ALERTER_TIMEOUT)
    _encode_u8_field(d, "dead_reckoning_time", 0, 255, errors, out, layout.EE_DEAD_RECKONING_TIME)
    _encode_u8_field(d, "time_source_address", 0, 255, errors, out, layout.EE_TIME_SOURCE_ADDRESS)
    _encode_u8_field(d, "tx_holdoff_centisecs", layout.TX_HOLDOFF_MIN, 255, errors, out,
                      layout.EE_TX_HOLDOFF)
    _encode_u8_field(d, "battery_okay_decivolts", 0, 255, errors, out, layout.EE_BATTERY_OKAY)
    _encode_u8_field(d, "battery_warn_decivolts", 0, 255, errors, out, layout.EE_BATTERY_WARN)
    _encode_u8_field(d, "battery_critical_decivolts", 0, 255, errors, out, layout.EE_BATTERY_CRITICAL)
    _encode_u8_field(d, "horn_threshold", 0, 255, errors, out, layout.EE_HORN_THRESHOLD)
    _encode_u8_field(d, "horn_threshold2", 0, 255, errors, out, layout.EE_HORN_THRESHOLD2)
    _encode_u8_field(d, "brake_threshold", 0, 255, errors, out, layout.EE_BRAKE_THRESHOLD)
    _encode_u8_field(d, "brake_low_threshold", 0, 255, errors, out, layout.EE_BRAKE_LOW_THRESHOLD)
    _encode_u8_field(d, "brake_high_threshold", 0, 255, errors, out, layout.EE_BRAKE_HIGH_THRESHOLD)

    if "config_bits" in d:
        out[layout.EE_CONFIGBITS] = _encode_config_bits(d["config_bits"], errors)
    else:
        errors.append("config_bits: missing")

    if errors:
        raise SlotValidationError(errors)
    return out
