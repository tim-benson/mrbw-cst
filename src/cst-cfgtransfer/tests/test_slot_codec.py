"""Offline round-trip tests for slot_codec.py - no hardware/avrdude needed.

Run with: python3 -m unittest discover -s src/cst-cfgtransfer/tests -t src/cst-cfgtransfer
       or: cd src/cst-cfgtransfer && python3 -m unittest discover tests
"""

import os
import sys
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import cst_eeprom_layout as layout  # noqa: E402
import slot_codec  # noqa: E402


def _valid_functions():
    d = {}
    for key, _offset, attrs in layout.FUNCTION_FIELDS:
        if attrs & layout.FUNC_MENU:
            d[key] = "BRKTEST"
        elif attrs & layout.FUNC_SPECIAL:
            d[key] = "EMRG"
        else:
            d[key] = "F02_MOM"
    return d


def _valid_options(brk_type="PULSE"):
    return {
        "unset": False,
        "type": brk_type,
        "estop_on_full_brake": True,
        "reverser_swap": False,
        "variable_brake": brk_type != "PULSE",
        "stack_5step": True,
        "pulse_width": 5,
        "stack_band_combos_5step": ["BRAKE1--", "BRAKE-23", "BRAKE1-3", "BRAKE12-", "BRAKE123"],
        "stack_band_combos_3step": ["BRAKE1--", "BRAKE-2-", "BRAKE--3"],
        "horn_type": "ADDITIVE",
    }


def _valid_speed():
    d = {}
    for key in layout.SPEED_FIELD_DEFAULTS:
        if key == "UNIT":
            d[key] = "MPH"
        elif key == "TYPE":
            d[key] = "V5DCC"
        elif key in layout.SPEED_WATCHED_FN_FIELDS:
            d[key] = "F09"
        else:
            d[key] = 42
    return d


def _valid_slot_dict(brk_type="PULSE"):
    return {
        "schema_version": slot_codec.SLOT_SCHEMA_VERSION,
        "source": {"scope": "slot", "slot": 5, "device_fw_version": "1.0", "exported_at": "2026-01-01T00:00:00"},
        "loco_address": {"address": 4302, "type": "long"},
        "force_functions": {"on": [3, 12], "off": []},
        "functions": _valid_functions(),
        "notch_speedstep": [10, 25, 40, 55, 70, 90, 110, 126],
        "speed": _valid_speed(),
        "options": _valid_options(brk_type),
    }


# The pre-schema-2 slot shape: flat force_function_on/off pair; `brake` object with `options_unset`.
# Kept as a fixture so the "still accepts the older shapes on import" back-compat path is exercised,
# not merely asserted.
def _legacy_flat_slot_dict(brk_type="PULSE"):
    d = _valid_slot_dict(brk_type)
    ff = d.pop("force_functions")
    d["force_function_on"] = ff["on"]
    d["force_function_off"] = ff["off"]
    opts = d.pop("options")
    opts["options_unset"] = opts.pop("unset")
    d["brake"] = opts
    return d


def _valid_global_dict():
    return {
        "schema_version": slot_codec.SLOT_SCHEMA_VERSION,
        "source": {"scope": "device", "device_fw_version": "1.0", "exported_at": "2026-01-01T00:00:00"},
        "system": {
            "battery_okay_decivolts": 90,
            "battery_warn_decivolts": 80,
            "battery_critical_decivolts": 70,
        },
        "comm": {
            "mrbus_device_address": 0x35,
            "mrbus_base_address": 0xD0,
            "time_source_address": 0,
            "mrbus_update_interval_decisecs": 20,
            "tx_holdoff_centisecs": 15,
        },
        "prefs": {
            "config_bits": {"main_screen_speed": False, "led_blink": True, "reverser_lock": True,
                             "strict_sleep": True},
            "sleep_timeout_minutes": 5,
            "alerter_timeout_minutes": 0,
            "dead_reckoning_time": 10,
            "pressure_config": 0,
        },
        "calibration": {
            "horn_threshold": 100,
            "horn_threshold2": 150,
            "brake_threshold": 100,
            "brake_low_threshold": 90,
            "brake_high_threshold": 200,
        },
    }


def _legacy_flat_global_dict():
    d = _valid_global_dict()
    flat = {"schema_version": d["schema_version"], "source": d["source"]}
    for cat in ("comm", "prefs", "calibration", "system"):
        flat.update(d[cat])
    return flat


class LayoutVersionRegressionTests(unittest.TestCase):
    """Guards that every EEPROM field the two-stage horn introduced is actually mirrored in this codec.
    The two-stage horn claims two bytes the codec would otherwise never touch - EE_HORN2_FUNCTION at
    0x4D (previously inert slot padding) and EE_HORN_THRESHOLD2 at 0x27 (a global calibration point) -
    and an incomplete mirror of either would silently misdecode. The CNF format version is no help
    against that: it only says "the layout changed", not "each field is present". Those are the checks
    that caught the real 2026-08-25 bug.

    test_function_fields_includes_horn2 / test_horn_threshold2_is_mirrored / test_horn_type_is_mirrored
    pin those three fields; test_layout_version_derive_path checks the one thing left to verify about
    SUPPORTED_LAYOUT_VERSION now that it is parsed from cst-eeprom.h rather than hand-copied.
    """

    def test_function_fields_includes_horn2(self):
        self.assertEqual(len(layout.FUNCTION_FIELDS), 26)
        self.assertIn(("HORN2", 0x4D, 0), layout.FUNCTION_FIELDS)

    def test_horn_threshold2_is_mirrored(self):
        self.assertEqual(layout.EE_HORN_THRESHOLD2, 0x27)
        d = _valid_global_dict()
        encoded = slot_codec.encode_global(d)
        decoded = slot_codec.decode_global(encoded, source=d["source"])
        self.assertEqual(decoded["calibration"]["horn_threshold2"], d["calibration"]["horn_threshold2"])

    def test_horn_type_is_mirrored(self):
        # OPTIONBITS_HORN_TYPE (bit 6 of optionBits) drives the OPTIONS menu's HORNTYPE item; it went
        # unmirrored until schema 2, so a round-trip silently lost Additive/Exclusive.
        self.assertEqual(layout.OPTIONBITS_HORN_TYPE, 6)
        for name in ("ADDITIVE", "EXCLUSIVE"):
            d = _valid_slot_dict("PULSE")
            d["options"]["horn_type"] = name
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["options"]["horn_type"], name)

    def test_layout_version_derive_path(self):
        parsed = layout._read_firmware_layout_version()
        self.assertIsInstance(parsed, int)
        self.assertNotIn(parsed, (0x00, 0xFF))
        self.assertEqual(layout.SUPPORTED_LAYOUT_VERSION, parsed)


class MenuOrderTests(unittest.TestCase):
    """JSON key order within each section is required to match the corresponding on-device menu (see
    slot_codec / cst_eeprom_layout comments). These lock that in so a reorder is a deliberate, visible
    change rather than something that drifts silently."""

    def test_functions_match_config_func_menu_order(self):
        # = the Functions enum in src/cst-functions.h, which advanceCurrentFunction() iterates.
        expected = ["HORN", "HORN2", "BELL", "BRAKE", "BRAKE2", "BRAKE3", "BRAKE_OFF", "AUX",
                    "ENGINE_ON", "ENGINE_OFF", "THR_UNLOCK", "REV_SWAP", "NEUTRAL", "ALERTER",
                    "COMPRESSOR", "BRAKE_TEST", "FRONT_HEADLIGHT", "FRONT_DITCH", "FRONT_DIM1",
                    "FRONT_DIM2", "REAR_HEADLIGHT", "REAR_DITCH", "REAR_DIM1", "REAR_DIM2",
                    "UP_BUTTON", "DOWN_BUTTON"]
        self.assertEqual([k for k, _off, _a in layout.FUNCTION_FIELDS], expected)
        self.assertEqual(list(slot_codec.decode_slot(bytes(128), source={})["functions"]), expected)

    def test_options_keys_match_options_menu_order(self):
        expected = ["unset", "variable_brake", "type", "pulse_width", "stack_5step",
                    "stack_band_combos_5step", "stack_band_combos_3step", "estop_on_full_brake",
                    "reverser_swap", "horn_type"]
        self.assertEqual(list(slot_codec.decode_slot(bytes(128), source={})["options"]), expected)

    def test_speed_keys_match_speed_cfg_menu_order(self):
        expected = ["ACCEL", "DECEL", "BRK1", "BRK2", "BRK3", "DELAY", "MAXSPEED", "UNIT", "HOLDFN",
                    "STOPFN", "OPLOAD", "OPLOADFN", "PRLOAD", "PRLOADFN", "TYPE", "ACCPCT", "ACCTGT",
                    "DECPCT", "DECTHR"]
        self.assertEqual([k for k, _off in layout.SPEED_FIELDS], expected)

    def test_config_bits_match_prefs_menu_order(self):
        self.assertEqual(list(slot_codec.CONFIGBITS_NAMED),
                         ["main_screen_speed", "led_blink", "reverser_lock", "strict_sleep"])

    def test_slot_top_level_sections_in_menu_order(self):
        # One object per menu: LOCO -> FORCE FUNC -> CONFIG FUNC -> NOTCH -> SPEED CFG -> OPTIONS.
        expected = ["schema_version", "source", "loco_address", "force_functions", "functions",
                    "notch_speedstep", "speed", "options"]
        self.assertEqual(list(slot_codec.decode_slot(bytes(128), source={})), expected)
        self.assertEqual(list(slot_codec.decode_slot(bytes(128), source={})["force_functions"]),
                         ["on", "off"])

    def test_device_categories_and_fields_in_menu_order(self):
        g = slot_codec.decode_global(bytes(64) + bytes(b"\xff" * 64), source={})
        # Top-level = config-menu cycle order.
        self.assertEqual(list(g), ["schema_version", "source", "system", "comm", "prefs", "calibration"])
        self.assertEqual(list(g["system"]), ["battery_okay_decivolts", "battery_warn_decivolts",
                                              "battery_critical_decivolts"])
        self.assertEqual(list(g["comm"]), ["mrbus_device_address", "mrbus_base_address",
                                            "time_source_address", "mrbus_update_interval_decisecs",
                                            "tx_holdoff_centisecs"])
        self.assertEqual(list(g["prefs"])[0], "config_bits")  # DISPLAY is PREFS item 1
        self.assertEqual(list(g["prefs"]), ["config_bits", "sleep_timeout_minutes",
                                             "alerter_timeout_minutes", "dead_reckoning_time",
                                             "pressure_config"])
        self.assertEqual(list(g["calibration"]), ["horn_threshold", "horn_threshold2", "brake_threshold",
                                                   "brake_low_threshold", "brake_high_threshold"])

    def test_legacy_flat_shapes_still_encode(self):
        # A pre-schema-2 backup (flat force_function_on/off; flat device fields) must still import,
        # producing byte-identical output to the grouped form.
        self.assertEqual(slot_codec.encode_slot(_legacy_flat_slot_dict()),
                         slot_codec.encode_slot(_valid_slot_dict()))
        self.assertEqual(slot_codec.encode_global(_legacy_flat_global_dict()),
                         slot_codec.encode_global(_valid_global_dict()))


class SlotRoundTripTests(unittest.TestCase):

    def test_round_trip_pulse(self):
        d = _valid_slot_dict("PULSE")
        encoded = slot_codec.encode_slot(d)
        self.assertEqual(len(encoded), layout.CONFIG_SIZE)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded, d)

    def test_round_trip_step(self):
        d = _valid_slot_dict("STEP")
        encoded = slot_codec.encode_slot(d)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded, d)

    def test_round_trip_stack(self):
        d = _valid_slot_dict("STACK")
        encoded = slot_codec.encode_slot(d)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded, d)

    def test_all_ff_slot_decodes_to_unset_not_crash(self):
        raw = bytes([0xFF] * layout.CONFIG_SIZE)
        decoded = slot_codec.decode_slot(raw, source={})
        # Self-healing fields all read as UNSET, not a fabricated default.
        self.assertTrue(decoded["options"]["unset"])
        for v in decoded["options"]["stack_band_combos_5step"]:
            self.assertEqual(v, slot_codec.UNSET)
        for v in decoded["options"]["stack_band_combos_3step"]:
            self.assertEqual(v, slot_codec.UNSET)
        for key in layout.SPEED_FIELD_DEFAULTS:
            self.assertEqual(decoded["speed"][key], slot_codec.UNSET)
        # Function bytes have no self-heal - an unrecognized raw byte round-trips as a RAW: passthrough.
        for key in decoded["functions"]:
            self.assertEqual(decoded["functions"][key], "RAW:0xFF")

    def test_unset_round_trips_to_0xff(self):
        d = _valid_slot_dict()
        # options_unset=True makes encode_slot ignore these; set them to decode_slot's own fixed
        # self-heal representation so the round-trip comparison below matches exactly.
        d["options"]["unset"] = True
        d["options"]["type"] = "PULSE"
        d["options"]["estop_on_full_brake"] = True
        d["options"]["reverser_swap"] = False
        d["options"]["variable_brake"] = False
        d["options"]["stack_5step"] = False
        for i in range(5):
            d["options"]["stack_band_combos_5step"][i] = slot_codec.UNSET
        for i in range(3):
            d["options"]["stack_band_combos_3step"][i] = slot_codec.UNSET
        for key in d["speed"]:
            d["speed"][key] = slot_codec.UNSET
        encoded = slot_codec.encode_slot(d)
        self.assertEqual(encoded[layout.EE_OPTIONBITS], 0xFF)
        for i in range(5):
            self.assertEqual(encoded[layout.EE_STACK_BAND_COMBOS + i], 0xFF)
        for i in range(3):
            self.assertEqual(encoded[layout.EE_STACK_BAND_COMBOS_3STEP + i], 0xFF)
        for _key, offset in layout.SPEED_FIELDS:
            self.assertEqual(encoded[offset], 0xFF)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded, d)

    def test_every_function_value_round_trips(self):
        cases = ["OFF", "F00_MOM", "F28_MOM", "F00_LAT", "F28_LAT"]
        for value in cases:
            d = _valid_slot_dict()
            d["functions"]["HORN"] = value
            encoded = slot_codec.encode_slot(d)
            decoded = slot_codec.decode_slot(encoded, source=d["source"])
            self.assertEqual(decoded["functions"]["HORN"], value)

    def test_emrg_valid_on_special_func_only(self):
        d = _valid_slot_dict()
        d["functions"]["AUX"] = "EMRG"
        slot_codec.encode_slot(d)  # should not raise

        d2 = _valid_slot_dict()
        d2["functions"]["HORN"] = "EMRG"
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d2)

    def test_brktest_valid_on_menu_func_only(self):
        d = _valid_slot_dict()
        d["functions"]["UP_BUTTON"] = "BRKTEST"
        slot_codec.encode_slot(d)  # should not raise

        d2 = _valid_slot_dict()
        d2["functions"]["AUX"] = "BRKTEST"  # SPECIAL but not MENU
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d2)

    def test_stack_combo_all_eight_values(self):
        for c1 in ("-", "1"):
            for c2 in ("-", "2"):
                for c3 in ("-", "3"):
                    s = "BRAKE" + c1 + c2 + c3
                    errors = []
                    raw = slot_codec._encode_stack_combo(s, errors, "test")
                    self.assertEqual(errors, [])
                    self.assertEqual(slot_codec._decode_stack_combo(raw), s)

    def test_loco_address_boundaries(self):
        for addr, kind in ((0, "short"), (127, "short"), (0, "long"), (9999, "long")):
            d = _valid_slot_dict()
            d["loco_address"] = {"address": addr, "type": kind}
            encoded = slot_codec.encode_slot(d)
            decoded = slot_codec.decode_slot(encoded, source=d["source"])
            self.assertEqual(decoded["loco_address"], {"address": addr, "type": kind})

    def test_loco_address_out_of_range_rejected(self):
        d = _valid_slot_dict()
        d["loco_address"] = {"address": 128, "type": "short"}
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_notch_speedstep_boundaries(self):
        d = _valid_slot_dict()
        d["notch_speedstep"] = [1, 1, 1, 1, 126, 126, 126, 126]
        encoded = slot_codec.encode_slot(d)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["notch_speedstep"], [1, 1, 1, 1, 126, 126, 126, 126])

    def test_missing_top_level_key_rejected(self):
        d = _valid_slot_dict()
        del d["speed"]
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_unknown_top_level_key_rejected(self):
        d = _valid_slot_dict()
        d["bogus_field"] = 1
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_unknown_function_value_rejected(self):
        d = _valid_slot_dict()
        d["functions"]["HORN"] = "NOT_A_REAL_VALUE"
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)


class GlobalRoundTripTests(unittest.TestCase):

    def test_round_trip(self):
        d = _valid_global_dict()
        encoded = slot_codec.encode_global(d)
        self.assertEqual(len(encoded), layout.CONFIG_START)
        decoded = slot_codec.decode_global(encoded, source=d["source"])
        self.assertEqual(decoded, d)

    def test_unknown_configbits_preserved(self):
        raw = bytearray(slot_codec.encode_global(_valid_global_dict()))
        raw[layout.EE_CONFIGBITS] = 0xFF  # every bit set, including unnamed ones
        decoded = slot_codec.decode_global(bytes(raw), source={})
        self.assertIn("raw_unknown_bits", decoded["prefs"]["config_bits"])
        re_encoded = slot_codec.encode_global(decoded)
        self.assertEqual(re_encoded[layout.EE_CONFIGBITS], 0xFF)

    def test_out_of_range_device_address_rejected(self):
        d = _valid_global_dict()
        d["comm"]["mrbus_device_address"] = 0x99  # outside MRBUS_DEV_ADDR_MIN..MAX
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_global(d)


class AllowMissingTests(unittest.TestCase):
    """`allow_missing=True` (the `--import-old` CLI flag) is for restoring a JSON backup taken under an
    OLDER codec, after a layout-version bump added a new field - see encode_slot()'s docstring. Confirms
    the flag relaxes only a genuinely ABSENT field within an existing category, never a whole missing
    top-level category, and never a field that's present but invalid."""

    def test_missing_function_key_rejected_by_default(self):
        d = _valid_slot_dict()
        del d["functions"]["HORN2"]
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_missing_function_key_allowed_and_decodes_to_raw_unset(self):
        d = _valid_slot_dict()
        del d["functions"]["HORN2"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["functions"]["HORN2"], "RAW:0xFF")

    def test_missing_speed_key_allowed_and_decodes_to_unset(self):
        d = _valid_slot_dict()
        del d["speed"]["ACCEL"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["speed"]["ACCEL"], slot_codec.UNSET)

    def test_missing_options_subfield_allowed(self):
        d = _valid_slot_dict()
        del d["options"]["pulse_width"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["options"]["pulse_width"], layout.BRAKE_PULSE_WIDTH_MIN)

    def test_missing_options_stack_combo_list_allowed_defaults_to_all_unset(self):
        d = _valid_slot_dict()
        del d["options"]["stack_band_combos_5step"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["options"]["stack_band_combos_5step"], [slot_codec.UNSET] * 5)

    def test_present_but_invalid_field_still_rejected_with_allow_missing(self):
        d = _valid_slot_dict()
        d["functions"]["HORN2"] = "NOT_A_REAL_VALUE"
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d, allow_missing=True)

    def test_missing_top_level_category_still_rejected_with_allow_missing(self):
        d = _valid_slot_dict()
        del d["functions"]
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d, allow_missing=True)


if __name__ == "__main__":
    unittest.main()
