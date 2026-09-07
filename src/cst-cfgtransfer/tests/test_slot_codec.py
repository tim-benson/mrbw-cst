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
            d[key] = "AIRBRAKE"
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


def _speed_field_value(key, type_name):
    if key == "UNIT":
        return "MPH"
    if key == "TYPE":
        return type_name
    if key in layout.SPEED_WATCHED_FN_FIELDS:
        return "F09"
    return 42


def _valid_speed(type_name="V5DCC"):
    return {key: _speed_field_value(key, type_name)
            for key in layout.speed_fields_for_type(type_name)}


def _valid_airbrake():
    # 80 satisfies every numeric field's range at once (BP_CHARGE 70-110, MR_LOAD 0-100, everything
    # else plain 0-254) so one value covers the numeric dict; DISPLAY and COMP_MODE are string enums.
    _strings = {"DISPLAY": "SINGLE", "COMP_MODE": "CONSIST"}
    return {key: _strings.get(key, 80) for key in layout.AIRBRAKE_FIELD_DEFAULTS}


def _valid_slot_dict(brk_type="PULSE"):
    return {
        "schema_version": slot_codec.SLOT_SCHEMA_VERSION,
        "source": {"scope": "slot", "slot": 5, "device_fw_version": "1.0", "exported_at": "2026-01-01T00:00:00"},
        "loco_address": {"address": 4302, "type": "long"},
        "force_functions": {"on": [3, 12], "off": []},
        "functions": _valid_functions(),
        "notch_speedstep": [10, 25, 40, 55, 70, 90, 110, 126],
        "speed": _valid_speed(),
        "airbrake": _valid_airbrake(),
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
            "config_bits": {"main_screen_speed": False, "airbrake": False, "led_blink": True,
                             "reverser_lock": True, "strict_sleep": True},
            "sleep_timeout_minutes": 5,
            "alerter_timeout_minutes": 0,
            "dead_reckoning_time": 10,
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
    The two-stage horn claims two bytes the codec would otherwise never touch - EE_HORN2_FUNCTION (at
    0x49 after the SPEED/AIRBRAKE hole repack, 0x4D before it) and EE_HORN_THRESHOLD2 at 0x27 (a global
    calibration point) - and an incomplete mirror of either would silently misdecode. The CNF format version is no help
    against that: it only says "the layout changed", not "each field is present". Those are the checks
    that caught the real 2026-08-25 bug.

    test_function_fields_includes_horn2 / test_horn_threshold2_is_mirrored / test_horn_type_is_mirrored
    pin those three fields; test_layout_version_derive_path checks the one thing left to verify about
    SUPPORTED_LAYOUT_VERSION now that it is parsed from cst-eeprom.h rather than hand-copied.
    """

    def test_function_fields_includes_horn2(self):
        self.assertEqual(len(layout.FUNCTION_FIELDS), 28)
        self.assertIn(("HORN2", 0x49, 0), layout.FUNCTION_FIELDS)

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
        expected = ["HORN", "HORN2", "BELL", "BRAKE", "BRAKE2", "BRAKE3", "AUX",
                    "ENGINE_ON", "ENGINE_OFF", "THR_UNLOCK", "REV_SWAP", "NEUTRAL",
                    "COMPRESSOR", "COMPRESSOR2", "BRAKE_SET", "BRAKE_REL", "ALERTER",
                    "EMERGENCY", "FRONT_HEADLIGHT", "FRONT_DITCH", "FRONT_DIM1",
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
        # SPEED CFG shows the 6 type-agnostic items first (TYPE leads), then the current TYPE's model
        # params - V5DCC/V5MULT carry all 13, V4 the 7-item subset. Mirrors the cst-speed.c descriptors.
        agnostic = ["TYPE", "MAXSPEED", "UNIT", "ACCEL", "DECEL", "BRK1"]
        self.assertEqual(layout.SPEED_AGNOSTIC_FIELDS, agnostic)
        self.assertEqual(layout.speed_fields_for_type("V5DCC"), agnostic + [
            "BRK2", "BRK3", "DELAY", "HOLDFN", "STOPFN", "OPLOAD", "OPLOADFN", "PRLOAD", "PRLOADFN",
            "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"])
        self.assertEqual(layout.speed_fields_for_type("V5MULT"), layout.speed_fields_for_type("V5DCC"))
        self.assertEqual(layout.speed_fields_for_type("V4"), agnostic + [
            "DELAY", "HOLDFN", "STOPFN", "ACCPCT", "ACCTGT", "DECPCT", "DECTHR"])

    def test_airbrake_keys_match_airbrake_cfg_menu_order(self):
        # = AIRBRAKE_CONFIG_SCREEN item order (cst-pressure.h AIRBRAKE_* enum) - all 9 always visible,
        # no ADV-FUNC gating left in this menu. DISPLAY sits immediately above COMP_MODE.
        expected = ["BP_CHARGE", "MR_LOAD", "MR_LOW", "MR_HIGH", "RECHARGE", "LEAK_RATE", "PUMP_RATE",
                    "DISPLAY", "COMP_MODE"]
        self.assertEqual([k for k, _off in layout.AIRBRAKE_FIELDS], expected)
        self.assertEqual(list(slot_codec.decode_slot(bytes(128), source={})["airbrake"]), expected)

    def test_config_bits_match_prefs_menu_order(self):
        self.assertEqual(list(slot_codec.CONFIGBITS_NAMED),
                         ["main_screen_speed", "airbrake", "led_blink", "reverser_lock", "strict_sleep"])

    def test_slot_top_level_sections_in_menu_order(self):
        # One object per menu: LOCO -> FORCE FUNC -> CONFIG FUNC -> NOTCH -> SPEED CFG -> AIRBRAKE CFG
        # -> OPTIONS.
        expected = ["schema_version", "source", "loco_address", "force_functions", "functions",
                    "notch_speedstep", "speed", "airbrake", "options"]
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
                                             "alerter_timeout_minutes", "dead_reckoning_time"])
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
        for key, _off in layout.AIRBRAKE_FIELDS:
            self.assertEqual(decoded["airbrake"][key], slot_codec.UNSET)
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
        for key in layout.SPEED_FIELD_OFFSET:
            self.assertEqual(encoded[layout.SPEED_FIELD_OFFSET[key]], 0xFF)
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

    def test_airbrake_fn_valid_on_menu_func_only(self):
        d = _valid_slot_dict()
        d["functions"]["UP_BUTTON"] = "AIRBRAKE"
        slot_codec.encode_slot(d)  # should not raise

        d2 = _valid_slot_dict()
        d2["functions"]["AUX"] = "AIRBRAKE"  # SPECIAL but not MENU
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


class SpeedTypeTests(unittest.TestCase):
    """The `speed` object is decoder-family-shaped: V5DCC/V5MULT carry all 13 model fields, V4 the
    7-item subset (no BRK2/BRK3, no load CVs). Mirrors cst-speed.c's per-TYPE descriptors and
    speedResetModel()/speedItemInert()."""

    V4_DROPPED = {"BRK2", "BRK3", "OPLOAD", "OPLOADFN", "PRLOAD", "PRLOADFN"}

    def _slot_with_speed(self, speed):
        d = _valid_slot_dict("PULSE")
        d["speed"] = speed
        return d

    def test_v5mult_round_trips_like_v5dcc(self):
        for tname in ("V5DCC", "V5MULT"):
            d = self._slot_with_speed(_valid_speed(tname))
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["speed"], d["speed"])
            self.assertEqual(len(decoded["speed"]), 19)

    def test_v4_speed_has_only_its_seven_model_fields(self):
        d = self._slot_with_speed(_valid_speed("V4"))
        decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
        self.assertEqual(decoded["speed"], d["speed"])
        self.assertEqual(set(decoded["speed"]) & self.V4_DROPPED, set())
        self.assertEqual(len(decoded["speed"]), 13)

    def test_encoding_v4_forces_dropped_slots_inert(self):
        d = self._slot_with_speed(_valid_speed("V4"))
        encoded = slot_codec.encode_slot(d)
        for key, inert in layout.SPEED_MODEL_INERT.items():
            self.assertEqual(encoded[layout.SPEED_FIELD_OFFSET[key]], inert,
                             "%s should be inert (%d) for a V4 slot" % (key, inert))

    def test_decoding_v4_type_byte_yields_v4_shape(self):
        # A raw image whose TYPE byte says V4 decodes to the 13-field shape regardless of what the
        # dropped slots hold.
        raw = bytearray(b"\xFF" * layout.CONFIG_SIZE)
        raw[layout.EE_SPEED_TYPE] = layout.SPEED_TYPE_V4
        raw[layout.SPEED_FIELD_OFFSET["BRK2"]] = 90   # stale value the V4 shape must not surface
        speed = slot_codec.decode_slot(bytes(raw), source={})["speed"]
        self.assertEqual(speed["TYPE"], "V4")
        self.assertNotIn("BRK2", speed)
        self.assertEqual(len(speed), 13)

    def test_v4_slot_rejects_a_dropped_field(self):
        speed = _valid_speed("V4")
        speed["BRK2"] = 70
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(self._slot_with_speed(speed))

    def test_flat_v4_backup_needs_import_old_then_round_trips(self):
        # A pre-split flat backup (all 19 fields) with TYPE hand-changed to V4: rejected on a plain
        # import, accepted with --import-old (the inapplicable fields ignored, slots forced inert).
        flat = _valid_speed("V5DCC")
        flat["TYPE"] = "V4"
        d = self._slot_with_speed(flat)
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        for key, inert in layout.SPEED_MODEL_INERT.items():
            self.assertEqual(encoded[layout.SPEED_FIELD_OFFSET[key]], inert)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["speed"], _valid_speed("V4"))


class AirbrakeFieldTests(unittest.TestCase):
    """DISPLAY and COMP_MODE are string enums rather than plain ints; BP_CHARGE and MR_LOAD are the two
    range-checked against the on-device UP/DOWN ceiling/floor rather than the generic 0-254 every other
    field gets. These pin that validation - see _encode_airbrake()."""

    def test_comp_mode_round_trips_both_values(self):
        for value in ("NORMAL", "CONSIST"):
            d = _valid_slot_dict()
            d["airbrake"]["COMP_MODE"] = value
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["airbrake"]["COMP_MODE"], value)

    def test_comp_mode_invalid_string_rejected(self):
        d = _valid_slot_dict()
        d["airbrake"]["COMP_MODE"] = "MAYBE"
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_display_round_trips_both_values(self):
        for value in ("DUAL", "SINGLE"):
            d = _valid_slot_dict()
            d["airbrake"]["DISPLAY"] = value
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["airbrake"]["DISPLAY"], value)

    def test_display_invalid_string_rejected(self):
        d = _valid_slot_dict()
        d["airbrake"]["DISPLAY"] = "TRIPLE"
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_bp_charge_out_of_range_rejected(self):
        for value in (layout.AIRBRAKE_BP_CHARGE_MIN - 1, layout.AIRBRAKE_BP_CHARGE_MAX + 1):
            d = _valid_slot_dict()
            d["airbrake"]["BP_CHARGE"] = value
            with self.assertRaises(slot_codec.SlotValidationError):
                slot_codec.encode_slot(d)

    def test_bp_charge_boundaries_accepted(self):
        for value in (layout.AIRBRAKE_BP_CHARGE_MIN, layout.AIRBRAKE_BP_CHARGE_MAX):
            d = _valid_slot_dict()
            d["airbrake"]["BP_CHARGE"] = value
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["airbrake"]["BP_CHARGE"], value)

    def test_mr_load_out_of_range_rejected(self):
        d = _valid_slot_dict()
        d["airbrake"]["MR_LOAD"] = layout.AIRBRAKE_MR_LOAD_MAX + 1
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d)

    def test_mr_load_boundaries_accepted(self):
        for value in (0, layout.AIRBRAKE_MR_LOAD_MAX):
            d = _valid_slot_dict()
            d["airbrake"]["MR_LOAD"] = value
            decoded = slot_codec.decode_slot(slot_codec.encode_slot(d), source=d["source"])
            self.assertEqual(decoded["airbrake"]["MR_LOAD"], value)


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

    def test_missing_airbrake_key_allowed_and_decodes_to_unset(self):
        d = _valid_slot_dict()
        del d["airbrake"]["LEAK_RATE"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        self.assertEqual(decoded["airbrake"]["LEAK_RATE"], slot_codec.UNSET)

    def test_missing_whole_airbrake_section_allowed_pre_v3_backup(self):
        # airbrake is the one whole-category exception - a v2 backup lacks it entirely.
        d = _valid_slot_dict()
        del d["airbrake"]
        encoded = slot_codec.encode_slot(d, allow_missing=True)
        decoded = slot_codec.decode_slot(encoded, source=d["source"])
        for key, _off in layout.AIRBRAKE_FIELDS:
            self.assertEqual(decoded["airbrake"][key], slot_codec.UNSET)
        # ...but still rejected without the flag
        d2 = _valid_slot_dict()
        del d2["airbrake"]
        with self.assertRaises(slot_codec.SlotValidationError):
            slot_codec.encode_slot(d2)

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
