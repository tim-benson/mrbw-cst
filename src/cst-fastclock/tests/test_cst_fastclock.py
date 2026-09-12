#!/usr/bin/env python3
# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Unit tests for cst_fastclock.py's pure functions - no hardware/serial port involved, matching the
test-without-hardware pattern used by cst-cfgnetwork/tests/test_cnf_radio_io.py."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))

import cst_fastclock as fc  # noqa: E402


class ParseHmsTests(unittest.TestCase):
    def test_hh_mm(self):
        self.assertEqual(fc.parse_hms("08:00"), 8 * 3600)

    def test_hh_mm_ss(self):
        self.assertEqual(fc.parse_hms("23:59:59"), 23 * 3600 + 59 * 60 + 59)

    def test_midnight(self):
        self.assertEqual(fc.parse_hms("00:00:00"), 0)

    def test_rejects_bad_shape(self):
        with self.assertRaises(Exception):
            fc.parse_hms("08")

    def test_rejects_out_of_range(self):
        with self.assertRaises(Exception):
            fc.parse_hms("24:00")
        with self.assertRaises(Exception):
            fc.parse_hms("08:60")

    def test_rejects_non_numeric(self):
        with self.assertRaises(Exception):
            fc.parse_hms("noon")


class FormatHmsTests(unittest.TestCase):
    def test_basic(self):
        self.assertEqual(fc.format_hms(8 * 3600 + 4 * 60 + 12), "08:04:12")

    def test_wraps_at_24h(self):
        self.assertEqual(fc.format_hms(86400 + 3661), "01:01:01")

    def test_negative_wraps_backward(self):
        self.assertEqual(fc.format_hms(-1), "23:59:59")


class ComputeFastSecondsTests(unittest.TestCase):
    def test_no_elapsed_time_returns_start(self):
        self.assertEqual(fc.compute_fast_seconds(8 * 3600, 4.0, 0.0), 8 * 3600)

    def test_scales_by_ratio(self):
        # 10 real seconds at 4x ratio -> 40 fast seconds elapsed.
        self.assertEqual(fc.compute_fast_seconds(0, 4.0, 10.0), 40)

    def test_real_time_ratio_is_pass_through(self):
        self.assertEqual(fc.compute_fast_seconds(100, 1.0, 5.0), 105)

    def test_wraps_past_midnight(self):
        start = 23 * 3600 + 59 * 60  # 23:59:00
        # 4x ratio, 30 real seconds -> 120 fast seconds elapsed -> 00:01:00 next day.
        self.assertEqual(fc.compute_fast_seconds(start, 4.0, 30.0), 60)


class BuildTimePayloadTests(unittest.TestCase):
    def test_length_and_offsets(self):
        payload = fc.build_time_payload(
            real_seconds=14 * 3600 + 32 * 60 + 7,
            fast_seconds=8 * 3600 + 4 * 60 + 12,
            ratio=4.0,
            flags=fc.FLAG_DISP_FAST,
        )
        self.assertEqual(len(payload), 12)
        self.assertEqual(tuple(payload[0:3]), (14, 32, 7))     # real h/m/s
        self.assertEqual(payload[3], fc.FLAG_DISP_FAST)        # flags
        self.assertEqual(tuple(payload[4:7]), (8, 4, 12))      # fast h/m/s
        self.assertEqual((payload[7] << 8) | payload[8], 40)   # scaleFactor = ratio*10
        self.assertEqual(tuple(payload[9:12]), (0, 0, 0))      # date bytes unused

    def test_scale_factor_rounding(self):
        payload = fc.build_time_payload(0, 0, 2.5, 0)
        self.assertEqual((payload[7] << 8) | payload[8], 25)

    def test_scale_factor_clamped_to_valid_range(self):
        low = fc.build_time_payload(0, 0, 0.0, 0)
        high = fc.build_time_payload(0, 0, 999.0, 0)
        self.assertEqual((low[7] << 8) | low[8], 1)
        self.assertEqual((high[7] << 8) | high[8], 999)

    def test_flags_pass_through(self):
        flags = fc.FLAG_DISP_FAST | fc.FLAG_DISP_FAST_HOLD | fc.FLAG_DISP_FAST_AMPM
        payload = fc.build_time_payload(0, 0, 1.0, flags)
        self.assertEqual(payload[3], flags)

    def test_wraps_seconds_inputs_over_24h(self):
        payload = fc.build_time_payload(86400 + 3661, 0, 1.0, 0)
        self.assertEqual(tuple(payload[0:3]), (1, 1, 1))


if __name__ == "__main__":
    unittest.main()
