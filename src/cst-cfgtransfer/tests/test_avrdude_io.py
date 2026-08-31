"""Offline tests for avrdude_io.py's pure retry logic - no hardware/avrdude needed. _run_avrdude() itself
is mocked out, since it's the one function that actually shells out.

Run with: python3 -m unittest discover -s src/cst-cfgtransfer/tests -t src/cst-cfgtransfer
       or: cd src/cst-cfgtransfer && python3 -m unittest discover tests
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

import avrdude_io  # noqa: E402


class TestRunAvrdudeWithRetries(unittest.TestCase):
    def test_succeeds_immediately_without_retry(self):
        with mock.patch.object(avrdude_io, "_run_avrdude", return_value="ok") as m:
            result = avrdude_io._run_avrdude_with_retries("eeprom:w:x:r")
        self.assertEqual(result, "ok")
        self.assertEqual(m.call_count, 1)

    def test_succeeds_on_a_later_attempt(self):
        err = avrdude_io.AvrdudeError("boom")
        with mock.patch.object(avrdude_io, "_run_avrdude", side_effect=[err, err, "ok"]):
            with mock.patch.object(avrdude_io, "time") as mock_time:
                result = avrdude_io._run_avrdude_with_retries("eeprom:w:x:r")
        self.assertEqual(result, "ok")
        self.assertEqual(mock_time.sleep.call_count, 2)  # only between attempts, not before the first

    def test_raises_write_failed_after_exhausting_all_attempts(self):
        errors = [avrdude_io.AvrdudeError("e%d" % i) for i in range(avrdude_io.WRITE_MAX_ATTEMPTS)]
        with mock.patch.object(avrdude_io, "_run_avrdude", side_effect=errors):
            with mock.patch.object(avrdude_io, "time"):
                with self.assertRaises(avrdude_io.AvrdudeWriteFailedError) as ctx:
                    avrdude_io._run_avrdude_with_retries("eeprom:w:x:r")
        self.assertEqual(ctx.exception.attempts, errors)
        self.assertIn("e%d" % (avrdude_io.WRITE_MAX_ATTEMPTS - 1), str(ctx.exception))

    def test_on_retry_called_before_each_retry_not_after_final_failure(self):
        errors = [avrdude_io.AvrdudeError("e%d" % i) for i in range(avrdude_io.WRITE_MAX_ATTEMPTS)]
        calls = []
        with mock.patch.object(avrdude_io, "_run_avrdude", side_effect=errors):
            with mock.patch.object(avrdude_io, "time"):
                with self.assertRaises(avrdude_io.AvrdudeWriteFailedError):
                    avrdude_io._run_avrdude_with_retries(
                        "eeprom:w:x:r", on_retry=lambda a, m, e: calls.append((a, m))
                    )
        # WRITE_MAX_ATTEMPTS attempts total, but on_retry only fires before a *retry* - one fewer call
        # than attempts, since the final exhausted attempt raises instead of retrying.
        self.assertEqual(calls, [(i, avrdude_io.WRITE_MAX_ATTEMPTS)
                                  for i in range(1, avrdude_io.WRITE_MAX_ATTEMPTS)])


class TestWriteFullEeprom(unittest.TestCase):
    def test_rejects_wrong_length(self):
        with self.assertRaises(ValueError):
            avrdude_io.write_full_eeprom(b"\x00" * (avrdude_io.layout.EEPROM_SIZE - 1))


class _FakeResult:
    def __init__(self, stdout=""):
        self.stdout = stdout
        self.stderr = ""
        self.returncode = 0


def _fake_avrdude(hfuse=0xD1, fail_on=None, fail_exc=None):
    """Returns (runner, calls). `runner` stands in for avrdude_io._run_avrdude_cmd: it records each
    call's args, answers a fuse read with three hex lines, and (if fail_on is a substring of the joined
    args) raises fail_exc (default AvrdudeError)."""
    calls = []

    def runner(extra_args, base=None):
        joined = " ".join(extra_args)
        calls.append(joined)
        if fail_on is not None and fail_on in joined:
            raise fail_exc or avrdude_io.AvrdudeError("simulated failure on %r" % joined)
        if "lfuse:r:-:h" in joined:
            return _FakeResult("0xFF\n0x%02X\n0xFD\n" % hfuse)
        return _FakeResult()

    return runner, calls


class TestReadFuses(unittest.TestCase):
    def test_parses_three_hex_values(self):
        runner, _ = _fake_avrdude(hfuse=0xD1)
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            self.assertEqual(avrdude_io.read_fuses(), (0xFF, 0xD1, 0xFD))

    def test_raises_on_unexpected_output(self):
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", return_value=_FakeResult("nope")):
            with self.assertRaises(avrdude_io.AvrdudeError):
                avrdude_io.read_fuses()


class TestBlankEepromViaFuseToggle(unittest.TestCase):
    _BLANK = b"\xff" * avrdude_io.layout.EEPROM_SIZE

    def test_clears_eesave_erases_then_restores_in_order(self):
        runner, calls = _fake_avrdude(hfuse=0xD1)
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            with mock.patch.object(avrdude_io, "read_full_eeprom", return_value=self._BLANK):
                avrdude_io.blank_eeprom_via_fuse_toggle()
        self.assertIn("-U hfuse:w:0xD9:m", calls)
        self.assertIn("-e", calls)
        self.assertIn("-U hfuse:w:0xD1:m", calls)
        self.assertLess(calls.index("-U hfuse:w:0xD9:m"), calls.index("-e"))
        self.assertLess(calls.index("-e"), calls.index("-U hfuse:w:0xD1:m"))

    def test_chip_erase_failure_still_restores_hfuse_then_raises(self):
        runner, calls = _fake_avrdude(hfuse=0xD1, fail_on="-e")
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            with mock.patch.object(avrdude_io, "read_full_eeprom", return_value=self._BLANK):
                with self.assertRaises(avrdude_io.EepromWipeError) as ctx:
                    avrdude_io.blank_eeprom_via_fuse_toggle()
        self.assertIn("-U hfuse:w:0xD1:m", calls)  # restore still happened
        self.assertIn("chip erase failed", str(ctx.exception))

    def test_hfuse_restore_failure_raises_eesave_warning(self):
        runner, _ = _fake_avrdude(hfuse=0xD1, fail_on="hfuse:w:0xD1:m")
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            with self.assertRaises(avrdude_io.EepromWipeError) as ctx:
                avrdude_io.blank_eeprom_via_fuse_toggle()
        self.assertIn("EESAVE", str(ctx.exception))
        self.assertIn("hfuse:w:0xD1:m", str(ctx.exception))

    def test_non_blank_readback_raises_with_offset(self):
        runner, _ = _fake_avrdude(hfuse=0xD1)
        dirty = b"\xff\xff\x42" + b"\xff" * (avrdude_io.layout.EEPROM_SIZE - 3)
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            with mock.patch.object(avrdude_io, "read_full_eeprom", return_value=dirty):
                with self.assertRaises(avrdude_io.EepromWipeError) as ctx:
                    avrdude_io.blank_eeprom_via_fuse_toggle()
        self.assertIn("0x0002", str(ctx.exception))

    def test_verify_readback_link_failure_is_best_effort_not_fatal(self):
        # all fuse ops + the erase succeed; only the post-erase readback hits a marginal ISP link
        runner, _ = _fake_avrdude(hfuse=0xD1)
        msgs = []
        with mock.patch.object(avrdude_io, "_run_avrdude_cmd", side_effect=runner):
            with mock.patch.object(avrdude_io, "read_full_eeprom",
                                    side_effect=avrdude_io.AvrdudeError("initialization failed")):
                avrdude_io.blank_eeprom_via_fuse_toggle(on_progress=msgs.append)  # must NOT raise
        self.assertTrue(any("could not read back" in m for m in msgs))


if __name__ == "__main__":
    unittest.main()
