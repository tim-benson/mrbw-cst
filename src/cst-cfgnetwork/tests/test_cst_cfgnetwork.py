"""Offline tests for cst_cfgnetwork.py's _is_cabbus_level_failure()/_explain_error() diagnostic pair -
distinguishes "ProtoThrottle Receiver reachable but doesn't speak the shared-CNF protocol" (e.g. stock ISE
firmware, or any build that predates cnf-store.c) from "not reachable at all" (wrong --cabbus-addr, dead
radio link, powered-off cabbus) after a BEGIN/reset timeout, using one supplementary unicast ping. This is
the first automated test for cst_cfgnetwork.py's own CLI-layer code (previously manually-verified only,
like cst_cfgtransfer.py's own CLI) - added because an inverted boolean here would be easy to miss and
would make the diagnostic actively misleading.

Run with: python3 -m unittest discover -s src/cst-cfgnetwork/tests -t src/cst-cfgnetwork
       or: cd src/cst-cfgnetwork && python3 -m unittest discover tests
"""

import os
import sys
import unittest
from unittest import mock

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import cnf_radio_io as radio  # noqa: E402
import cst_cfgnetwork as tool  # noqa: E402
from test_cnf_radio_io import FakeSerial  # noqa: E402

MY_ADDR = 0x30
CABBUS_ADDR = 0xD0


class PingOnlyResponder:
    """Models a cabbus running firmware that predates the shared-CNF store: reachable and correctly
    addressed (answers a basic 'A' ping), but with nothing in its packet dispatch that recognizes 'C'/'D'
    at all - exactly stock PktHandler()'s behavior for an unhandled packet type (silently dropped, no
    reply, no NACK - see CLAUDE.md's "Diagnose ... in cst_cfgnetwork.py" for how this was confirmed)."""

    def __init__(self, addr):
        self.addr = addr
        self.ping_count = 0

    def responder(self, dest, src, type_byte, payload):
        if dest != self.addr:
            return []
        if type_byte == radio.PING_TYPE_REQ:
            self.ping_count += 1
            return [(src, self.addr, radio.PING_TYPE_ACK, b"")]
        return []


class SilentResponder:
    """Models an unreachable cabbus - wrong --cabbus-addr, dead radio link, or powered off. Answers
    nothing at all, including the diagnostic ping."""

    def responder(self, dest, src, type_byte, payload):
        return []


def make_link(responder):
    return radio.CnfRadioLink(my_addr=MY_ADDR, cabbus_addr=CABBUS_ADDR,
                               serial_obj=FakeSerial(responder.responder))


def _fast_timeouts():
    return mock.patch.multiple(radio, CNF_CHUNK_TIMEOUT_S=0.02, CNF_CHUNK_MAX_RETRIES=1,
                                CNF_POLL_INTERVAL_S=0.005, SWEEP_PROBE_TIMEOUT_S=0.02)


class ExplainErrorTests(unittest.TestCase):

    def setUp(self):
        tool._diagnosed_cabbuses.clear()  # per-process cache - reset between tests

    def test_ping_only_responder_reports_unsupported_firmware(self):
        responder = PingOnlyResponder(CABBUS_ADDR)
        link = make_link(responder)
        with _fast_timeouts():
            try:
                link.pull_entry(0)
                self.fail("expected a CnfTimeoutError")
            except radio.CnfTimeoutError as e:
                self.assertTrue(tool._is_cabbus_level_failure(e))
                message = tool._explain_error(link, e)
        self.assertIn("ProtoThrottle Receiver is reachable but is not running ProtoThrottle X compatible "
                      "firmware", message)
        self.assertTrue(message.startswith("ERROR:"))

    def test_silent_responder_reports_no_reply_at_all(self):
        link = make_link(SilentResponder())
        with _fast_timeouts():
            try:
                link.pull_entry(0)
                self.fail("expected a CnfTimeoutError")
            except radio.CnfTimeoutError as e:
                self.assertTrue(tool._is_cabbus_level_failure(e))
                message = tool._explain_error(link, e)
        self.assertIn("ProtoThrottle Receiver not reachable", message)

    def test_diagnosis_is_cached_per_cabbus(self):
        responder = PingOnlyResponder(CABBUS_ADDR)
        link = make_link(responder)
        with _fast_timeouts():
            for _ in range(3):
                try:
                    link.pull_entry(0)
                    self.fail("expected a CnfTimeoutError")
                except radio.CnfTimeoutError as e:
                    tool._explain_error(link, e)
        self.assertEqual(responder.ping_count, 1)

    def test_reset_timeout_also_gets_diagnosed(self):
        # reset_table()'s step name is "reset", not "...BEGIN" - _is_cabbus_level_failure() has a
        # separate condition for it, since it's the same failure mode (an isolated request with nothing
        # yet established about the far end).
        responder = PingOnlyResponder(CABBUS_ADDR)
        link = make_link(responder)
        with _fast_timeouts():
            try:
                link.reset_table()
                self.fail("expected a CnfTimeoutError")
            except radio.CnfTimeoutError as e:
                self.assertTrue(tool._is_cabbus_level_failure(e))
                message = tool._explain_error(link, e)
        self.assertIn("ProtoThrottle Receiver is reachable but is not running ProtoThrottle X compatible "
                      "firmware", message)

    def test_non_begin_timeout_is_not_a_cabbus_level_failure(self):
        # A DATA-step timeout only happens after BEGIN already succeeded, so the far end is already known
        # to understand the protocol - callers should keep using str(error) for these, never call
        # _explain_error() (which would ping needlessly and produce a misleading whole-cabbus message).
        err = radio.CnfTimeoutError("push DATA@10")
        self.assertFalse(tool._is_cabbus_level_failure(err))

    def test_non_timeout_error_is_not_a_cabbus_level_failure(self):
        err = radio.CnfBusyError("cabbus is busy with a different in-progress transfer")
        self.assertFalse(tool._is_cabbus_level_failure(err))


class TargetEntryFromSourceTests(unittest.TestCase):
    """source.scope was renamed "shared_slot" -> "network_slot" as part of the network-terminology
    wording pass - _target_entry_from_source() must keep accepting the old value too, so JSON files
    already exported before the rename are not silently orphaned (their auto-entry-detection would
    otherwise start failing with "pass --entry explicitly" on every existing backup)."""

    def test_accepts_new_scope_value(self):
        entry = tool._target_entry_from_source({"scope": "network_slot", "entry": 5}, "f.json")
        self.assertEqual(entry, 5)

    def test_accepts_legacy_scope_value(self):
        entry = tool._target_entry_from_source({"scope": "shared_slot", "entry": 5}, "f.json")
        self.assertEqual(entry, 5)

    def test_rejects_unrelated_scope_value(self):
        with self.assertRaises(SystemExit):
            tool._target_entry_from_source({"scope": "slot", "entry": 5}, "f.json")


if __name__ == "__main__":
    unittest.main()
