#!/usr/bin/env python3
# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Broadcasts an MRBus/MRBee fast-clock 'T' packet over a USB-attached XBee radio, so a ProtoThrottle
throttle running this fork's firmware can display a running fast clock during bench testing - no physical
ISE mrb-fcm fast-clock-master hardware needed.

The wire format matches ISE's mrb-fcm.c exactly (mrb-fcm.c's transmit code, read from GitHub), and this
throttle already fully implements the receive side unchanged - see src/cst-time.c's processTimePacket().
So this tool only needs to *transmit*, periodically, the same 'T' packet mrb-fcm sends; no ack/retry logic
is needed, matching mrb-fcm's own fire-and-forget behavior. See README.md for the byte-layout reference and
the one-time on-device setup (COMM CFG -> TIME ADR, PREFS -> DISPLAY).

Has no EEPROM/config-slot dependency at all, so unlike cst_cfgnetwork.py this does not import
slot_codec.py/cst_eeprom_layout.py - only cnf_radio_io.py's radio transport (serial port, XBee API framing,
MRBus CRC16) is reused, via the same sys.path trick cst_cfgnetwork.py uses to reach a sibling tool
directory. Requires pyserial (`pip install pyserial`). Run with -h for usage.
"""

import argparse
import os
import sys
import time

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cst-cfgnetwork"))

import cnf_radio_io as radio  # noqa: E402

TIME_PKT_TYPE = ord('T')

# cst-time.h's TIME_FLAGS_DISP_* bits.
FLAG_DISP_FAST = 0x01
FLAG_DISP_FAST_HOLD = 0x02
FLAG_DISP_REAL_AMPM = 0x04
FLAG_DISP_FAST_AMPM = 0x08

DEFAULT_RATIO = 4.0
DEFAULT_INTERVAL_S = 2.0

RATIO_MIN = 0.1
RATIO_MAX = 99.9


def _auto_int(s):
    try:
        return int(s, 0)  # accepts plain decimal ("80") or 0x-prefixed hex ("0x50")
    except ValueError:
        raise argparse.ArgumentTypeError("invalid integer %r (decimal or 0x-prefixed hex)" % s)


def parse_hms(s):
    """Parses 'HH:MM' or 'HH:MM:SS' into a 0-86399 seconds-since-midnight int."""
    parts = s.split(":")
    if len(parts) not in (2, 3):
        raise argparse.ArgumentTypeError("expected HH:MM or HH:MM:SS, got %r" % s)
    try:
        parts = [int(p) for p in parts]
    except ValueError:
        raise argparse.ArgumentTypeError("expected HH:MM or HH:MM:SS, got %r" % s)
    hours, minutes = parts[0], parts[1]
    seconds = parts[2] if len(parts) == 3 else 0
    if not (0 <= hours < 24 and 0 <= minutes < 60 and 0 <= seconds < 60):
        raise argparse.ArgumentTypeError("time out of range: %r" % s)
    return hours * 3600 + minutes * 60 + seconds


def format_hms(total_seconds):
    total_seconds = int(total_seconds) % 86400
    h, rem = divmod(total_seconds, 3600)
    m, s = divmod(rem, 60)
    return "%02d:%02d:%02d" % (h, m, s)


def compute_fast_seconds(start_seconds, ratio, elapsed_real_s):
    """Pure function: the current fast-clock time (seconds since midnight, wrapping at 24h) given the
    fast time at start, the real:fast speed ratio, and elapsed real seconds since then. Kept independent
    of wall-clock time (the caller supplies elapsed_real_s from time.monotonic()) so it is unaffected by
    clock adjustments and is trivially unit-testable."""
    return (start_seconds + ratio * elapsed_real_s) % 86400


def build_time_payload(real_seconds, fast_seconds, ratio, flags):
    """Builds the 12-byte MRBus 'T' packet payload following the type byte (packet offsets 6-17 - see
    README.md's byte-layout table, ported unchanged from ISE's mrb-fcm.c). Date bytes (offsets 15-17) are
    sent as zero - this throttle's cst-time.c:processTimePacket() never reads them."""
    real_h, real_rem = divmod(int(real_seconds) % 86400, 3600)
    real_m, real_s = divmod(real_rem, 60)
    fast_h, fast_rem = divmod(int(fast_seconds) % 86400, 3600)
    fast_m, fast_s = divmod(fast_rem, 60)
    scale_raw = max(1, min(999, round(ratio * 10)))
    return bytes([
        real_h, real_m, real_s,
        flags,
        fast_h, fast_m, fast_s,
        (scale_raw >> 8) & 0xFF, scale_raw & 0xFF,
        0, 0, 0,  # year/month/day - unused by this throttle firmware
    ])


def _build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Broadcast an MRBus fast-clock 'T' packet over XBee for ProtoThrottle bench testing.")
    parser.add_argument("--port", required=True,
                         help="serial port of the USB-attached XBee, e.g. /dev/cu.usbserial-D30DUGOT")
    parser.add_argument("--my-addr", required=True, type=_auto_int,
                         help="MRBus address for this tool to send as (decimal or 0x-prefixed hex) - any "
                              "address not otherwise in use on the layout")
    parser.add_argument("--start", required=True, type=parse_hms,
                         help="fast-clock time as of now, HH:MM or HH:MM:SS")
    parser.add_argument("--ratio", type=float, default=DEFAULT_RATIO,
                         help="fast:real speed ratio, e.g. 4 = 4x real time (default %(default)s)")
    parser.add_argument("--interval", type=float, default=DEFAULT_INTERVAL_S,
                         help="seconds between broadcasts (default %(default)s)")
    parser.add_argument("--ampm", action="store_true", help="display 12-hour AM/PM instead of 24-hour")
    parser.add_argument("--hold", action="store_true",
                         help="broadcast the fast clock paused (HOLD) at --start instead of running")
    return parser


def main(argv=None):
    args = _build_arg_parser().parse_args(argv)

    if not (RATIO_MIN <= args.ratio <= RATIO_MAX):
        sys.exit("ERROR: --ratio must be between %s and %s" % (RATIO_MIN, RATIO_MAX))
    if args.interval <= 0:
        sys.exit("ERROR: --interval must be positive")

    flags = FLAG_DISP_FAST
    if args.hold:
        flags |= FLAG_DISP_FAST_HOLD
    if args.ampm:
        flags |= FLAG_DISP_FAST_AMPM | FLAG_DISP_REAL_AMPM

    try:
        link = radio.CnfRadioLink(port=args.port, my_addr=args.my_addr)
    except ImportError:
        sys.exit("ERROR: pyserial is required for this tool - install it with: pip install pyserial")
    except Exception as e:
        sys.exit("ERROR: could not open %s: %s" % (args.port, e))

    print("Broadcasting fast clock from %s as MRBus address 0x%02X (Ctrl+C to stop)"
          % (args.port, args.my_addr))
    t0 = time.monotonic()
    try:
        with link:
            while True:
                elapsed = 0.0 if args.hold else (time.monotonic() - t0)
                fast_seconds = compute_fast_seconds(args.start, args.ratio, elapsed)
                now = time.localtime()
                real_seconds = now.tm_hour * 3600 + now.tm_min * 60 + now.tm_sec
                payload = build_time_payload(real_seconds, fast_seconds, args.ratio, flags)
                link.send_raw(0xFF, TIME_PKT_TYPE, payload)
                tag = "HOLD" if args.hold else "%.1fx" % args.ratio
                print("FAST %s (%s)  real %s" % (format_hms(fast_seconds), tag, format_hms(real_seconds)))
                time.sleep(args.interval)
    except KeyboardInterrupt:
        print()
        print("Stopped.")


if __name__ == "__main__":
    main()
