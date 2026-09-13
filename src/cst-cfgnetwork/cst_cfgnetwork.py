#!/usr/bin/env python3
# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Export/import loco configurations to/from the ProtoThrottle Receiver (mrbw-cabbus) shared network Loco
Configuration (CNF) store (N01-N20), over a USB-attached XBee radio speaking the same 'C' (push)/'D'
(pull) protocol used by a real throttle for the on-device SAVE CNF / LOAD CNF menu N01-N20 entries - no
physical throttle or ISP access is needed.

Reuses slot_codec.py and cst_eeprom_layout.py from src/cst-cfgtransfer directly (see the sys.path setup
below) so JSON files are byte-for-byte interchangeable with the output from that tool - only
cnf_radio_io.py (the radio transport for this tool) is new. See README.md for the full field reference,
XBee setup, and safety workflow. Requires pyserial (`pip install pyserial`) - the one dependency not
needed by the ISP-only design of cst_cfgtransfer.py. Run with -h for usage.
"""

import argparse
import glob
import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "cst-cfgtransfer"))

import cst_eeprom_layout as layout  # noqa: E402
import slot_codec  # noqa: E402

import cnf_radio_io as radio

CNF_ENTRY_MIN = 1
CNF_ENTRY_MAX = radio.CNF_ENTRY_COUNT  # 20


def _auto_int(s):
    try:
        return int(s, 0)  # accepts plain decimal ("208") or 0x-prefixed hex ("0xD0")
    except ValueError:
        raise argparse.ArgumentTypeError("invalid integer %r (decimal or 0x-prefixed hex)" % s)


def _now_iso():
    import datetime
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def _format_loco_address(loco_addr_dict):
    suffix = "S" if loco_addr_dict["type"] == "short" else "L"
    return "%d%s" % (loco_addr_dict["address"], suffix)


def _decode_loco_word(word):
    # Same bit layout slot_codec.py's (private) _decode_loco_address uses, duplicated here in terms of
    # only the public layout.LOCO_ADDRESS_SHORT constant - `list`'s lightweight 2-byte peek never has a
    # full 128-byte slot to hand decode_slot(), so it can't reuse that function directly.
    if word & layout.LOCO_ADDRESS_SHORT:
        return {"address": word & ~layout.LOCO_ADDRESS_SHORT, "type": "short"}
    return {"address": word, "type": "long"}


def _addr_str_for_filename(raw_slot_bytes):
    if all(b == 0xFF for b in raw_slot_bytes):
        return "NONE"
    loco = slot_codec.decode_slot(raw_slot_bytes, source={})["loco_address"]
    return _format_loco_address(loco)


def _entry_label(n):
    return "N%02d" % n


def _validate_cli_entry(n):
    if not (CNF_ENTRY_MIN <= n <= CNF_ENTRY_MAX):
        sys.exit("ERROR: entry %d is out of range (must be %d-%d, i.e. N%02d-N%02d)"
                  % (n, CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX))


def _cabbus_dir(args):
    return os.path.join(args.out_dir, "cabbus-%d" % args.cabbus_addr)


def _open_link(args, need_cabbus=True):
    try:
        return radio.CnfRadioLink(port=args.port, my_addr=args.my_addr,
                                   cabbus_addr=args.cabbus_addr if need_cabbus else None)
    except ImportError:
        sys.exit("ERROR: pyserial is required for this tool - install it with: pip install pyserial")
    except Exception as e:
        sys.exit("ERROR: could not open %s: %s" % (args.port, e))


_diagnosed_cabbuses = {}  # cabbus_addr -> pinged_ok (bool); per-process cache so a run that hits the same
                           # BEGIN timeout many times (e.g. list's 20-entry loop) only probes once


def _is_cabbus_level_failure(error):
    """True for a BEGIN-step timeout, or a reset timeout - both are the first/only packet of an isolated
    exchange, with nothing yet established about the far end, and _explain_error() diagnoses these
    conclusively for the WHOLE receiver, not just the one entry that happened to trigger it. A caller
    mid-loop (list, export) should stop entirely on one of these rather than continue trying the
    remaining entries against the same unreachable/incompatible receiver. Every other error (busy,
    checksum, a later DATA/COMMIT timeout, etc.) already means the receiver DOES understand the protocol,
    so it's still just this one entry's problem - callers should keep using `str(error)` for those."""
    return isinstance(error, radio.CnfTimeoutError) and ("BEGIN" in error.step or "reset" == error.step)


def _explain_error(link, error):
    """For an error where _is_cabbus_level_failure() is true, does one supplementary unicast ping to
    distinguish a receiver that's reachable but running firmware that doesn't understand the shared-CNF
    protocol (e.g. predates this feature) from one that isn't reachable at all, and returns a complete,
    ready-to-print multi-paragraph ERROR message for whichever case applies. Callers should only call this
    when _is_cabbus_level_failure(error) is true."""
    addr = link.cabbus_addr
    if addr not in _diagnosed_cabbuses:
        _diagnosed_cabbuses[addr] = link.probe_address(addr, radio.SWEEP_PROBE_TIMEOUT_S, retries=1)
    if _diagnosed_cabbuses[addr]:
        return (
            "ERROR: ProtoThrottle Receiver is reachable but is not running ProtoThrottle X compatible "
            "firmware.\n\n"
            "Please upgrade the Receiver firmware and try again."
        )
    return (
        "ERROR: ProtoThrottle Receiver not reachable.\n\n"
        "Please check --cabbus-addr, the radio link, and that the ProtoThrottle Receiver is powered on."
    )


def _pull_or_none(link, n):
    """Pulls entry n (1-20), returning None for an EMPTY entry, or exiting with a clear message for any
    other radio failure (busy/timeout/checksum) - those aren't recoverable by treating them as "empty"."""
    try:
        raw, _layout_version = link.pull_entry(n - 1)
        return raw
    except radio.CnfEmptyError:
        return None
    except radio.CnfRadioError as e:
        if _is_cabbus_level_failure(e):
            sys.exit(_explain_error(link, e))
        sys.exit("ERROR: could not read %s from the ProtoThrottle Receiver: %s" % (_entry_label(n), e))


def _write_json_file(path, d):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(d, f, indent=2)
        f.write("\n")


def _load_json_file(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


def _encode_or_exit(d, path, base=None, allow_missing=False):
    try:
        return slot_codec.encode_slot(d, base=base, allow_missing=allow_missing)
    except slot_codec.SlotValidationError as e:
        print("ERROR: %s failed validation:" % path, file=sys.stderr)
        for err in e.errors:
            print("  - %s" % err, file=sys.stderr)
        sys.exit(1)


def _print_dict_diff(old, new, prefix=""):
    keys = sorted(set(old.keys()) | set(new.keys()))
    for key in keys:
        if key in ("source", "schema_version"):
            continue
        old_v = old.get(key, "<missing>")
        new_v = new.get(key, "<missing>")
        if isinstance(old_v, dict) and isinstance(new_v, dict):
            _print_dict_diff(old_v, new_v, prefix + key + ".")
        elif old_v != new_v:
            print("  %s%s: %r -> %r" % (prefix, key, old_v, new_v))


# --- discover ---

def cmd_discover(args):
    link = _open_link(args, need_cabbus=False)
    try:
        print("Broadcasting for %.1fs..." % args.wait)
        if not args.no_sweep:
            # mrbw-cabbus deliberately ignores broadcast pings, so a broadcast alone can never find one -
            # the sweep phase individually probes every base-station address (0xD0-0xEF) to catch it/them.
            # Deliberately not capped at one hit: a layout can have more than one cabbus.
            n_addrs = radio.MRBUS_BASE_ADDR_MAX - radio.MRBUS_BASE_ADDR_MIN + 1
            print("Then sweeping %d base-station addresses (up to ~%.1fs)..."
                  % (n_addrs, n_addrs * args.sweep_timeout * (1 + radio.SWEEP_PROBE_RETRIES)))
        found = link.discover_nodes(wait_s=args.wait, sweep_base_addrs=not args.no_sweep,
                                     sweep_timeout_s=args.sweep_timeout)
    finally:
        link.close()

    if not found:
        print("No MRBus nodes responded.")
        return
    print()
    for addr, method in found:
        if method == "sweep":
            tag = "ProtoThrottle Receiver (confirmed via direct probe)"
        elif layout.MRBUS_BASE_ADDR_MIN <= addr <= layout.MRBUS_BASE_ADDR_MAX:
            tag = "likely ProtoThrottle Receiver (base station address range)"
        elif layout.MRBUS_DEV_ADDR_MIN <= addr <= layout.MRBUS_DEV_ADDR_MAX:
            tag = "likely throttle"
        else:
            tag = "unrecognized address range"
        print("  0x%02X (%d)  %s" % (addr, addr, tag))


# --- sniff ---

_THROTTLE_STATUS_FLAGS = (
    (radio.THROTTLE_STATUS_EMERGENCY, "EMRG"),
    (radio.THROTTLE_STATUS_ALL_STOP, "ALLSTOP"),
    (radio.THROTTLE_STATUS_ALERTER, "ALERTER"),
    (radio.THROTTLE_STATUS_SLEEP, "SLEEP"),
)


def _format_cst_status(src, dest, s):
    loco = _decode_loco_word(s["loco_raw"])
    loco_str = "%d%s" % (loco["address"], "S" if loco["type"] == "short" else "L")
    active = [n for n in range(29) if s["function_mask"] & (1 << n)]
    fns = ",".join("F%d" % n for n in active) if active else "-"
    flags = "|".join(name for bit, name in _THROTTLE_STATUS_FLAGS if s["throttle_status"] & bit) or "-"
    return ("0x%02X->0x%02X  loco=%-7s %s  step=%-3d  F:%-32s  %-16s  %.1fV"
            % (src, dest, loco_str, "FWD" if s["forward"] else "REV",
               s["speed_step"], fns, flags, s["battery_decivolts"] / 10.0))


def cmd_sniff(args):
    import time
    link = _open_link(args, need_cabbus=False)
    want = {radio.CST_STATUS_TYPE} if (args.status_only or args.changes) else None
    if want is not None and args.cnf:
        want = want | radio.CNF_PACKET_TYPES   # keep the 'S' filter but let CNF traffic through too
    last = {}  # src -> last (fnmask, step, fwd, status) key, for --changes
    print("Listening on %s%s - Ctrl-C to stop\n"
          % (args.port, "" if args.seconds is None else " for %.0fs" % args.seconds))
    try:
        for dest, src, type_byte, payload in link.sniff(packet_types=want, stop_after_s=args.seconds):
            ts = time.strftime("%H:%M:%S")
            s = radio.decode_cst_status(payload) if type_byte == radio.CST_STATUS_TYPE else None
            if s is not None:
                if args.changes:
                    key = (s["function_mask"], s["speed_step"], s["forward"], s["throttle_status"])
                    if last.get(src) == key:
                        continue
                    last[src] = key
                print("%s  %s" % (ts, _format_cst_status(src, dest, s)))
            elif args.cnf and type_byte in radio.CNF_PACKET_TYPES:
                print("%s  0x%02X->0x%02X  %s"
                      % (ts, src, dest, radio.format_cnf_packet(type_byte, payload)))
            elif not args.changes and not args.status_only:
                ch = chr(type_byte) if 32 <= type_byte <= 126 else "?"
                body = " ".join("%02X" % b for b in payload)
                print("%s  0x%02X->0x%02X  '%s'(0x%02X)  %s" % (ts, src, dest, ch, type_byte, body))
    except KeyboardInterrupt:
        print()
    finally:
        link.close()


# --- list ---

def _version_tag(entry_version):
    """Per-row version-mismatch note for `list` - "" when the entry's reported layout_version is unset
    (never pinned) or matches this tool's own SUPPORTED_LAYOUT_VERSION, else a visible tag."""
    if entry_version in (radio.CNF_VERSION_UNSET, layout.SUPPORTED_LAYOUT_VERSION):
        return ""
    return "  (CNF format v%d, tool supports v%d)" % (entry_version, layout.SUPPORTED_LAYOUT_VERSION)


def cmd_list(args):
    link = _open_link(args)
    try:
        entries = [args.entry] if args.entry else list(range(CNF_ENTRY_MIN, CNF_ENTRY_MAX + 1))
        for n in entries:
            _validate_cli_entry(n)
        for n in entries:
            label = _entry_label(n)
            try:
                addr_bytes, entry_version = link.peek_loco_address(n - 1)
                word = addr_bytes[0] | (addr_bytes[1] << 8)
                print("  %s  %s%s" % (label, _format_loco_address(_decode_loco_word(word)),
                                       _version_tag(entry_version)))
            except radio.CnfEmptyError:
                print("  %s  NONE" % label)
            except radio.CnfBusyError:
                print("  %s  BUSY" % label)
            except radio.CnfRadioError as e:
                if _is_cabbus_level_failure(e):
                    sys.exit(_explain_error(link, e))
                print("  %s  FAIL (%s)" % (label, e))
    finally:
        link.close()


# --- export ---

def _run_export_to_dir(link, cabbus_addr, out_dir, entries):
    """Pulls `entries` (1-20) from `link` and writes each to its own JSON file in `out_dir`. Shared by
    `export`, the pre-upgrade backup step inside `import` (see cmd_import()), and `reset-cabbus`'s default
    backup - all three need "back up the whole table to a folder," just triggered differently. Returns
    True if every entry was read successfully (False if any were skipped due to a radio failure)."""
    os.makedirs(out_dir, exist_ok=True)

    manifest = []
    had_failure = False
    for n in entries:
        label = _entry_label(n)
        try:
            raw, _layout_version = link.pull_entry(n - 1)
        except radio.CnfEmptyError:
            raw = bytes([0xFF] * layout.CONFIG_SIZE)
        except radio.CnfRadioError as e:
            if _is_cabbus_level_failure(e):
                sys.exit(_explain_error(link, e))
            print("  %s  FAILED (%s) - skipped" % (label, e))
            had_failure = True
            continue

        is_empty = all(b == 0xFF for b in raw)
        source = {"scope": "network_slot", "entry": n, "cabbus_addr": "0x%02X" % cabbus_addr,
                  "transport": "radio", "exported_at": _now_iso(), "empty": is_empty}
        d = slot_codec.decode_slot(raw, source)
        addr_str = _addr_str_for_filename(raw)
        fname = "%s_addr%s.json" % (label, addr_str)
        path = os.path.join(out_dir, fname)
        _write_json_file(path, d)
        manifest.append((label, addr_str, fname))

    print("Cabbus address: %d (0x%02X)  ->  %s" % (cabbus_addr, cabbus_addr, out_dir))
    print()
    for label, addr_str, fname in manifest:
        print("  %-5s loco %-10s %s" % (label, addr_str, fname))
    return not had_failure


def _check_table_version_or_exit(link, entries):
    """Peeks the table-wide pinned layout_version (via any one valid entry - the pin isn't per-entry) and
    aborts with a clear message if it's a real (non-UNSET), non-matching version - same "refuse rather
    than silently misdecode" posture as cst_cfgtransfer.py's own device-level guard. Returns the peeked
    version (informational - export doesn't otherwise need it, but a caller like cmd_import's upgrade
    check does)."""
    try:
        pinned_version = link.peek_pinned_version(entries[0] - 1)
    except radio.CnfRadioError as e:
        if _is_cabbus_level_failure(e):
            sys.exit(_explain_error(link, e))
        sys.exit("ERROR: could not check the CNF format version pinned on the ProtoThrottle Receiver: %s"
                  % e)
    if pinned_version != radio.CNF_VERSION_UNSET and pinned_version != layout.SUPPORTED_LAYOUT_VERSION:
        sys.exit(
            "ERROR: the shared network CNF table on the ProtoThrottle Receiver is pinned to CNF format version "
            "%d, but this source checkout (src/cst-eeprom.h) is version %d - check out the mrbw-cst revision "
            "matching the firmware that pinned the table before touching this ProtoThrottle Receiver."
            % (pinned_version, layout.SUPPORTED_LAYOUT_VERSION))
    return pinned_version


def cmd_export(args):
    link = _open_link(args)
    try:
        entries = sorted(set(args.entry)) if args.entry else list(range(CNF_ENTRY_MIN, CNF_ENTRY_MAX + 1))
        for n in entries:
            _validate_cli_entry(n)
        _check_table_version_or_exit(link, entries)
        ok = _run_export_to_dir(link, args.cabbus_addr, _cabbus_dir(args), entries)
    finally:
        link.close()
    if not ok:
        sys.exit(1)


# --- import ---

def _target_entry_from_source(source, path):
    if source.get("scope") not in ("network_slot", "shared_slot"):  # shared_slot: pre-rename exports
        sys.exit("ERROR: %s: not a shared-network-slot export (source.scope=%r) - pass --entry explicitly if "
                  "you are sure this is the right file" % (path, source.get("scope")))
    entry = source.get("entry")
    if not isinstance(entry, int):
        sys.exit("ERROR: %s: source.entry missing or invalid - pass --entry explicitly" % path)
    return entry


def _gather_import_items(args):
    """Returns a list of (entry, path, loaded_dict). Validates every file up front (against a zero-filled
    base, purely to check for errors before touching the radio) - the real, base-preserving encode
    happens later in cmd_import() once each entry's current bytes are known.

    --dir batches automatically skip any file whose source.empty is true (a NONE/never-configured stub
    from export) - such a file isn't re-importable as-is (decode_slot() faithfully reproduces a few
    fields, like force_functions' bits 29-31, that encode_slot() then rejects - the same asymmetry
    cst_cfgtransfer.py's shared codec would show for a truly blank local slot), and since it represents
    "nothing was ever configured here," pushing it would be a no-op anyway even if it could be encoded. A
    single explicitly-named file (no --dir) is never auto-skipped - if you point directly at one NONE
    stub, you get the real validation error, not silence, since that's the one thing you asked for.
    """
    items = []
    if args.dir:
        paths = sorted(glob.glob(os.path.join(args.dir, "*.json")))
        if not paths:
            sys.exit("ERROR: no *.json files found in %s" % args.dir)
        skipped_empty = []
        for path in paths:
            d = _load_json_file(path)
            if d.get("source", {}).get("empty"):
                skipped_empty.append(path)
                continue
            entry = _target_entry_from_source(d.get("source", {}), path)
            _validate_cli_entry(entry)
            _encode_or_exit(d, path, allow_missing=args.import_old)  # validate only; result discarded
            items.append((entry, path, d))
        if skipped_empty:
            print("Skipping %d NONE/never-configured source file(s) (not re-importable as-is):"
                  % len(skipped_empty))
            for path in skipped_empty:
                print("  - %s" % path)
            print()
        if not items:
            sys.exit("ERROR: every *.json file in %s was a NONE/never-configured stub - nothing to import"
                      % args.dir)
    else:
        d = _load_json_file(args.file)
        entry = args.entry if args.entry is not None else _target_entry_from_source(d.get("source", {}),
                                                                                      args.file)
        _validate_cli_entry(entry)
        _encode_or_exit(d, args.file, allow_missing=args.import_old)
        items.append((entry, args.file, d))

    seen = {}
    for entry, path, _d in items:
        if entry in seen:
            sys.exit("ERROR: both %s and %s target %s - aborting, nothing sent"
                      % (seen[entry], path, _entry_label(entry)))
        seen[entry] = path
    return items


def _check_push_safe_version_or_exit(link, entry0):
    """Peeks the table-wide version pin (via entry0, any one target entry - the pin isn't per-entry) and
    refuses outright unless it's already at this tool's own SUPPORTED_LAYOUT_VERSION. Used by every
    command that pushes (import, wipe-entry) - NOT the lenient _check_table_version_or_exit() that
    export/list use, since a push is the one thing that can advance the pin.

    Only real throttle firmware is allowed to establish or advance the pin - see CLAUDE.md's "Shared
    network CNF store" section ("Only real throttle firmware may advance the pin") for why. This tool's
    own SUPPORTED_LAYOUT_VERSION can drift ahead of every physical throttle from nothing more than a
    `git pull`, so letting it advance the pin risks locking every real throttle out of N01-N20 (both push
    and pull refused) until every one of them is individually reflashed to catch up - a bad failure mode
    for a PC tool to be able to trigger by accident."""
    try:
        pinned_version = link.peek_pinned_version(entry0)
    except radio.CnfRadioError as e:
        if _is_cabbus_level_failure(e):
            sys.exit(_explain_error(link, e))
        sys.exit("ERROR: could not check the CNF format version pinned on the ProtoThrottle Receiver: "
                  "%s" % e)

    if pinned_version == radio.CNF_VERSION_UNSET:
        sys.exit(
            "ERROR: ProtoThrottle Receiver has not yet been provisioned with the new Locomotive "
            "Configuration Version system.\n\n"
            "Please save a Loco Configuration to the ProtoThrottle Receiver from a ProtoThrottle "
            "running the newer ProtoThrottle X firmware first (SAVE CNF, choose an N01-N20 "
            "entry).\n\n"
            "Then re-run this - it will succeed once the Locomotive Configuration versions match."
        )
    if pinned_version < layout.SUPPORTED_LAYOUT_VERSION:
        sys.exit(
            "ERROR: ProtoThrottle Receiver has Locomotive Configuration Version %d and this tool "
            "expects Version %d.\n\n"
            "Please save a Loco Configuration to the ProtoThrottle Receiver from a ProtoThrottle "
            "running the newer ProtoThrottle X firmware first (SAVE CNF, choose an N01-N20 "
            "entry).\n\n"
            "Then re-run this - it will succeed once the Locomotive Configuration versions match." %
            (pinned_version, layout.SUPPORTED_LAYOUT_VERSION)
        )


def cmd_import(args):
    if bool(args.dir) == bool(args.file):
        sys.exit("ERROR: pass exactly one of a JSON file or --dir")
    if args.dir and args.entry is not None:
        sys.exit("ERROR: --entry is not valid with --dir - the source.entry embedded in each file is used "
                  "instead")

    items = _gather_import_items(args)
    link = _open_link(args)
    try:
        _check_push_safe_version_or_exit(link, items[0][0] - 1)

        if args.import_old:
            notices = []
            for _entry, path, d in items:
                for notice in slot_codec.describe_missing_fields(d):
                    notices.append("%s: %s" % (path, notice))
            if notices:
                print("--import-old: the following fields are missing from their source file and will")
                print("be defaulted (not silently - listed here so you can review before confirming):")
                for notice in notices:
                    print("  - %s" % notice)
                print()

        encoded_items = []  # (entry, path, d, encoded, old_bytes_or_None)
        for entry, path, d in items:
            label = _entry_label(entry)
            old_bytes = _pull_or_none(link, entry)
            encoded = _encode_or_exit(d, path, base=old_bytes, allow_missing=args.import_old)
            old_summary = _addr_str_for_filename(old_bytes) if old_bytes is not None else "NONE"
            new_summary = _format_loco_address(d["loco_address"])
            print("  %-5s loco %-10s -> %-10s  <- %s" % (label, old_summary, new_summary, path))
            encoded_items.append((entry, path, d, encoded, old_bytes))

        if args.dry_run:
            print()
            print("--dry-run: showing field-level changes, nothing sent over the radio.")
            for entry, path, d, encoded, old_bytes in encoded_items:
                empty_stub = bytes([0xFF] * layout.CONFIG_SIZE)
                old_d = slot_codec.decode_slot(old_bytes if old_bytes is not None else empty_stub,
                                                source={})
                # Compares against the actually-encoded-then-redecoded result, not the raw loaded `d` -
                # with --import-old, a defaulted field would otherwise misleadingly show as <missing>
                # here instead of the real value it's about to become.
                new_d = slot_codec.decode_slot(bytes(encoded), source={})
                print()
                print("=== %s (%s) ===" % (path, _entry_label(entry)))
                _print_dict_diff(old_d, new_d)
            return

        if not args.yes:
            answer = input("\nPush the above to the ProtoThrottle Receiver now? [y/N] ").strip().lower()
            if answer != "y":
                print("Aborted, nothing sent.")
                return

        print("\nPushing...")
        for entry, path, d, encoded, _old_bytes in encoded_items:
            try:
                link.push_entry(entry - 1, bytes(encoded), layout.SUPPORTED_LAYOUT_VERSION)
            except radio.CnfVersionMismatchError as e:
                sys.exit("ERROR: push failed for %s (%s): %s\nThe CNF format version of this source "
                          "checkout (%d) is older than the version pinned on the table - check out the "
                          "mrbw-cst revision matching the firmware that pinned it before pushing here."
                          % (path, _entry_label(entry), e, layout.SUPPORTED_LAYOUT_VERSION))
            except radio.CnfRadioError as e:
                if _is_cabbus_level_failure(e):
                    sys.exit(_explain_error(link, e))
                sys.exit("ERROR: push failed for %s (%s): %s\n(any entries pushed earlier in this run "
                          "were written; this one and any after it were not - re-run for the rest once "
                          "the problem is resolved)" % (path, _entry_label(entry), e))
            print("  %s: pushed" % _entry_label(entry))

        if not args.verify:
            print("Done.")
            return

        print("Verifying...")
        ok = True
        for entry, path, d, encoded, _old_bytes in encoded_items:
            reread = _pull_or_none(link, entry)
            if reread != bytes(encoded):
                print("  FAIL: %s (%s) did not verify" % (path, _entry_label(entry)))
                ok = False
            else:
                print("  PASS: %s (%s)" % (path, _entry_label(entry)))
        if not ok:
            sys.exit(1)
        print("Done.")
    finally:
        link.close()


# --- wipe-entry ---

def cmd_wipe_entry(args):
    """Blanks one or more shared network entries (N01-N20) to raw 0xFF, via the same push_entry() an
    import uses - just with a fixed all-0xFF payload instead of an encoded JSON file. The cabbus wire
    protocol has no per-entry delete (only a whole-table CNF_SUBTYPE_RESET), so the wiped entry stays
    "occupied" in the cabbus's own key bookkeeping - a pull right after this succeeds and returns the
    all-0xFF bytes rather than raising CnfEmptyError. That's not a problem in practice: every place this
    tool displays loco content already collapses an all-0xFF payload to "NONE", the same string used for
    a truly-never-occupied entry (see _addr_str_for_filename()/cmd_export()'s is_empty flag) - so a wiped
    entry reads identically to one that was never configured everywhere except the cabbus's internal
    bookkeeping."""
    entries = sorted(set(args.entry))
    for n in entries:
        _validate_cli_entry(n)

    link = _open_link(args)
    try:
        _check_push_safe_version_or_exit(link, entries[0] - 1)

        blank = bytes([0xFF] * layout.CONFIG_SIZE)
        targets = []  # (entry, old_summary)
        for n in entries:
            label = _entry_label(n)
            old_bytes = _pull_or_none(link, n)
            old_summary = _addr_str_for_filename(old_bytes) if old_bytes is not None else "NONE"
            print("  %-5s loco %-10s -> %-10s" % (label, old_summary, "NONE"))
            targets.append(n)

        if args.dry_run:
            print()
            print("--dry-run: nothing sent over the radio.")
            return

        if not args.yes:
            print()
            print("Back up first, e.g.:  python3 cst_cfgnetwork.py export --entry N --out-dir "
                  "~/protothrottle-backups/")
            answer = input("\nWipe the above entry/entries now? [y/N] ").strip().lower()
            if answer != "y":
                print("Aborted, nothing sent.")
                return

        print("\nPushing...")
        for n in targets:
            try:
                link.push_entry(n - 1, blank, layout.SUPPORTED_LAYOUT_VERSION)
            except radio.CnfVersionMismatchError as e:
                sys.exit("ERROR: wipe failed for %s: %s\nThe CNF format version of this source checkout "
                          "(%d) is older than the version pinned on the table - check out the mrbw-cst "
                          "revision matching the firmware that pinned it before wiping here."
                          % (_entry_label(n), e, layout.SUPPORTED_LAYOUT_VERSION))
            except radio.CnfRadioError as e:
                if _is_cabbus_level_failure(e):
                    sys.exit(_explain_error(link, e))
                sys.exit("ERROR: wipe failed for %s: %s\n(any entries wiped earlier in this run stayed "
                          "wiped; this one and any after it were not - re-run for the rest once the "
                          "problem is resolved)" % (_entry_label(n), e))
            print("  %s: wiped" % _entry_label(n))

        if not args.verify:
            print("Done.")
            return

        print("Verifying...")
        ok = True
        for n in targets:
            reread = _pull_or_none(link, n)
            if reread != blank:
                print("  FAIL: %s did not verify" % _entry_label(n))
                ok = False
            else:
                print("  PASS: %s" % _entry_label(n))
        if not ok:
            sys.exit(1)
        print("Done.")
    finally:
        link.close()


# --- reset-cabbus ---

def cmd_reset_cabbus(args):
    link = _open_link(args)
    try:
        if not args.skip_backup:
            if not args.out_dir:
                sys.exit("ERROR: reset-cabbus backs up by default - pass --out-dir, or --skip-backup to "
                          "proceed without one")
            print("Backing up all 20 entries to %s before resetting..." % _cabbus_dir(args))
            _run_export_to_dir(link, args.cabbus_addr, _cabbus_dir(args),
                                list(range(CNF_ENTRY_MIN, CNF_ENTRY_MAX + 1)))
            print()

        if not args.yes:
            answer = input("This wipes all 20 shared network entries and clears the version pin on the table. "
                            "Continue? [y/N] ").strip().lower()
            if answer != "y":
                print("Aborted, nothing reset.")
                return

        try:
            link.reset_table()
        except radio.CnfRadioError as e:
            if _is_cabbus_level_failure(e):
                sys.exit(_explain_error(link, e))
            sys.exit("ERROR: reset failed: %s" % e)
        print("Table reset - all 20 entries wiped, version pin cleared.")
    finally:
        link.close()


# --- argument parsing ---

def _add_common_radio_args(p, need_cabbus=True):
    p.add_argument("--port", required=True,
                    help="Serial device for the USB XBee adapter, e.g. /dev/tty.usbserial-XXXX")
    p.add_argument("--my-addr", required=True, type=_auto_int,
                    help="The MRBus address for this tool (throttle range 0x30-0x49) - pick one not "
                         "already used by a real throttle on the layout")
    if need_cabbus:
        p.add_argument("--cabbus-addr", required=True, type=_auto_int,
                        help="The MRBus address of the target ProtoThrottle Receiver (base-station range "
                             "0xD0-0xEF) - run `discover` first if you do not already know it")


def build_parser():
    parser = argparse.ArgumentParser(prog="cst_cfgnetwork.py", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    disc = sub.add_parser("discover", help="Find MRBus nodes: a broadcast ping (finds throttles) plus a "
                                            "base-station address sweep (finds ProtoThrottle Receiver "
                                            "units, which ignore broadcast)")
    _add_common_radio_args(disc, need_cabbus=False)
    disc.add_argument("--wait", type=float, default=2.0,
                       help="Seconds to listen for broadcast-ping replies (default: 2.0)")
    disc.add_argument("--no-sweep", action="store_true",
                       help="Skip the base-station address sweep - faster, but will not find any "
                            "ProtoThrottle Receiver (ProtoThrottle Receiver units do not answer the "
                            "broadcast ping alone)")
    disc.add_argument("--sweep-timeout", type=float, default=radio.SWEEP_PROBE_TIMEOUT_S,
                       help="Seconds to wait per address during the sweep (default: %.1f)"
                            % radio.SWEEP_PROBE_TIMEOUT_S)
    disc.set_defaults(func=cmd_discover)

    snf = sub.add_parser("sniff",
                          help="Passively watch radio traffic and decode ProtoThrottle status packets "
                               "(loco, direction, speed step, function mask, battery), and with --cnf the "
                               "shared-CNF 'C'/'D' transfer packets too - read-only, never transmits")
    _add_common_radio_args(snf, need_cabbus=False)
    snf.add_argument("--status-only", action="store_true",
                      help="Only ProtoThrottle 'S' status packets (skip pings, version announcements, etc.)")
    snf.add_argument("--changes", action="store_true",
                      help="'S' status packets, but print only when a given throttle's decoded state "
                           "(function mask / speed / direction / status flags) actually changes - best "
                           "for spotting a transient glitch such as a light-knob flicker")
    snf.add_argument("--cnf", action="store_true",
                      help="Also decode shared-CNF 'C'/'c'/'D'/'d' push/pull packets "
                           "(BEGIN/DATA/COMMIT/DONE, entry, offset, status) instead of showing them as "
                           "raw hex - use this while a throttle or this tool is doing a SAVE/LOAD CNF. "
                           "Combines with --status-only/--changes (shows 'S' plus CNF, nothing else).")
    snf.add_argument("--seconds", type=float,
                      help="Stop automatically after this many seconds (default: run until Ctrl-C)")
    snf.set_defaults(func=cmd_sniff)

    lst = sub.add_parser("list", help="Peek loco addresses for all (or one) shared network entries, no full pull")
    _add_common_radio_args(lst)
    lst.add_argument("--entry", type=int, help="Only this entry (%d-%d, i.e. N%02d-N%02d)"
                                                 % (CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX))
    lst.set_defaults(func=cmd_list)

    exp = sub.add_parser(
        "export",
        help="Pull shared network entries from the ProtoThrottle Receiver to JSON file(s) - all 20 by default",
        description="Pull shared network entries from the ProtoThrottle Receiver to JSON file(s). Omit --entry "
                     "to export ALL %d entries (N%02d-N%02d) in one run - this is the default, not an "
                     "opt-in flag." % (CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX))
    _add_common_radio_args(exp)
    exp.add_argument("--out-dir", required=True, type=os.path.expanduser,
                      help="Parent backup directory; a cabbus-<addr>/ subfolder is created/reused")
    exp.add_argument("--entry", type=int, action="append",
                      help="Export only this numbered shared network entry (%d-%d, i.e. N%02d-N%02d); "
                           "repeatable. Omit entirely to export all %d entries in one run (the default)."
                           % (CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MAX))
    exp.set_defaults(func=cmd_export)

    imp = sub.add_parser(
        "import",
        help="Push JSON file(s) to shared network entries on the ProtoThrottle Receiver - use --dir for all "
             "slots at once",
        description="Push JSON file(s) to shared network entries on the ProtoThrottle Receiver. To import many "
                     "or all slots in one run, point --dir at a folder of files (e.g. one produced by "
                     "export) - each goes to its own embedded source.entry, and any NONE or "
                     "never-configured stub files in that folder are automatically skipped (they are not "
                     "meaningfully re-importable - see README). Refuses outright if this push would "
                     "establish or advance the shared network CNF version pin on the ProtoThrottle Receiver - "
                     "only real throttle firmware is allowed to do that (see README); push from a real "
                     "throttle first, then re-run this import.")
    _add_common_radio_args(imp)
    imp.add_argument("file", nargs="?", type=os.path.expanduser,
                      help="JSON file to import (omit if using --dir)")
    imp.add_argument("--dir", type=os.path.expanduser,
                      help="Import every *.json in this directory in one run (use this for "
                                    "importing many/all slots at once) - each goes to its own embedded "
                                    "source.entry. NONE/never-configured stub files are skipped "
                                    "automatically.")
    imp.add_argument("--entry", type=int, help="Target this numbered shared network entry (%d-%d, i.e. "
                                                 "N%02d-N%02d) - not valid together with --dir, which "
                                                 "uses the source.entry embedded in each file instead"
                                                 % (CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX))
    imp.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    imp.add_argument("--dry-run", action="store_true",
                      help="Show what would change without pushing anything over the radio")
    imp.add_argument("--verify", action="store_true",
                      help="Pull each entry back and diff after pushing, beyond the CRC gate already "
                           "built into COMMIT")
    imp.add_argument("--import-old", action="store_true",
                      help="Allow a JSON file exported under an older/smaller schema (e.g. a backup taken "
                           "before a CNF format bump added a new field) - fields missing from an "
                           "existing functions/speed/options object are defaulted instead of rejected. A "
                           "field that is present but invalid is still always rejected, and a whole "
                           "missing category (e.g. no \"functions\" object at all) still always fails.")
    imp.set_defaults(func=cmd_import)

    wpe = sub.add_parser(
        "wipe-entry",
        help="Blank one or more shared network entries (N01-N20) to raw 0xFF",
        description="Blanks one or more shared network entries to raw 0xFF - the same state as an entry "
                     "that was never configured. The cabbus wire protocol has no per-entry delete, so "
                     "this pushes an all-0xFF payload via the same mechanism as import; every listing/"
                     "export in this tool already shows an all-0xFF entry as NONE, same as a truly-empty "
                     "one. Refuses outright under the same version-pin conditions as import (see "
                     "README) - only real throttle firmware may establish or advance the pin.")
    _add_common_radio_args(wpe)
    wpe.add_argument("--entry", type=int, action="append", required=True,
                      help="Numbered shared network entry (%d-%d, i.e. N%02d-N%02d) to wipe; repeatable"
                           % (CNF_ENTRY_MIN, CNF_ENTRY_MAX, CNF_ENTRY_MIN, CNF_ENTRY_MAX))
    wpe.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    wpe.add_argument("--dry-run", action="store_true",
                      help="Show what would change without pushing anything over the radio")
    wpe.add_argument("--verify", action="store_true",
                      help="Pull each entry back and confirm it reads all-0xFF after wiping")
    wpe.set_defaults(func=cmd_wipe_entry)

    rst = sub.add_parser(
        "reset-cabbus",
        help="Wipe the entire shared network CNF table and clear the version pin",
        description="Deliberately wipes all 20 shared network entries on the ProtoThrottle Receiver and clears "
                     "the table-wide version pin, independent of any version comparison - use this to "
                     "recover from a bad state or to get a clean slate without needing an actual "
                     "higher-version push to trigger it. Backs up all 20 entries first by default.")
    _add_common_radio_args(rst)
    rst.add_argument("--out-dir", type=os.path.expanduser,
                      help="Back up all 20 entries here first (required unless --skip-backup)")
    rst.add_argument("--skip-backup", action="store_true", help="Proceed without backing up first")
    rst.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    rst.set_defaults(func=cmd_reset_cabbus)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    args.func(args)


if __name__ == "__main__":
    main()
