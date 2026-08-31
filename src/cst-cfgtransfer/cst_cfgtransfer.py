#!/usr/bin/env python3
# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Export/import ProtoThrottle (MRBW-CST) loco configurations to/from hand-editable JSON files, over the
ISP programmer (avrdude), while the throttle's own firmware is not running.

See README.md for the full field reference and safety workflow. Run with -h for usage.
"""

import argparse
import glob
import json
import os
import sys

import avrdude_io
import cst_eeprom_layout as layout
import slot_codec


def _check_layout_version(raw_eeprom):
    chip_version = raw_eeprom[layout.EE_LAYOUT_VERSION]
    if chip_version == layout.SUPPORTED_LAYOUT_VERSION:
        return
    if chip_version == 0xFF:
        sys.exit(
            "ERROR: this throttle's EEPROM has no CNF format version stamped (reads 0xFF) - it likely "
            "hasn't booted with firmware built after the CNF format version guard was added. Flash "
            "current firmware and power the throttle on at least once (readConfig() stamps this "
            "automatically on every boot), then retry."
        )
    sys.exit(
        "ERROR: EEPROM CNF format version mismatch - the throttle's firmware stamped CNF format "
        "version %d, but this source checkout (src/cst-eeprom.h) is version %d. The codec only decodes "
        "the layout it was built for, so this is a hard stop. Check out the mrbw-cst revision matching "
        "the throttle's firmware - or, if this checkout is the newer one, flash it and power-cycle the "
        "throttle so readConfig() restamps - then retry." % (chip_version, layout.SUPPORTED_LAYOUT_VERSION)
    )


def _mrbus_addr(raw_eeprom):
    return raw_eeprom[layout.EE_MRBUS_DEVICE_ADDR]


def _addr_str_for_filename(raw_slot_bytes):
    if all(b == 0xFF for b in raw_slot_bytes):
        return "OFF"
    loco = slot_codec.decode_slot(raw_slot_bytes, source={})["loco_address"]
    suffix = "S" if loco["type"] == "short" else "L"
    return "%d%s" % (loco["address"], suffix)


def _format_loco_address(loco_addr_dict):
    suffix = "S" if loco_addr_dict["type"] == "short" else "L"
    return "%d%s" % (loco_addr_dict["address"], suffix)


def _now_iso():
    import datetime
    return datetime.datetime.now().astimezone().isoformat(timespec="seconds")


def _fw_version_str(raw_eeprom):
    # "X" matches this fork's git tag convention (X<major>.<minor>, see git-revision.sh) - EE_VERSION_MAJOR/
    # MINOR only ever hold the numeric major.minor parsed out of that tag by parseVersionStr(), with no
    # fork-name byte of its own in EEPROM, so the prefix is re-added here purely for a readable label.
    return "X%d.%d" % (raw_eeprom[layout.EE_VERSION_MAJOR], raw_eeprom[layout.EE_VERSION_MINOR])


# --- export ---

def _slot_range(n):
    start = layout.config_offset(n)
    return start, start + layout.CONFIG_SIZE


def cmd_export(args):
    raw = avrdude_io.read_full_eeprom()
    _check_layout_version(raw)

    addr = _mrbus_addr(raw)
    throttle_dir = os.path.join(args.out_dir, "throttle-%d" % addr)
    os.makedirs(throttle_dir, exist_ok=True)

    full_export = not (args.slot or args.active or args.device)
    slots = sorted(set(args.slot)) if args.slot else ([] if not full_export else list(range(1, 21)))
    do_active = args.active or full_export
    do_device = args.device or full_export

    fw_version = _fw_version_str(raw)
    manifest = []

    if do_device:
        source = {"scope": "device", "device_fw_version": fw_version, "exported_at": _now_iso()}
        d = slot_codec.decode_global(raw[0:layout.CONFIG_START], source)
        path = os.path.join(throttle_dir, "device.json")
        _write_json_file(path, d)
        manifest.append(("device", "-", os.path.basename(path)))

    if do_active:
        start, end = _slot_range(layout.WORKING_CONFIG)
        raw_slot = raw[start:end]
        source = {"scope": "slot", "slot": "active", "device_fw_version": fw_version,
                  "exported_at": _now_iso()}
        d = slot_codec.decode_slot(raw_slot, source)
        fname = "slot00_active_addr%s.json" % _addr_str_for_filename(raw_slot)
        path = os.path.join(throttle_dir, fname)
        _write_json_file(path, d)
        manifest.append(("active", _addr_str_for_filename(raw_slot), os.path.basename(path)))

    for n in slots:
        if not (1 <= n <= layout.MAX_CONFIGS):
            sys.exit("ERROR: --slot %d is out of range (must be 1-%d)" % (n, layout.MAX_CONFIGS))
        start, end = _slot_range(n)
        raw_slot = raw[start:end]
        source = {"scope": "slot", "slot": n, "device_fw_version": fw_version, "exported_at": _now_iso()}
        d = slot_codec.decode_slot(raw_slot, source)
        fname = "slot%02d_addr%s.json" % (n, _addr_str_for_filename(raw_slot))
        path = os.path.join(throttle_dir, fname)
        _write_json_file(path, d)
        manifest.append(("slot %d" % n, _addr_str_for_filename(raw_slot), os.path.basename(path)))

    print("Throttle MRBus address: %d  ->  %s" % (addr, throttle_dir))
    print()
    for scope, addr_str, fname in manifest:
        print("  %-10s loco %-10s %s" % (scope, addr_str, fname))


def _write_json_file(path, d):
    with open(path, "w", encoding="utf-8", newline="\n") as f:
        json.dump(d, f, indent=2)
        f.write("\n")


def _load_json_file(path):
    with open(path, encoding="utf-8") as f:
        return json.load(f)


# --- import ---

def _target_key_from_source(source, path):
    scope = source.get("scope")
    if scope == "device":
        return ("device",)
    if scope == "slot":
        slot = source.get("slot")
        if slot == "active":
            return ("active_from_source",)  # deliberately not auto-selectable; see caller
        if isinstance(slot, int):
            return ("slot", slot)
    sys.exit("ERROR: %s: can't determine target (source.scope/source.slot missing or invalid) - "
             "pass --slot/--active/--device explicitly" % path)


def _resolve_single_file_target(args, d, path):
    if args.slot is not None:
        return ("slot", args.slot)
    if args.active:
        return ("active",)
    if args.device:
        return ("device",)
    key = _target_key_from_source(d.get("source", {}), path)
    if key == ("active_from_source",):
        sys.exit("ERROR: %s was exported from the active/working profile - pass --active explicitly "
                 "to confirm you want to write it there (not auto-selected)." % path)
    return key


def _target_label(key):
    if key[0] == "device":
        return "device"
    if key[0] == "active":
        return "active"
    return "slot %d" % key[1]


def _target_byte_range(key):
    if key[0] == "device":
        return 0, layout.CONFIG_START
    if key[0] == "active":
        return _slot_range(layout.WORKING_CONFIG)
    return _slot_range(key[1])


def _encode_target(key, d, path, base=None, allow_missing=False):
    # allow_missing (--import-old) only applies to encode_slot() - encode_global() has no per-field
    # "UNSET" sentinel to default to (every device-level field is a plain ranged integer with no natural
    # "not configured yet" value), and the CNF-restore-after-upgrade use case this flag exists for is
    # entirely about per-loco slot data, not device-level settings.
    try:
        if key[0] == "device":
            return slot_codec.encode_global(d, base=base)
        return slot_codec.encode_slot(d, base=base, allow_missing=allow_missing)
    except slot_codec.SlotValidationError as e:
        print("ERROR: %s failed validation:" % path, file=sys.stderr)
        for err in e.errors:
            print("  - %s" % err, file=sys.stderr)
        sys.exit(1)


def _gather_import_items(args):
    """Returns a list of (target_key, path, loaded_dict). Validates every file up front (against a
    zero-filled base, purely to check for errors before any hardware I/O) - the real, base-preserving
    encode happens later in cmd_import() once the current on-device bytes are known.
    """
    items = []
    if args.dir:
        paths = sorted(glob.glob(os.path.join(args.dir, "*.json")))
        if not paths:
            sys.exit("ERROR: no *.json files found in %s" % args.dir)
        for path in paths:
            d = _load_json_file(path)
            key = _target_key_from_source(d.get("source", {}), path)
            if key == ("active_from_source",):
                key = ("active",)  # --dir batches are trusted to restore a full backup verbatim
            _encode_target(key, d, path, allow_missing=args.import_old)  # validate only; result discarded
            items.append((key, path, d))
    else:
        d = _load_json_file(args.file)
        key = _resolve_single_file_target(args, d, args.file)
        _encode_target(key, d, args.file, allow_missing=args.import_old)  # validate only; result discarded
        items.append((key, args.file, d))

    seen = {}
    for key, path, _d in items:
        if key in seen:
            sys.exit("ERROR: both %s and %s target %s - aborting, nothing written"
                     % (seen[key], path, _target_label(key)))
        seen[key] = path
    return items


def _report_torn_write_and_exit(write_err, encoded_items):
    """Every write_full_eeprom() retry failed - re-read the chip and tell the user exactly which
    targeted item(s), if any, actually ended up inconsistent, rather than leaving them guessing. Reuses
    the same per-key byte-range comparison the normal post-write verify step uses."""
    print("\nERROR: %s" % write_err, file=sys.stderr)
    print("\nChecking whether the chip was left partially written...", file=sys.stderr)
    try:
        post = avrdude_io.read_full_eeprom()
    except avrdude_io.AvrdudeError as read_err:
        sys.exit(
            "ERROR: could not even read the chip back after the failed write (%s) - its current state is "
            "UNKNOWN. Re-run 'export' once the connection is working again before trusting the throttle."
            % read_err
        )
    torn = [(key, path) for key, path, _d, encoded in encoded_items
            for start, end in [_target_byte_range(key)] if post[start:end] != encoded]
    if not torn:
        sys.exit(
            "The write reported failure, but a fresh read-back shows all targeted data actually made it "
            "onto the chip correctly. Nothing appears corrupted, but re-run 'export' and check before "
            "trusting this."
        )
    print("\nERROR: the following targeted item(s) do NOT match what was intended and may now hold "
          "inconsistent data:", file=sys.stderr)
    for key, path in torn:
        print("  - %-8s <- %s" % (_target_label(key), path), file=sys.stderr)
    print("\nRe-run 'export' now to capture the chip's actual current state.", file=sys.stderr)
    sys.exit(1)


def cmd_import(args):
    if bool(args.dir) == bool(args.file):
        sys.exit("ERROR: pass exactly one of a JSON file or --dir")
    if args.dir and (args.slot is not None or args.active or args.device):
        sys.exit("ERROR: --slot/--active/--device aren't valid with --dir - each file's own embedded "
                 "source.scope/source.slot is used")

    items = _gather_import_items(args)

    print("Reading current EEPROM from throttle...")
    raw = avrdude_io.read_full_eeprom()
    _check_layout_version(raw)
    print("Throttle MRBus address: %d" % _mrbus_addr(raw))
    print()

    if args.import_old:
        notices = []
        for _key, path, d in items:
            for notice in slot_codec.describe_missing_fields(d):
                notices.append("%s: %s" % (path, notice))
        if notices:
            print("--import-old: the following fields are missing from their source file and will be")
            print("defaulted (not silently - listed here so you can review before confirming):")
            for notice in notices:
                print("  - %s" % notice)
            print()

    spliced = bytearray(raw)
    changed_ranges = []
    encoded_items = []  # (key, path, d, encoded) - encoded bytes computed with the real on-device base
    for key, path, d in items:
        start, end = _target_byte_range(key)
        old_bytes = raw[start:end]
        # Re-encode now that we know the real current bytes, so fields outside this schema (global-block
        # bytes like EE_LAYOUT_VERSION, or a slot's unused padding) are preserved rather than zero-filled.
        encoded = _encode_target(key, d, path, base=old_bytes, allow_missing=args.import_old)
        if key[0] == "device":
            old_summary = "device settings"
            new_summary = "device settings"
        else:
            old_loco = slot_codec.decode_slot(old_bytes, source={})["loco_address"]
            old_summary = _format_loco_address(old_loco)
            new_summary = _format_loco_address(d["loco_address"])
        print("  %-8s loco %-10s -> %-10s  <- %s" % (_target_label(key), old_summary, new_summary, path))
        spliced[start:end] = encoded
        changed_ranges.append((start, end))
        encoded_items.append((key, path, d, encoded))

    # Safety assertion: confirm only the intended byte ranges actually differ.
    for i in range(layout.EEPROM_SIZE):
        if raw[i] != spliced[i] and not any(start <= i < end for start, end in changed_ranges):
            sys.exit("INTERNAL ERROR: byte 0x%04X changed outside any targeted range - aborting before "
                     "writing anything to hardware. This is a bug in cst_cfgtransfer.py, not your JSON."
                     % i)

    if args.dry_run:
        print()
        print("--dry-run: showing field-level changes, nothing written to hardware.")
        for key, path, d, encoded in encoded_items:
            start, end = _target_byte_range(key)
            old_bytes = raw[start:end]
            if key[0] == "device":
                old_d = slot_codec.decode_global(old_bytes, source={})
                new_d = slot_codec.decode_global(bytes(encoded), source={})
            else:
                old_d = slot_codec.decode_slot(old_bytes, source={})
                # Compares against the actually-encoded-then-redecoded result, not the raw loaded `d` -
                # with --import-old, a defaulted field would otherwise misleadingly show as <missing>
                # here instead of the real value it's about to become.
                new_d = slot_codec.decode_slot(bytes(encoded), source={})
            print()
            print("=== %s (%s) ===" % (path, _target_label(key)))
            _print_dict_diff(old_d, new_d)
        return

    if not args.yes:
        answer = input("\nWrite the above to the throttle now? [y/N] ").strip().lower()
        if answer != "y":
            print("Aborted, nothing written.")
            return

    print("\nWriting EEPROM...")

    def _report_retry(attempt, max_attempts, exc):
        print("  attempt %d/%d failed, retrying: %s" % (attempt, max_attempts, str(exc).splitlines()[0]))

    try:
        avrdude_io.write_full_eeprom(bytes(spliced), on_retry=_report_retry)
    except avrdude_io.AvrdudeWriteFailedError as e:
        _report_torn_write_and_exit(e, encoded_items)

    if args.no_verify:
        print("Done (--no-verify: skipped post-write verification).")
        return

    print("Verifying...")
    reread = avrdude_io.read_full_eeprom()
    ok = True
    for key, path, _d, encoded in encoded_items:
        start, end = _target_byte_range(key)
        if reread[start:end] != encoded:
            print("  FAIL: %s (%s) did not verify" % (path, _target_label(key)))
            ok = False
        else:
            print("  PASS: %s (%s)" % (path, _target_label(key)))
    if not ok:
        sys.exit(1)
    print("Done.")


# --- dump ---

def cmd_dump(args):
    """Raw EEPROM readback - no decode, and deliberately NOT version-gated (you may need to dump a chip
    whose format this tool doesn't understand, precisely to work out what's wrong with it)."""
    raw = avrdude_io.read_full_eeprom()
    header = "firmware %s, MRBus addr 0x%02X" % (_fw_version_str(raw), _mrbus_addr(raw))
    if args.out:
        with open(args.out, "wb") as f:
            f.write(raw)
        n_set = sum(1 for b in raw if b != 0xFF)
        print("wrote %s (%d bytes; %d non-0xFF)  [%s]" % (args.out, len(raw), n_set, header))
        return
    print("# %s" % header)
    any_row = False
    for base in range(0, len(raw), 16):
        row = raw[base:base + 16]
        if any(b != 0xFF for b in row):
            print("0x%04X  %s" % (base, " ".join("%02X" % b for b in row)))
            any_row = True
    if not any_row:
        print("(entire EEPROM is 0xFF)")


# --- wipe ---

def cmd_wipe(args):
    """Factory-blank the throttle. Not version-gated - a wipe is how you recover from a version mismatch."""
    if not args.yes:
        print("This ERASES THE THROTTLE'S FLASH *AND* EEPROM: all 20 saved slots, the active profile,")
        print("device settings, threshold calibration, and the firmware image itself. There is no undo.")
        print("Back up first:  python3 cst_cfgtransfer.py export --out-dir ~/protothrottle-backups/")
        if input('Type "ERASE" to continue: ').strip() != "ERASE":
            sys.exit("Aborted.")
    print("Blanking EEPROM (HFUSE EESAVE toggle + chip erase)...")
    try:
        avrdude_io.blank_eeprom_via_fuse_toggle(on_progress=lambda msg: print("  %s" % msg))
    except avrdude_io.EepromWipeError as e:
        sys.exit("ERROR: %s" % e)
    print("Done - EEPROM is all 0xFF and HFUSE is restored. Flash is empty: flash firmware and")
    print("power the throttle on (it will land on the on-device Factory Reset screen).")


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


# --- argument parsing ---

def build_parser():
    parser = argparse.ArgumentParser(prog="cst_cfgtransfer.py", description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)

    exp = sub.add_parser("export", help="Read the connected throttle's EEPROM to JSON file(s)")
    exp.add_argument("--out-dir", required=True, type=os.path.expanduser,
                      help="Parent backup directory; a throttle-<mrbus-addr>/ subfolder is created/reused")
    exp.add_argument("--slot", type=int, action="append",
                      help="Export only this numbered slot (1-20); repeatable")
    exp.add_argument("--active", action="store_true",
                      help="Export only the active/working profile (written as slot00_active_addr*.json)")
    exp.add_argument("--device", action="store_true", help="Export only the device-level settings")
    exp.set_defaults(func=cmd_export)

    imp = sub.add_parser("import", help="Write JSON file(s) to the connected throttle's EEPROM")
    imp.add_argument("file", nargs="?", type=os.path.expanduser,
                      help="JSON file to import (omit if using --dir)")
    imp.add_argument("--dir", type=os.path.expanduser,
                      help="Import every *.json in this directory, each to its own embedded "
                                    "source.scope/source.slot")
    target = imp.add_mutually_exclusive_group()
    target.add_argument("--slot", type=int, help="Target this numbered slot (1-20)")
    target.add_argument("--active", action="store_true", help="Target the active/working profile")
    target.add_argument("--device", action="store_true", help="Target the device-level settings")
    imp.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    imp.add_argument("--dry-run", action="store_true",
                      help="Show what would change without writing to hardware")
    imp.add_argument("--no-verify", action="store_true", help="Skip post-write verification")
    imp.add_argument("--import-old", action="store_true",
                      help="Allow a JSON file exported under an older/smaller schema (e.g. a backup taken "
                           "before a CNF format bump added a new field) - fields missing from an "
                           "existing functions/speed/options object are defaulted instead of rejected. A "
                           "field that's present but invalid is still always rejected, and a whole "
                           "missing category (e.g. no \"functions\" object at all) still always fails.")
    imp.set_defaults(func=cmd_import)

    dmp = sub.add_parser("dump", help="Read the raw EEPROM to a file, or print a hex summary "
                                       "(no decode, no version check)")
    dmp.add_argument("--out", metavar="FILE", type=os.path.expanduser,
                      help="Write the raw 4096-byte image here (default: print a hex summary to stdout)")
    dmp.set_defaults(func=cmd_dump)

    wp = sub.add_parser("wipe", help="Factory-blank the throttle's EEPROM (and flash) via an HFUSE "
                                      "EESAVE toggle + chip erase")
    wp.add_argument("--yes", action="store_true", help="Skip the confirmation prompt")
    wp.set_defaults(func=cmd_wipe)

    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    try:
        args.func(args)
    except avrdude_io.AvrdudeError as e:
        sys.exit("ERROR: %s" % e)


if __name__ == "__main__":
    main()
