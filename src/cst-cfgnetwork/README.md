# cst_cfgnetwork.py

Export/import loco configurations to/from the ProtoThrottle Receiver (mrbw-cabbus) shared network **Loco
Configuration (CNF) store** (`N01`-`N20`), over a USB-attached XBee radio speaking the same `'C'`
(push)/`'D'` (pull) MRBus/MRBee protocol used by a real throttle for the on-device `SAVE CNF` / `LOAD CNF`
menu `N01`-`N20` entries.

This is the wireless sibling of [`cst-cfgtransfer`](../cst-cfgtransfer/): that tool reads and writes the
EEPROM of a throttle over ISP with `avrdude`; this one reads and writes **shared network entries on the
ProtoThrottle Receiver** over the radio, with no physical throttle or ISP access needed at all - just a
USB XBee module on the same PAN as the layout. It reuses `slot_codec.py`/`cst_eeprom_layout.py` from that
tool directly (see the `sys.path` line at the top of `cst_cfgnetwork.py`) rather than a second copy, so
**JSON files from either tool are interchangeable** - the 128-byte payload format is identical, only the
embedded `source` dict differs.

## Requirements

- Python 3, plus **[`pyserial`](https://pyserial.readthedocs.io/)** (`pip install pyserial`) - the one
  dependency not needed by the ISP-only design of `cst_cfgtransfer.py`, since this tool talks to a real
  serial port instead of shelling out to `avrdude`.
- A USB-attached XBee radio (any USB-serial adapter with an FTDI-family chip is a safe bet for macOS -
  see the top-level repo discussion that led to this tool for hardware recommendations) carrying an XBee3
  (2.4GHz) module, configured with the **same firmware and profile used by the cst and cabbus boards** via
  the XCTU tool: the `XB3-24A_200D-th` firmware image (the `FIRMWARE` target in `doc/xbee/Makefile`
  references it under the XCTU install path - it is not a repo file itself) plus the `doc/xbee/xb3j_200D_cst.xml`
  profile in this repo, which is what actually pins the PAN ID/channel/options this layout uses. Mismatched
  PAN ID/channel/firmware means the radio simply will not see any traffic - there is no
  partial-compatibility state to debug around.
- The MRBus address of the ProtoThrottle Receiver (base-station range `0xD0`-`0xEF`) - run `discover`
  (below) if it is not already known.

## Platform notes

All development and testing to date has been on macOS 26 (Apple Silicon). The code is portable in
principle - Python 3, `pyserial`, `os.path` throughout - so Linux and Windows are supported by the code
but unverified; the notes below are the known setup, not a tested path.

- **USB-serial driver for the XBee adapter.** FTDI and CP2102 adapters install automatically on Windows
  10 and 11 and are in-kernel on Linux; a CH340 adapter can need a manual driver on Windows. macOS needs
  no driver for any of the three on a current release.
- **Linux**: add the calling user to the `dialout` group for access to the serial port, otherwise the
  tool cannot open it without root.
- **Serial port naming** differs by platform and is passed with `--port`: `COM7` on Windows,
  `/dev/ttyUSB0` on Linux, `/dev/tty.usbserial-XXXX` on macOS.
- On Windows, invoke the tool as `python cst_cfgnetwork.py ...`; the `./cst_cfgnetwork.py` form (shebang
  plus executable bit) is a macOS and Linux convenience only.

## Quick start

```bash
# Find the address of the ProtoThrottle Receiver (also lists any throttles that happen to be on and in range)
python3 cst_cfgnetwork.py discover --port /dev/tty.usbserial-XXXX --my-addr 0x48

# See what is stored in each shared network slot without a full pull (fast - just a 2-byte peek per entry)
python3 cst_cfgnetwork.py list --port /dev/tty.usbserial-XXXX --my-addr 0x48 --cabbus-addr 0xD0

# Watch the radio and decode throttle status packets (read-only diagnostic)
python3 cst_cfgnetwork.py sniff --port /dev/tty.usbserial-XXXX --my-addr 0x48 --changes

# Back up all 20 shared network entries to JSON
python3 cst_cfgnetwork.py export --port /dev/tty.usbserial-XXXX --my-addr 0x48 --cabbus-addr 0xD0 \
    --out-dir ~/protothrottle-backups/
#   -> ~/protothrottle-backups/cabbus-208/N01..N20_addr*.json

# Preview an edited config before pushing anything over the radio
python3 cst_cfgnetwork.py import --port /dev/tty.usbserial-XXXX --my-addr 0x48 --cabbus-addr 0xD0 \
    --dry-run edited-N05.json

# Push it for real, then pull it back and diff as extra assurance
python3 cst_cfgnetwork.py import --port /dev/tty.usbserial-XXXX --my-addr 0x48 --cabbus-addr 0xD0 \
    --verify edited-N05.json

# Restore a whole exported folder in one shot
python3 cst_cfgnetwork.py import --port /dev/tty.usbserial-XXXX --my-addr 0x48 --cabbus-addr 0xD0 \
    --dir ~/protothrottle-backups/cabbus-208/ --yes
```

Run `cst_cfgnetwork.py -h`, or `<subcommand> -h`, for the full flag reference.

### Choosing `--my-addr` / `--cabbus-addr`

`--my-addr` is the MRBus address used by this tool for the duration of the run - pick anything in the
throttle range `0x30`-`0x49` that is not already used by a real throttle on the layout (there is no
live-collision detection; a duplicate address just means confusing cross-talk). `--cabbus-addr` is the
configured address of the target ProtoThrottle Receiver, `0xD0`-`0xEF` - `discover` finds it and tags
anything in that range as `likely ProtoThrottle Receiver (base station address range)`.

## Watching radio traffic: `sniff`

`sniff` passively listens on the radio and prints every CRC-valid MRBus packet it sees. It never
transmits, needs no `--cabbus-addr`, and is safe to leave running. A ProtoThrottle status (`'S'`)
packet is decoded into a readable line - loco address, direction, DCC speed step, the active DCC
function numbers, throttle status flags (`EMRG`/`ALLSTOP`/`ALERTER`/`SLEEP`), and battery voltage;
every other packet type prints as `src->dest 'type' hex-bytes`.

```bash
python3 cst_cfgnetwork.py sniff --port /dev/tty.usbserial-XXXX --my-addr 0x48
```

- `--status-only` - drop everything except throttle `'S'` packets.
- `--changes` - only print a line when the decoded state of a given throttle actually changes (function
  mask, speed, direction, or status flags). This is the view for catching a transient glitch - e.g.
  twisting a light knob and confirming the headlight function number never blips out.
- `--cnf` - also decode the shared-CNF `'C'`/`'c'`/`'D'`/`'d'` push/pull packets (otherwise shown as
  raw hex). Run this while a throttle - or this tool - is doing a `SAVE CNF` / `LOAD CNF` to watch the
  transfer step by step:

  ```
  21:14:07  0x30->0xD0  PUSH BEGIN  N05  len=128 ver=1
  21:14:07  0xD0->0x30  PUSH BEGIN-ACK  N05  OK
  21:14:07  0x30->0xD0  PUSH DATA  N05  off=0 len=10  8B 10 00 ...
  21:14:07  0xD0->0x30  PUSH DATA-ACK  N05  off=0  OK
  ...
  21:14:08  0x30->0xD0  PUSH COMMIT  N05  crc=0x4F2A
  21:14:08  0xD0->0x30  PUSH COMMIT-ACK  N05  OK
  ```

  Combines with `--status-only` / `--changes` (shows `'S'` plus CNF, nothing else).
- `--seconds N` - stop automatically after `N` seconds instead of running until `Ctrl-C`.

The `'S'` decoder mirrors the packet layout of the status-packet builder in `mrbw-cst.c`
(`decode_cst_status()` in `cnf_radio_io.py`); it has been stable since stock ISE firmware. The `--cnf`
decoder (`format_cnf_packet()`) mirrors the wire layout of `push_entry()` / `pull_entry()` themselves and
is best-effort - a short or unrecognized packet just prints as hex, it never stops the sniff.

## Safety workflow

1. **`list` or `export` first.** Both are read-only against the ProtoThrottle Receiver - always safe.
2. **Validate offline.** `python3 -m unittest discover tests` runs the framing/CRC/protocol-state-machine
   tests with no hardware attached at all (see the docstring in `tests/test_cnf_radio_io.py`). This does not
   need a radio, but also does not validate the specific XBee setup in use - see step 3.
3. **Try a real edit on a spare/unused entry first** (e.g. `N20`, if nothing is using it), then confirm on
   the `LOAD CNF` picker of a real throttle that the loco-address preview and a subsequent load look right.
4. **`export --entry N` of a target entry immediately before importing to it** - this is a trustworthy read
   path already built into this tool, so it doubles as a backup that can be pushed back if something goes
   wrong. (Unlike the ISP access used by `cst_cfgtransfer.py`, there is no independent raw-EEPROM-dump
   fallback here - a shared network entry is only ever visible through this same protocol.)
5. `--dry-run` shows a full field-level diff without touching the radio; `--verify` pulls each entry back
   and diffs after pushing, on top of the CRC gate already built into the push protocol (see below).

`import` always validates every input file before touching the radio (aborting the whole run on any
error, no partial writes from a bad file) and, for a file targeting an entry that already holds data,
pulls the current bytes first so unmodeled padding bytes round-trip untouched rather than being zeroed.
**Each entry is pushed as its own independent radio transaction** (unlike the single atomic whole-image
write used over ISP) - if a multi-entry `--dir` batch fails partway through, entries pushed earlier in
that run are already written; the tool reports exactly where it stopped so the rest can be re-run.

**Push integrity**: the final `COMMIT` step of a push carries a CRC16 of the full 128-byte payload, and the
ProtoThrottle Receiver only actually writes its EEPROM if the CRC it computes fresh over what it received
matches - a mismatch is rejected outright (`CnfChecksumError`), and nothing gets written. This is a
stronger guarantee than a separate post-write read-back, which is why `--verify` (an extra full
pull-and-diff after every push) is opt-in rather than the default.

## Scoping

Every entry-touching subcommand takes `--entry N` (1-20, the on-device `N01`-`N20` numbering):

- `list`/`export` default to **all 20** entries when `--entry` is omitted; `--entry` is repeatable on
  `export` to select a subset.
- `import` targets exactly one entry per file: pass `--entry` explicitly, or omit it and let
  `source.entry` in the JSON decide (set automatically by `export`, or by hand when authoring a file from
  scratch). `import --dir <directory>` imports every `*.json` in that directory in one run, each to its
  own embedded `source.entry` - `--entry` is not valid alongside `--dir`, since there is one target per
  file already.

## Folder / filename convention

`export --out-dir <dir>` creates/reuses a `<dir>/cabbus-<address>/` subfolder (decimal address, matching
the `throttle-<address>/` convention used by `cst_cfgtransfer.py`) so the same `--out-dir` can be reused
across multiple ProtoThrottle Receiver units. Filenames are `N<NN>_addr<address>.json`, e.g.
`N05_addr4302L.json` (long address 4302), `N20_addr55S.json` (short address 55), `N12_addrNONE.json` (an
entry that has never been pushed to - matching the `NONE` vocabulary already used by the on-device picker
for an empty entry, not the local-slot `OFF` used by `cst_cfgtransfer`). Repeated exports overwrite the
folder in place - always "the latest," not dated snapshots; copy the folder aside first for history.

**A `NONE` stub file is diagnostic, not re-importable as is.** It is produced by decoding an all-`0xFF`
buffer, which round-trips a few fields (bits 29-31 of `force_functions`) that the encoder does not
accept back - the same asymmetry shown by `cst_cfgtransfer.py` for a truly blank local slot, since it uses
the same codec. Every `export`ed file embeds `source.empty: true`/`false` recording whether it was one of
these stubs; `import --dir` reads that flag and **automatically skips any `empty: true` file**, printing
which ones were skipped, so pointing `--dir` at a full, unfiltered export folder (see "Exporting and
importing every slot at once" below) just works without manual filtering. A single explicitly-named file
(no `--dir`) is never auto-skipped this way - pointing `import` directly at one `NONE` stub gives the real
validation error, not silence. Any real edit to a stub naturally replaces those force-function fields with
a program-derived list of DCC function numbers (0-28), which encodes fine either way.

## Exporting and importing every slot at once

Both are one command, no special flag needed to opt into "everything":

```bash
# Export all 20 shared network entries in one run - this is the default for export, not an opt-in
python3 cst_cfgnetwork.py export --port ... --my-addr ... --cabbus-addr ... --out-dir ~/backups/

# Import every file just produced by export, in one run - NONE stubs among them are skipped
# automatically (see above), so this "just works" on an unfiltered export folder
python3 cst_cfgnetwork.py import --port ... --my-addr ... --cabbus-addr ... \
    --dir ~/backups/cabbus-208/ --yes
```

`export --entry N` (repeatable) narrows to specific entries instead; `import --dir` always processes
every file in the given directory (there is no partial-directory flag - point it at a folder containing
only the files that need to be touched if a subset is needed).

## JSON field reference

Identical to [the field reference for `cst-cfgtransfer`](../cst-cfgtransfer/README.md#json-field-reference)
- same `loco_address`/`force_functions`/`functions`/`notch_speedstep`/`speed`/`options` structure, same
`"UNSET"` sentinel semantics, same function-value vocabulary. The only difference
is the embedded `source` dict: here it is always `{"scope": "network_slot", "entry": N, "cabbus_addr":
"0x..", "transport": "radio", "exported_at": ...}` (files exported before this rename used `"shared_slot"`
- still accepted on import) rather than the `{"scope": "slot"|"active"|"device", ...}` used by `cst_cfgtransfer` -
`source` is metadata only, never required on import, and not itself part of the loco configuration.

## Layout-version guard (protocol-level, table-wide)

The shared network CNF table on the ProtoThrottle Receiver (mrbw-cabbus) pins a single table-wide
`(layout_version, payload_length)` pair, set by whichever push established or last advanced it - every
`'C'`/`'D'` exchange goes through this guard, and `cst_cfgnetwork.py` enforces the same policy
`cst_cfgtransfer.py` does at the device level, just over the radio instead of via `avrdude`. See the
top-level `CLAUDE.md` for mrbw-cst ("Shared network CNF store" section) for the full design and the
rejected intermediate shapes.

- **`export`/`list`** compare the version pinned on the table against `SUPPORTED_LAYOUT_VERSION` from
  `cst_eeprom_layout.py`, built into this tool, and refuse (export) or flag per-row (list) on a real
  mismatch - `export` aborts before pulling anything, since the pin is a table-wide fact.
- **`import`** peeks the pin before pushing. A same-version push proceeds normally. **A push that would
  establish the first pin on the table, or advance it past an older existing one, is refused outright** -
  this tool never establishes or advances the pin itself, only real throttle firmware may. A strictly
  *lower* version than what is pinned is also refused, before any DATA traffic, with a message reporting
  that `cst_eeprom_layout.py` (or the firmware that pinned the table) needs updating first.
- **`reset-cabbus`** deliberately wipes the whole table and clears the pin, independent of any version
  comparison - for recovering from a bad state, or a clean slate. Backs up first by default (`--out-dir`
  required unless `--skip-backup`). This still works regardless of the restriction above, since it does
  not push a versioned payload at all.

**Why only a real throttle may advance the pin**: the `SUPPORTED_LAYOUT_VERSION` built into this tool
tracks the same source tree as the firmware, so it can drift ahead of every physical throttle from
nothing more than a `git pull` - no reflash required. If `import` were allowed to advance the pin on that
basis, a routine tool update could push the table to a version *no throttle in the field has yet*, and
every real throttle (still on the older firmware) would be refused on both push and pull - `N01`-`N20`
effectively disabled network-wide until every one of them is individually reflashed. Requiring a real
throttle to advance the pin first means the pin can never get ahead of what is actually deployed. (The
wire protocol used by mrbw-cabbus has no way to distinguish a PC tool from a real throttle at the packet
level - both use the same throttle address range - so this is enforced only by a policy in this tool, not
something the firmware can verify.)

To advance the table to a new version, push a real loco config from a real throttle (`SAVE CNF`, choose an
`N01`-`N20` entry) first; `import` will then succeed normally against the now-current pin. See the
top-level `CLAUDE.md` for mrbw-cst ("Shared network CNF store" section) for the full design, the
wipe-on-advance behavior, and the 192-byte-per-entry headroom reserved so *most* future format growth
never needs a ProtoThrottle Receiver reflash.

### Restoring a pre-upgrade backup: `--import-old`

`import --import-old` relaxes the validation done by `slot_codec.py` for a JSON file exported under an
OLDER schema than the one currently used by this tool - a field ABSENT from an existing
`functions`/`speed`/`options` object is defaulted (to `RAW:0xFF`, `"UNSET"`, or the existing invalid-value
fallback already used for that field, matching what a never-written value already means) rather than
rejected. A field that is *present* but invalid is still always rejected, and a whole missing category (no
`"functions"` object at all, say) still always fails - this only bridges "predates a newer field," never
"genuinely corrupt file." `--dry-run --import-old` shows the real defaulted values in its diff (not
`<missing>`), and prints an explicit list of what is being defaulted before the confirmation prompt.

This is what makes recovering from a version-advancing push made by a real throttle non-destructive in
practice: `export --out-dir <dir>` beforehand (a plain, manually-run backup - no longer something `import`
offers to do automatically, now that it never triggers an advance itself) captures the table under its
old schema; once a real throttle has advanced the pin (wiping the rest of the table),
`cst_cfgnetwork.py import --dir <dir> --import-old --yes` restores everything else, with any genuinely-new
field defaulted rather than blocking the whole restore.

## Maintaining this tool across EEPROM layout changes

Same codec, same maintenance obligation as `cst_cfgtransfer.py` - see the README for that tool and the
top-level `CLAUDE.md` ("PC tooling" section, "Maintenance checklist" subsection) for the full checklist.
Because this tool imports `slot_codec.py`/`cst_eeprom_layout.py` directly rather than duplicating them,
**any fix made there for `cst_cfgtransfer.py` automatically applies here too** - the only thing specific to
this tool is `cnf_radio_io.py` (the radio transport/protocol itself), which only needs touching if the `'C'`/
`'D'` wire protocol changes (see `cst-sync.c`/`cst-sync.h` in this repo and `cnf-store.c`/`cnf-store.h`/
`cabbus-eeprom.h` in mrbw-cabbus), not for an EEPROM *field* layout change. `.githooks/pre-commit` (wired
up by `make setup`, see the top-level `CLAUDE.md`) catches the specific class of drift where
`SUPPORTED_LAYOUT_VERSION` in `cst_eeprom_layout.py` is not bumped alongside `EEPROM_LAYOUT_VERSION` - a
real bug this exact tool family hit once (the HORN2 field) - but it is a local safety net, not a
substitute for following the checklist.
