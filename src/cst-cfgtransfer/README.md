# cst_cfgtransfer.py

Export/import MRBW-CST ("ProtoThrottle") loco configurations to/from hand-editable JSON files, over the
ISP programmer (`avrdude`), while the firmware on the throttle itself is **not** running.

This is offline EEPROM manipulation with the chip sitting in an ISE AVR Programmer (or any
`avrdude`-compatible ISP programmer) wired to the ISP header on the board — the same physical setup used
for
`make flash`. It is not a wireless/runtime interface; the throttle does not need to be powered up in the
normal sense, just connected to the programmer.

## Requirements

- Python 3, stdlib only — no `pip install` needed.
- `avrdude` on `PATH`, already required for `make flash`/`make fuse` (see the top-level CLAUDE.md for this
  repo, "Build / flash" section, for the macOS/Homebrew setup).
- The throttle must have **booted at least once with firmware that includes the layout-version guard**
  (i.e. built after `EE_LAYOUT_VERSION` was added to `src/cst-eeprom.h`). `readConfig()` stamps this
  automatically on every boot, so a single power-cycle after flashing current firmware is enough — the
  tool will tell you exactly this if it is not the case yet.

## Platform notes

All development and testing to date has been on macOS 26 (Apple Silicon). The code is portable in
principle — Python 3, standard library only, `os.path` throughout — so Linux and Windows are supported by
the code but unverified; the setup below is the known path, not a tested one.

- **avrdude 8.2 or newer** ships a stock `iseavrprog` entry (VID `0x1209` / PID `0x6570`). On an older
  `avrdude`, add that programmer to `avrdude.conf` by hand.
- **Linux**: the ISP programmer needs a udev rule granting the calling user access to USB device
  `1209:6570` (usually through the `plugdev` or `dialout` group), otherwise `avrdude` must run as root.
- **Windows**: the programmer needs a WinUSB or libusb driver, installed with Zadig — usbtiny-class
  devices have no stock Windows driver. Invoke the tool as `python cst_cfgtransfer.py ...`; the
  `./cst_cfgtransfer.py` form (shebang plus executable bit) is a macOS and Linux convenience only.
- The EEPROM-write retry and timeout logic is platform-agnostic, but the recovery behaviour after a
  failed write has been exercised only on macOS.

## Quick start

```bash
# Back up everything (20 saved slots + the active profile + device settings) to JSON
python3 cst_cfgtransfer.py export --out-dir ~/protothrottle-backups/
#   -> ~/protothrottle-backups/throttle-<mrbus-addr>/{slot00_active,slot01..slot20}_addr*.json, device.json

# Preview an edited config before writing anything to hardware
python3 cst_cfgtransfer.py import --slot 5 --dry-run edited-slot05.json

# Write it for real
python3 cst_cfgtransfer.py import --slot 5 edited-slot05.json

# Restore a whole backup folder for one throttle in one shot
python3 cst_cfgtransfer.py import --dir ~/protothrottle-backups/throttle-42/ --yes

# Raw EEPROM readback (no decode) - hex summary, or the full image to a file
python3 cst_cfgtransfer.py dump
python3 cst_cfgtransfer.py dump --out throttle-42-raw.bin

# Factory-blank the throttle (erases EEPROM AND flash - back up first!)
python3 cst_cfgtransfer.py wipe
```

Run `cst_cfgtransfer.py -h`, `export -h`, or `import -h` for full flag reference.

## Safety workflow (read this before writing to a live throttle)

1. **Export first.** It is read-only against hardware — always safe, and gives you a backup regardless of
   what you do next.
2. **Validate offline.** `python3 -m unittest discover tests` runs the codec round-trip tests with no
   hardware attached.
3. **Round-trip test on a spare slot** (e.g. slot 20, not a slot you are actively using): export it,
   `import --slot 20` the same file back unmodified, re-export and diff — should be byte-identical.
4. **Try a real edit on that same spare slot**, then confirm on the on-device menu of the throttle itself
   that exactly the field you changed actually changed.
5. **Only then** use the tool on your real active profile / live numbered slots.
6. Before your first real import to a live slot, consider keeping an independent raw EEPROM backup
   (`avrdude -P usb -c iseavrprog -p atmega1284p -B1 -U eeprom:r:backup.bin:r`) as a safety net.

`import` always: validates every input file before touching hardware (aborting the whole operation on any
error, never a partial write), reads the current full EEPROM, splices in only the byte range(s) you
actually targeted, and writes the full image back — every other slot and the global block round-trip
byte-for-byte unchanged — then, unless `--no-verify`, reads the chip back afterward to confirm the write
actually took. Use `--dry-run` to see a field-level before/after diff without writing anything.

Writes retry automatically (up to 3 times) if the connection hiccups — this is a known, structural
reliability quirk of the EEPROM-write path of the ISP programmer (see the PC tooling section of
`CLAUDE.md` for why), not a sign anything is wrong with your throttle or cable. A printed retry message is normal; only a
failure across all 3 attempts needs your attention, at which point `import` re-reads the chip and tells you
exactly which item(s), if any, may now be inconsistent. Occasionally (rare, but possible after several
failed writes in a row) the connection can hang rather than fail cleanly — every command, including that
re-read, has a 90-second timeout so this always ends in a clean error instead of hanging forever. No
power-cycle needed to recover from it; just retry.

### Restoring a pre-upgrade backup: `--import-old`

`import --import-old` allows a JSON file exported under an OLDER/smaller schema than this copy of the
tool (e.g. a backup taken before a layout-version bump added a new field, like the shared-network-CNF-table
version-guard upgrade documented in the top-level `CLAUDE.md`, "Shared network CNF store" section) — a field ABSENT from an existing
`functions`/`speed`/`airbrake`/`options`/`force_functions` object is defaulted (`RAW:0xFF` for a
function, `"UNSET"` for a speed/airbrake field, `"ADDITIVE"` for `options.horn_type`, `[]` for
`force_functions.on`/`.off`, or the existing invalid-value fallback already defined for that field)
instead of rejected — this is how a pre-schema-2 backup restores. The whole `airbrake` object is also
new (schema v3) — a v2 backup lacking it entirely restores fine with this flag (every field
`"UNSET"`), the one whole-category exception. (The old flat/renamed shapes — `force_function_on`/`off`,
`brake` / `options_unset`, ungrouped device fields — are also accepted, independent of this flag.) A
field that is *present* but invalid, or any other whole missing
category (no `"functions"` object at all), is still always rejected regardless of this flag — only
genuine absence within an already-present category (or a missing `airbrake`) is relaxed. `--dry-run --import-old` shows the real
defaulted value in its diff, and every import with this flag set prints an explicit "will be defaulted"
list before the confirmation prompt.

## Factory-blanking / recovering a throttle: `wipe` and `dump`

`dump` is a raw EEPROM readback with no decode and no version check — a hex summary of the non-`0xFF`
rows, or `--out FILE` for the full 4096-byte image. Use it to see what is actually on a chip, including
one whose format this tool would otherwise refuse (`export`/`import` are version-gated; `dump` is not).

`wipe` sets every EEPROM byte to `0xFF` — a genuine factory-blank. Use it before handing a throttle to
someone else, to recover from a corrupted or wrong-`EEPROM_LAYOUT_VERSION` EEPROM, or for a clean slate
without walking the on-device Factory Reset 5-count. It is **not** version-gated: a wipe is *how* you
recover from a version mismatch. It erases **flash as well** — the throttle has no firmware afterward
until you re-flash it (`make flash` from `src/`), after which it boots to the on-device Factory Reset
screen.

`wipe` does **not** use the ordinary EEPROM-write path — a full all-`0xFF` image write over this
programmer hits exactly the write-reliability quirk described above. Instead it clears the HFUSE `EESAVE`
bit (`0xD1`→`0xD9`), does a chip erase (which then wipes EEPROM as a side effect, via the ordinary
reliable erase of avrdude), and restores `EESAVE`. The restore runs on every exit path and is verified; if it ever
fails, `wipe` stops with a loud error and the exact `avrdude … -U hfuse:w:0xD1:m` command to fix it —
until you do, an ordinary `make flash` would itself wipe the throttle config, since a cleared `EESAVE`
means "erase EEPROM on every chip erase". Always `export` a backup first — the stored configs of a wiped
throttle are unrecoverable.

## Scoping flags

Both `export` and `import` understand three targets:

| Flag | Target |
|---|---|
| `--slot N` | Numbered saved slot 1-20 |
| `--active` | The live/working profile (`WORKING_CONFIG`) — what the throttle is driving with right now |
| `--device` | The device-level global settings (MRBus address, battery thresholds, sleep/alerter timeouts, brake/horn lever calibration) — specific to this physical throttle, not to a loco |

`export` with no scoping flags does all of the above (22 files total). `export --slot 5 --active` does
just those two, leaving any other previously-exported files in the folder for that throttle untouched.

`import` targets exactly one thing per file: pass `--slot`/`--active`/`--device` explicitly, or omit them
and let `source.scope`/`source.slot` embedded in the JSON itself decide — except a file whose
`source.scope` is `"active"` or `"device"` is never auto-targeted without the matching explicit flag, since
writing the live profile or the device settings themselves is a deliberately higher-friction operation than
writing a saved slot.

`import --dir <directory>` imports every `*.json` file in that directory in one read+write pass, each to
its own embedded `source.scope`/`source.slot` — this is how you restore a whole `export`ed folder, or any
subset of it (a directory containing only some of the files only touches the targets of those files;
everything else on the chip is left exactly as it was).

## Per-throttle backup folders

`export --out-dir <dir>` treats `<dir>` as a **parent** location and creates/reuses a
`throttle-<mrbus-address>/` subfolder inside it, so the same `--out-dir` can be reused across multiple
physical throttles without their backups colliding. The MRBus device address is used as the throttle
identifier because the ATmega1284P has no factory-unique serial number readable over ISP — but MRBus
already requires each unit on a layout to have a distinct address to function, so it is a practical
stand-in.
Repeated exports of the same throttle overwrite that folder in place (always "the latest" state); copy the
folder aside first if you want to keep a dated snapshot.

Filenames encode both the slot number and the loco address stored there, e.g. `slot05_addr4302L.json`
(long address 4302), `slot20_addr55S.json` (short address 55), `slot03_addrOFF.json` (a slot that has never
been configured — every byte reads `0xFF`). The live/working profile is `slot00_active_addr*.json` — named
`slot00` so it groups with the numbered slots in a file listing, but it is targeted with `--active` (not
`--slot 0`), and its `source.slot` is `"active"`.

## JSON field reference

Every field is written using the same vocabulary — and the same order — as the on-device menus of the
throttle itself, so it is editable without cross-referencing the C source in this codebase. The JSON is
**grouped one object per config menu**; within each object the keys are in the item order of that menu, and
the objects follow the top-level menu cycle
(`loco_address` → `force_functions` → `functions` → `notch_speedstep` → `speed` → `airbrake` →
`options`).

- **`loco_address`**: `{"address": N, "type": "long"|"short"}` — long is 0-9999, short is 0-127.
- **`force_functions`** (the `FORCE FUNC` menu — separate from `CONFIG FUNC`): `{"on": [...], "off":
  [...]}`, each a list of DCC function numbers (0-28) always forced on / off, independent of any
  physical control.
- **`functions`**: 30 keys, in `CONFIG FUNC` menu order — `HORN`, `HORN2`, `BELL`, `BRAKE`, `BRAKE2`,
  `BRAKE3`, `AUX`, `ENGINE_ON`, `ENGINE_OFF`, `THR_UNLOCK`, `REV_SWAP`, `NEUTRAL`, `COMPRESSOR`,
  `COMPRESSOR2`, `BRAKE_SET`, `BRAKE_REL`, `ALERTER`, `EMERGENCY`, `FRONT_HEADLIGHT`, `FRONT_DITCH`,
  `FRONT_DIM1`, `FRONT_DIM2`, `REAR_HEADLIGHT`, `REAR_DITCH`, `REAR_DIM1`, `REAR_DIM2`, `UP_BUTTON`,
  `DOWN_BUTTON`, `MENU_BUTTON`, `SEL_BUTTON` — each a string: `"OFF"`, `"F00_MOM"`..`"F28_MOM"`
  (momentary DCC function 0-28), `"F00_LAT"`..`"F28_LAT"` (latching), `"EMRG"` (emergency stop — only
  valid on `AUX`, `ALERTER`, `UP_BUTTON`, `DOWN_BUTTON`, `MENU_BUTTON`, `SEL_BUTTON`), `"AIRBRAKE"`
  (opens the AIRBRAKE gauge screen — only valid on `UP_BUTTON`, `DOWN_BUTTON`, `MENU_BUTTON`,
  `SEL_BUTTON`), or `"LOAD"` (cycles the SPEED CFG `OPLOADFN`/`PRLOADFN` load simulation on each press
  — only valid on `UP_BUTTON`, `DOWN_BUTTON`, `MENU_BUTTON`, `SEL_BUTTON`, and rejected on more than
  one of those four at once, mirroring the on-device firmware restriction). `MENU_BUTTON` /
  `SEL_BUTTON` are the OPS MODE function buttons — only driven while the OPS MODE screen is active (a
  long-press of MENU from the base screen with the `ops_mode` PREFS bit set), but always present in
  the JSON. `COMPRESSOR2` only appears as a selectable value on-device
  when `airbrake.COMP_MODE` is `"CONSIST"` and the `airbrake` PREFS bit is on, but is always present
  in the JSON regardless.
- **`notch_speedstep`**: 8 entries, 1-126, the reverser-notch-to-DCC-speed-step table.
- **`speed`**: decoder-family-shaped, under the exact `SPEED CFG` on-device menu names. Five fields are
  always first — `TYPE` (`"V5DCC"` / `"V5MULT"` / `"V4"`), `MAXSPEED`, `UNIT` (`"MPH"` / `"KMH"`),
  `ACCEL`, `DECEL` — with `ACCELADJ` / `DECELADJ` keyed immediately after the `ACCEL` / `DECEL` they
  adjust for a `TYPE` that has them, then the rest of the model parameters that `TYPE` uses. `BRK1` is
  present for every `TYPE` too, but it leads that remainder (grouped with `BRK2`/`BRK3`) rather than
  the agnostic block. `V5DCC` and `V5MULT` carry all 16 model fields (`ACCELADJ`, `DECELADJ`, `BRK1`,
  `BRK2`, `BRK3`, `DELAY`, `HOLDFN`, `STOPFN`, `OPLOAD`, `OPLOADFN`, `PRLOAD`, `PRLOADFN`, `ACCPCT`,
  `ACCTGT`, `DECPCT`, `DECTHR`); `V4` carries the 8-field subset (`BRK1`, `DELAY`, `HOLDFN`, `STOPFN`,
  `ACCPCT`, `ACCTGT`, `DECPCT`, `DECTHR`) — it has no `CV23`/`CV24`/`CV180`/`CV181`/`CV103`/`CV104`, so
  `ACCELADJ`/`DECELADJ`, `BRK2`/`BRK3` and the load CVs are absent. `ACCELADJ`/`DECELADJ` mirror ESU
  CV23/CV24 (a signed factor added to `ACCEL`/`DECEL`) and are a plain signed integer `-127`..`127`.
  `ACCEL` and `DECEL` are genuine
  `0`-`255` fields (a decoder's literal CV3/CV4 can be `255`, so a raw `0xFF` decodes to `255`, and a
  bare `"UNSET"` for one of them imports as its default rather than the sentinel); every other
  plain-numeric `speed` field is `0`-`254` or `"UNSET"`. Watched-function fields
  (`HOLDFN`/`STOPFN`/`OPLOADFN`/`PRLOADFN`) are `"OFF"` or `"F00"`..`"F28"`. A pre-5-schema backup
  missing `ACCELADJ`/`DECELADJ` needs `--import-old` (they default to `0`); a `V4` `TYPE` on a flat backup
  needs `--import-old` too, which ignores the inapplicable fields.
- **`airbrake`**: 9 fields under the exact `AIRBRAKE CFG` on-device menu names — `BP_CHARGE`, `MR_LOAD`,
  `MR_LOW`, `MR_HIGH`, `RECHARGE`, `LEAK_RATE`, `PUMP_RATE`, `DISPLAY`, `COMP_MODE` — the per-loco
  parameters of the AIRBRAKE air-brake model, all always visible on-device (no `ADV FUNC` gating left in
  this menu). Every field is a plain 0-254 number or `"UNSET"` except two string enums (same `"UNSET"`
  sentinel handling, matching the `options.horn_type` precedent): `DISPLAY` is `"DUAL"` (the `BP:`/`MR:`
  two-pressure view) or `"SINGLE"` (the analogue BP dial) — which rendering the AIRBRAKE screen uses;
  `COMP_MODE` is `"NORMAL"` or `"CONSIST"` — gates the `COMPRSR`/`COMPRSR2` split. `BP_CHARGE` is
  range-checked 70-110 and `MR_LOAD` 0-100 on import, matching the on-device UP/DOWN ceiling/floor
  exactly — an out-of-range value is rejected rather than silently clamped. A pre-v3 backup lacking the
  whole `airbrake` object, or one whose `airbrake` object predates `DISPLAY`, restores fine with
  `--import-old` (missing fields default to `"UNSET"`).
- **`options`** (the `OPTIONS` menu — mostly brake, plus reverser/horn), in menu order: `variable_brake`,
  `type` (`"PULSE"`/`"STEP"`/`"STACK"`), `pulse_width` (2-10), `stack_5step`, then
  `stack_band_combos_5step` (5 entries) / `stack_band_combos_3step` (3 entries) — both always present
  regardless of which variant is selected, each entry a string like `"BRAKE1--"`..`"BRAKE123"` (digit =
  that brake asserted, dash = not), matching the `STEP1`..`STEPn` editor display exactly — then
  `estop_on_full_brake`, `reverser_swap` (booleans), and `horn_type` (`"ADDITIVE"` = Horn2 stacks on top
  of Horn1, the default; `"EXCLUSIVE"` = Horn2 replaces Horn1 — the `HORNTYPE` / `1 ←→ 1+2` vs `1 ←→ 2`
  menu item). `unset: true` means the option byte has never been written on this device (the throttle
  fills in its own defaults on next boot) — when true, every `options.*` field besides `pulse_width` and
  the combo arrays is ignored on import. (Backups written before this rename used `brake` /
  `options_unset` — still accepted on import.)

**`"UNSET"`**: any field the throttle firmware itself treats as "self-healing" (the `options` group,
each STACK combo entry independently, and every `speed` field independently) shows the literal string
`"UNSET"` when the raw EEPROM byte has never actually been written (`0xFF`) — this is faithfully showing
"the throttle has not set this yet," not a fabricated current default. Writing `"UNSET"` back on import is
safe and expected; the firmware fills in its real default the next time that slot is loaded.

## Device-settings (`device.json`) fields

Grouped one object per config menu, in top-level-menu-cycle order (`SYSTEM` → `COMM` → `PREFS` →
`THRESHOLD CAL`):

- **`system`**: `battery_okay_decivolts`, `battery_warn_decivolts`, `battery_critical_decivolts` — the
  low-battery thresholds (`BAT OKAY` / `BAT WARN` / `BAT CRIT` in the SYSTEM menu, only shown when
  `ADV FUNC` is on).
- **`comm`**: `mrbus_device_address`, `mrbus_base_address`, `time_source_address`,
  `mrbus_update_interval_decisecs`, `tx_holdoff_centisecs` (`10`-`254` — the firmware heals a stored
  `0xFF` to the default).
- **`prefs`**: `config_bits` (the six booleans `main_screen_speed`, `ops_mode`, `airbrake`,
  `led_blink`, `reverser_lock`, `strict_sleep`, in `PREFS` order — `main_screen_speed` false = main
  screen shows the clock, the default; `ops_mode` false = OPS MODE off, the default; `airbrake` false
  = AIRBRAKE off, the default), then `sleep_timeout_minutes`, `alerter_timeout_minutes`,
  `dead_reckoning_time`.
- **`calibration`**: `horn_threshold`, `horn_threshold2`, `brake_threshold`, `brake_low_threshold`,
  `brake_high_threshold` (the `THRESHOLD CAL` lever-position captures).

These are specific to the radio address and lever calibration of this physical throttle itself — think
twice before importing the `device.json` of one throttle onto a different physical unit. Import also
accepts the older flat shape (all fields at the top level, no category objects).

## Maintaining this tool across EEPROM layout changes

`cst_eeprom_layout.py` and `slot_codec.py` are a **hand-maintained mirror** of `src/cst-eeprom.h` and the
decode logic in `src/mrbw-cst.c` — not generated from them. Whenever the EEPROM layout of the firmware
changes (new field, moved offset, repurposed byte), see the "PC tooling" section ("Maintenance checklist"
subsection) of the top-level `CLAUDE.md` for this repo for the full checklist. The one thing you do **not**
keep in sync by hand is the CNF format version: `SUPPORTED_LAYOUT_VERSION` is parsed from
the `#define EEPROM_LAYOUT_VERSION` in `src/cst-eeprom.h` at import, so bumping that one `#define` is enough for
the tool to refuse loudly against an out-of-sync device instead of silently misdecoding it. The field
tables (`FUNCTION_FIELDS`/`SPEED_FIELDS`/etc.) themselves still have to be updated by hand.

`check_layout_change_bumps_version.py` in this directory + `.githooks/pre-commit` (wired up once
`make setup` has run `git config core.hooksPath .githooks`) is a local safety net that flags a change to
a layout-defining `#define` in `cst-eeprom.h` (the `EE_*` offsets, `CONFIG_*`, `MAX_CONFIGS` /
`WORKING_CONFIG`) that forgot to bump `EEPROM_LAYOUT_VERSION` — not a substitute for the rest of the
checklist, and it does not see a new field added under an unrecognised name or a repurposed byte whose
offset did not move. `git commit --no-verify` bypasses it, same as any git hook.
