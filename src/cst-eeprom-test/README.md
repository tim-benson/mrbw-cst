# EEPROM layout-migration reference-trace test

A host-compiled regression net for the one-shot EEPROM layout migrations in
`../cst-eeprom.c` (`applyEepromMigrations()`, called once from `readConfig()` -
see the "EEPROM layout" note in the root `CLAUDE.md`). Same technique and layout
as `../cst-speed-test/` and `../cst-pressure-test/`.

## Why this one matters

The migrations are the highest-risk, least-verifiable code in the firmware: one
bespoke per-slot byte-remapping transform per `EEPROM_LAYOUT_VERSION` bump, run
once on a throttle that boots newer firmware over an older EEPROM. A wrong offset
in the 2->3 relocation map silently corrupts every one of 20 stored loco
profiles on upgrade, and there is otherwise no way to exercise "a layout-2 chip
boots layout-4 firmware" without a physical throttle holding a specific EEPROM
image. `slot_codec.py` (the PC-tooling mirror) does not model migrations at all.

## What it does

`test_eeprom.c` compiles `../cst-eeprom.c` natively (plain `cc`, not `avr-gcc`)
and runs the real `applyEepromMigrations()` over synthetic pre-migration EEPROM
images - a plain 4096-byte array, built programmatically (no checked-in binary
blobs):

- `buildBlank()` - every byte `0xFF`, a factory-blank chip.
- `buildLayout(v)` - blank, then stamp `EE_LAYOUT_VERSION = v`, then fill every
  profile slot + the working config with a distinguishable per-offset sentinel
  (`0x40 + offset`), so the blessed "after" image shows exactly which source
  byte each migrated byte came from.

For each starting version (`from_blank`, `from_layout1`, `from_layout2`,
`from_layout3`, `from_layout4_noop`) it dumps the post-migration 4096-byte image
(16 bytes/row, all-`0xFF` rows elided) to a plain-text **trace**, one file per
scenario, compared byte-for-byte against the checked-in copies under
`reference/`.

`main()` also asserts four invariants it prints as `PASS`/`FAIL` lines, exiting
non-zero if any fails:

1. a current-layout (`4`) image is left **completely untouched** - zero bytes
   written;
2. the migration is **idempotent** - re-running after a real `2 -> 4` migration
   changes nothing more;
3. a **blank chip becomes a valid layout 4** - the version byte is stamped and
   the five raw-read SPEED bytes (`0x28`/`0x2E`/`0x57`/`0x61`/`0x62`) are seeded
   to their defaults;
4. the **`2 -> 3` relocation preserves every value** - a sentinel at each old
   scattered offset lands at its new `EE_SPEED_MODEL_PAYLOAD` slot.

A difference means the migration output moved - either an intended change (a new
migration block, a fix) and the reference is re-blessed, or an unintended
regression and the code is fixed. Snapshot / characterization / golden-master
testing; "reference trace" is the name used in this tree.

## Running it

From `../` (the `src/` directory):

```
make eepromtest          # build + run + diff against reference/  (PASS / FAIL)
make eepromtest-accept    # regenerate reference/ from the current cst-eeprom.c
```

`make eepromtest` also runs automatically from `.githooks/pre-commit`, but only
when the commit touches `cst-eeprom.c`, `cst-eeprom.h`, or this directory.

After `make eepromtest-accept`, always inspect `git diff` on `reference/` before
committing - that diff is the human-readable statement of exactly how the
migration behaviour changed. Every trace should read as a correct migration:
the version byte stamped to the current `EEPROM_LAYOUT_VERSION`, sentinels
relocated to the addresses the migration comments claim, defaults seeded where a
slot had nothing, and no writes outside the ranges each block owns.

## Layout

| Path | Purpose |
|---|---|
| `test_eeprom.c` | the harness - image builders, the dump, the invariants, `main()` |
| `stubs/avr/eeprom.h` | host shim: `eeprom_read_byte` / `eeprom_write_byte` over the `g_eeprom[4096]` array |
| `stubs/avr/wdt.h` | host shim: `wdt_reset()` as a no-op |
| `reference/*.txt` | the blessed post-migration image for each starting version |
| `out/` | scratch output of the last run (git-ignored) |
| `eepromtest` | the compiled harness (git-ignored) |

The `stubs/` shims are needed because `cst-eeprom.c` includes `<avr/eeprom.h>`
and `<avr/wdt.h>`; the migrations use only the byte-at-a-time eeprom API (no
word/dword/block, no `EEMEM`), so two three-line shims cover it. Every other
header `cst-eeprom.c` pulls (`cst-eeprom.h` / `cst-functions.h` / `cst-speed.h`
/ `cst-pressure.h`) already includes nothing but `<stdint.h>`.

## Scope and fidelity

`applyEepromMigrations()` does only `uint8_t` / `uint16_t` address arithmetic and
byte copies - no signed math, no width-sensitive intermediates - so it is
**identical between AVR 16-bit `int` and host 32-bit `int` by construction**. The
`-Wno-int-to-pointer-cast` in the harness `CFLAGS` only silences the noise from
casting a small integer EEPROM address to `uint8_t*` on a 64-bit host (clean on
AVR, where `int` and pointer are both 16-bit).

The harness passes the pre-stamp `EE_LAYOUT_VERSION` byte to
`applyEepromMigrations()` exactly as `readConfig()` does, so the version-gate
behaviour (stamp-then-no-op, `!= VERSION` vs `< N`) is exercised as shipped.

It covers only `applyEepromMigrations()`. The rest of `readConfig()` - decoding
the *current* layout into RAM globals - is out of scope (and unchanged from its
near-upstream shape once the migrations are factored out here).

Adding a migration block: add it to `applyEepromMigrations()`, add a
`sc_from_layoutN()` scenario and any invariant that locks the new transform, run
`make eepromtest-accept`, review every changed `reference/` file, commit.
