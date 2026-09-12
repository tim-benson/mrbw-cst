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
boots current firmware" without a physical throttle holding a specific EEPROM
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
`from_layout3`, `from_layout4`, `from_layout5`, `from_current_noop`) it dumps the
post-migration 4096-byte image (16 bytes/row, all-`0xFF` rows elided) to a
plain-text **trace**, one file per scenario, compared byte-for-byte against the
checked-in copies under `reference/`. `from_layout5` is a frozen historical
fixture (built from the literal value `5`, the layout this scenario has always
tested from) and is never rewritten to track later bumps; `from_current_noop` is
the opposite - built from the live `EEPROM_LAYOUT_VERSION` macro, so it keeps
meaning "a chip already on the current layout must see zero writes" across every
future version bump without needing to be hand-updated itself (see "Scope and
fidelity" below for why this distinction matters). A further scenario,
`reset_model`, does the same for `eepromResetProfileModel()` (the factory-default
writer `resetConfig()` uses) run over a sentinel-filled working-config slot.

`main()` also asserts seven invariants it prints as `PASS`/`FAIL` lines, exiting
non-zero if any fails:

1. a current-layout image is left **completely untouched** - zero bytes written;
2. the migration is **idempotent** - re-running after a real `2 -> current`
   migration changes nothing more;
3. a **blank chip becomes a valid current layout** - the version byte is stamped,
   the five raw-read SPEED bytes (`0x28`/`0x2E`/`0x57`/`0x61`/`0x62`) are seeded
   to their defaults, and the MENU BTN / SEL BTN function slots (`0x2C`/`0x2D`)
   are seeded to `FN_OFF`;
4. the **`2 -> 3` relocation preserves every value** - a sentinel at each old
   scattered offset lands at its new `EE_SPEED_MODEL_PAYLOAD` slot;
5. **`eepromResetProfileModel()` covers every model offset** - each of `0x28-0x62`
   is a model field (set to its default), a function slot, or a freed hole, and
   the three sets partition the range exactly (a new field missed here, or a
   reused hole, fails);
6. `eepromResetProfileModel()` is **confined** - it writes only the model
   offsets, nothing else in the slot or the image;
7. the reset and the `< 2` migration **agree** on the `0x54-0x60` SPEED payload
   defaults (the two default sources in `cst-eeprom.c` must not drift).

A difference means the migration or reset output moved - either an intended
change (a new migration block, a new field, a fix) and the reference is
re-blessed, or an unintended regression and the code is fixed. Snapshot /
characterization / golden-master testing; "reference trace" is the name used in
this tree.

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
**A per-transform gate must always compare against a fixed number (`< N`), never
against the live `EEPROM_LAYOUT_VERSION` macro** - the layout 4->5 migration
originally used `!= EEPROM_LAYOUT_VERSION`, which happened to work only for as
long as that macro equalled 5; the first later bump (Menu Customisation, -> 6)
made a layout-5 chip satisfy `5 != 6` again and silently re-wipe its already-
migrated `MENU BTN` / `SEL BTN` function slots. `from_layout5`/`from_current_noop`
(above) exist specifically to keep catching this class of bug on every future
bump - only the top-level version-stamp write is correct to compare against the
live macro, since it is unconditionally "did the version actually change."

It covers `applyEepromMigrations()` and `eepromResetProfileModel()`. The rest of
`readConfig()` / `resetConfig()` - decoding the *current* layout into RAM
globals, and the non-model per-profile / global-config writes - is out of scope.

Adding a migration block: gate it on a fixed `oldLayoutVersion < N` (plus
`|| (0xFF == oldLayoutVersion)` for the blank-chip case), never on
`!= EEPROM_LAYOUT_VERSION` - see "Scope and fidelity" above for why. Add it to
`applyEepromMigrations()`, add a `sc_from_layoutN()` scenario and any invariant
that locks the new transform, run
`make eepromtest-accept`, review every changed `reference/` file, commit.

Adding a per-profile SPEED/AIRBRAKE/STACK model field: add its offset to
`eepromResetProfileModel()` and to `resetModel_check[]` in `test_eeprom.c` (if
it reuses a freed hole, also move that offset out of `isFreedHole()`), run
`make eepromtest-accept`, review the `reset_model.txt` diff.
