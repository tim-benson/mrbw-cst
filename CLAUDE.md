# CLAUDE.md

Technical reference for **ProtoThrottle X** — the architecture, the per-feature design rationale, the
shared-config wire protocol, and the PC tooling. Read this before working on the firmware; it is the
authoritative description of how each subsystem behaves and why. It also serves as the project
instructions loaded automatically by AI coding assistants (Claude Code and similar).

## Conventions for this document

This is a condensed technical reference, not a development diary. Write in a timeless, descriptive
style — what the code does and why, not the story of how or when it was built. Avoid dated entries,
"confirmed working on hardware on such-and-such date" narrative asides, or references to specific work
sessions. That kind of note belongs in private development notes, not here. Do not use contractions,
colloquialisms, or genitive language (i.e. avoid apostrophes except where required for code or naming).

## What this repo is

Hardware + firmware source for the **MRBW-CST** ("Control Stand Throttle") from Iowa Scaled Engineering —
the wireless throttle sold as the **ProtoThrottle**. The repo bundles the schematics/PCB/mechanical CAD
alongside the AVR firmware; `src/` is the only directory with code in the normal sense. Everything else
(`sch/`, `pcb/`, `fp/`, `sym/`, `mech/`, `doc/`, `pg/`) is gEDA schematic/PCB CAD, 3D-printable parts, and
datasheets/manuals — only relevant if the task is about the physical hardware design, not the firmware.

This is **ProtoThrottle X** (the `X` is for *Extensions*), a fork of `IowaScaledEngineering/mrbw-cst`,
tagged `X<major>.<minor>.<commits>` to keep this fork versioning unambiguous — see "Firmware
versioning" below.

## Build / flash

All commands run from `src/`.

```bash
make setup      # one-time: fetches the src/mrbus git submodule (shared MRBus/MRBee radio library)
make hex        # compile -> mrbw-cst.hex, also copies a versioned copy into src/hex/
make size       # show flash/RAM usage (avr-size)
make disasm     # objdump disassembly of the built .elf, for low-level debugging
make clean      # remove build artifacts
```

Flashing requires a physical AVR programmer wired to the ISP header on the board (default
`PROGRAMMER_TYPE = iseavrprog`; `usbtiny`/`avrispmkii` also supported — edit the top of `src/Makefile` to
switch):

```bash
make fuse       # write fuse bits (only needed once per chip / after a chip erase)
make flash      # flash mrbw-cst.hex
make program    # fuse + flash in one step
make terminal   # open an avrdude terminal against the programmer
```

Target: **ATmega1284P** @ 11.0592 MHz / 3.3V, compiled with `avr-gcc` (`-std=gnu99`), version string and
git hash are baked into the build from `git describe` via `src/git-revision.sh` — the working tree must be
a git checkout (not a tarball) for `make hex` to compute a correct version.

There is no unit test suite and no linter configured for the firmware itself. `src/eep-test/*.py` are
standalone Python scripts (`mrbus.py`, `dumppkts.py`, `test.py`) for sniffing/decoding MRBus/MRBee packets
off the radio for manual debugging — not an automated test harness.

### macOS build environment

A stock macOS machine has neither `avr-gcc` nor `avrdude` preinstalled, and there is no standalone
(non-package-manager) installer for `avr-gcc` on macOS — Homebrew is the practical path:

```bash
brew tap osx-cross/avr   # requires `brew trust osx-cross/avr` first on newer Homebrew versions
brew install avr-gcc     # installs avr-gcc@9 + avr-binutils
brew install avrdude     # official avrdudes/avrdude formula
```

`avrdude` (v8.2+) ships an `iseavrprog` entry in its stock `avrdude.conf` (VID `0x1209`/PID `0x6570`) — no
manual `avrdude.conf` edit is needed to use the ISE AVR Programmer.

`git describe` fails with "No names found, cannot describe anything" on a checkout with no tags reachable
from `HEAD`, which makes `VERSION_STRING` come out as the degenerate `.0+` form — cosmetic only, does not
block the build.

## Reading electrical specifications in this codebase

The hardware directories are **gEDA/gschem**, not KiCad — `.sch`/`.sym`/`.fp` files are plain text, so they
can be grepped directly without opening a CAD tool. To go from a firmware signal name to the physical part
and its datasheet:

1. **Find the net in the schematic.** Firmware ADC/GPIO names usually match schematic net names closely
   (e.g. `ANALOG_VLIGHT_F` in code ↔ `netname=VLIGHT_F` in the `.sch`). `grep -n "netname=VLIGHT_F"
   sch/mrbw-cst.sch` finds every point that net touches.
2. **Find the component on that net.** gEDA component instances look like `C <x> <y> ... <symbol>.sym`
   followed by a `{...}` attribute block with `refdes=`, `device=`, and `footprint=` lines. The net lines
   (`N ...`) and component lines are positioned near each other in the file (same x/y neighborhood), so the
   component block immediately surrounding a net coordinates is normally the one driving/reading it — e.g.
   `VLIGHT_F`/`VLIGHT_R` runs to two `kc14.sym` instances, `refdes=SW6`/`SW7`, `device=KC14`.
3. **Match `device=`/`footprint=` to a datasheet.** `doc/datasheets/*.pdf` filenames are usually the exact
   manufacturer part number or a close prefix (e.g. `device=KC14` → `KC14A10.001NPS.pdf`).
4. **Gerbers and fab outputs** live per-hardware-revision under `pg/<board>-v<rev>-<gitrev>/gerber/`, not in
   `pcb/` (which holds the live/editable `.pcb` gEDA PCB files). `mech/` has 3D-printable enclosure parts
   (`.stl`/`.f3d`).

## Architecture

**`src/mrbw-cst.c` is a single large file** (5,000+ lines) containing `main()`, the main polling loop, the
entire on-device LCD menu system, and the brake/reverser/throttle state machines. There is no RTOS — it is
a bare-metal `while(1)` loop: read hardware inputs → run state machines → build a DCC function bitmask →
push a packet onto the MRBee transmit queue → sleep until the next tick. Supporting `.c`/`.h` pairs factor
out specific subsystems (LCD driver, battery monitoring, EEPROM config, brake-pipe pressure simulation,
tonnage/load sound, fast-clock time sync) but the orchestration logic all lives in `mrbw-cst.c`.

**Function abstraction (`cst-functions.c`/`.h`)**: physical controls (brake lever, horn button, headlight
switch, etc.) are decoupled from DCC output via a `Functions` enum (`BRAKE_FN`, `HORN_FN`, `BELL_FN`, ...).
Each entry is user-configurable through the on-device menu to any DCC function number, in momentary or
latching mode, or special values (`FN_OFF`, `FN_EMRG`). `getFunctionMask()` / `isFunctionOff()` /
`isFunctionLatching()` etc. translate between the logical function and its configured DCC meaning. Adding
new triggerable behavior means adding a `Functions` enum entry and an EEPROM address rather than
hardcoding a DCC function number.

**`functionMask` is rebuilt from scratch every main-loop pass** (`mrbw-cst.c`, `functionMask = 0;` near the
top of the packet-build section): every logical function bit is independently re-derived each iteration
from whatever the current live state is (`controls` bits, `activeReverserSetting`, `engineState`, etc.),
then transmitted. There is no "pending pulse" bookkeeping at this stage — a function stays asserted for
exactly as long as its underlying condition stays true, and drops the instant it does not. `NEUTRAL_FN`
("reverser center") is the clearest example: `if(NEUTRAL == activeReverserSetting) functionMask |=
getFunctionMask(NEUTRAL_FN);` — continuously held while the reverser sits centered, the same
*continuous-hold-while-condition-is-true* idiom used by `BRAKE_CONTROL`/`BRAKE_OFF_CONTROL` in
standard/pulse brake mode, just checked directly against live state instead of through an intermediate
`controls` bit set by a state machine. This is the dominant pattern for persistent (non-pulsed) behavior in
this codebase.

**EEPROM layout (`cst-eeprom.h`)**: a small fixed global block, plus up to `MAX_CONFIGS` (20) numbered
128-byte config slots so one throttle can store multiple loco/road-number profiles. The currently active
profile is copied into a scratch "working config" slot (`WORKING_CONFIG = 31`) at the end of EEPROM, and
all the `EE_*_FUNCTION`, threshold, and option-byte addresses are computed relative to
`CONFIG_OFFSET(WORKING_CONFIG)`. A blank, never-configured chip reads every EEPROM byte as `0xFF`; most
fields are read through `readByteOrDefault()`, which detects that sentinel and substitutes+persists a real
default rather than trusting a plainly-invalid raw byte.

**The LCD is only 8 columns × 2 rows** (`LCD_DISP_LENGTH` in `lcd.h`, `= 8`) — easy to mistake for wider
given the unrelated `CANVAS_COLS 20` constant in `cst-pressure.c` (a custom-bitmap-canvas size, not the
physical display width). `lcd_gotoxy(x,y)`/`lcd_puts()` do no bounds checking: writing at column ≥8 does
not clip or wrap visibly, it corrupts into the DDRAM of the other row. The cramped-looking choices in the
existing screens are not style, they are this hard ceiling — e.g. boolean prefs show `" ON "`/`" OFF"` (4
chars, not `"ON"`/`"OFF"`) because only columns 4-7 are left after their `lcd_gotoxy(4,1)`. Any new menu
text needs its start column + length checked against this 8-column limit before assuming a wider
terminal-width mental model.

**`src/mrbus/` is a git submodule** providing the shared MRBus/MRBee packet queue, CRC, and radio driver
(`mrbee-avr.c`) used across the whole ISE product line. Treat it as vendored/external — it must be fetched
with `make setup` before building.

## Firmware versioning

Tags follow `X<major>.<minor>` (e.g. `X1.0`), created only at meaningful milestones — the `X` prefix
stands for *Extensions* (the fork is "ProtoThrottle X" — extensions to the stock ProtoThrottle firmware)
and unambiguously marks this as a distinct fork rather than colliding with any numbering the upstream ISE
repo might use, and is a literal git-visible tag component, not just a display-time label. The third
(patch/build) number is not a separate tag — it auto-increments with every commit since the last matching
tag (`git rev-list <tag>..HEAD --count`), `+` appended for a dirty working tree (e.g. `X1.0.1`, `X1.0.7+`).
The tag-match glob patterns in `src/git-revision.sh` and the leading-alpha skip in `parseVersionStr()`
(`mrbw-cst.c`) are the two places this convention is implemented; `VERSION_STRING` (including the `X`
prefix) fits the 8-column LCD in the worst case (`"X9.9.999"` = exactly 8 characters).

**Known, accepted limitation**: the commits-since-tag delta is a `uint8_t`, written directly into one byte
of the outgoing MRBus `'v'` status-query response packet — past 255 commits since a tag, that wire-protocol
byte wraps. Neither this repo nor `mrbw-cabbus` reads/consumes that field from a `'v'` packet today, so
this is a low-priority, wire-protocol-only quirk rather than a live bug.
