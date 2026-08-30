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

This path is how the light-knob part (E-Switch KC14A10.001NPS) was identified and confirmed as
break-before-make directly from its datasheet — see "Light-knob debounce" below.

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

**Brake logic**: see "Brake logic" below.

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

## Light-knob debounce

The light knob (front and rear, `cst-hardware.c`) is a detented rotary switch read via a resistor ladder —
confirmed from its datasheet (E-Switch KC14A10.001NPS) to be "TIMING (BBM) NON-SHORTING"
(break-before-make): twisting between detents involves a real open-circuit moment where the ADC input
floats and can transiently read as *any* band, including non-adjacent ones like `LIGHT_OFF`. Since
`LIGHT_OFF` asserts zero function bits and the classification originally had no filtering, a single bad
sample got transmitted as-is — a visible dark flicker at the decoder.

Hysteresis was rejected (it only stops oscillation *at* a boundary between adjacent states, not a
transient landing in a non-adjacent one) in favor of **debounce on the already-classified state**: a new
`frontLight`/`rearLight` value is only committed (and so only transmitted) once the same classification has
held for roughly `LIGHT_DEBOUNCE_THRESHOLD + 2` consecutive ADC cycles (~5 with the default of 3 — the
counter saturates at the threshold, then one further matching read commits); a disagreeing candidate
reading is held, not acted on, until it either repeats enough times to replace the committed value or
another reading overrides the candidate. Applied via a candidate+counter static pair per channel inside `processADC()`
(`ADC_STATE_READ_VLIGHT_F`/`ADC_STATE_READ_VLIGHT_R` in `cst-hardware.c`).

The reverser (`ADC_STATE_READ_VREV`) uses the identical raw-threshold-no-smoothing pattern and could in
principle exhibit the same class of transient glitch — not reported as an issue, not currently addressed.

## Brake logic

`brakeState` (`BrakeStates` enum: `BRAKE_LOW_BEGIN` ... `BRAKE_FULL_WAIT`, `mrbw-cst.c`) drives two
outputs, `BRAKE_CONTROL` and `BRAKE_OFF_CONTROL`, which map to the `BRAKE_FN`/`BRAKE_OFF_FN` logical
functions. Which variant runs is selected by `BRK TYPE`, a 3-way cycle in `optionBits`
(`OPTIONBITS_BRK_TYPE_LSB`, bits 3-4; `GET_BRK_TYPE`/`SET_BRK_TYPE` macros), set via the on-device menu:

- **Standard** (`BRK TYPE` variable-brake bit off): on/off threshold with hysteresis.
- **Pulse**: duty-cycle pulses `BRAKE_CONTROL` proportional to lever percentage, period set by
  `brakePulseWidth` (`BRK RATE` in menu).
- **Step**: advances through 20/40/60/80/100% bands only as the lever *increases*; only a full return to
  the bottom resets it (a TCS-decoder-friendly mode, since some decoder brake implementations only step
  forward).
- **Stack**: see below.

`EE_BRAKE_THRESHOLD` / `EE_BRAKE_LOW_THRESHOLD` / `EE_BRAKE_HIGH_THRESHOLD` are global (once-per-throttle,
not per-profile — same as `EE_HORN_THRESHOLD`) calibration values set through the on-device
threshold-calibration menu screens, not compile-time constants.
`OPTIONBITS_ESTOP_ON_BRAKE` ("BRK ESTP") is a mode-independent check that runs *before* the brake-mode
dispatch, comparing raw `brakePosition` against `brakeLowThreshold`/`brakeHighThreshold` directly — pushing
the lever to max triggers the throttle built-in emergency stop (if enabled) regardless of `BRK TYPE`,
clearing only once the lever returns fully to the bottom.

### STACK combo brake mode

A fourth mode where lever percentage drives combinations of **three DCC functions** — "Brake1" (reuses the
existing `BRAKE_FN`/`BRAKE_CONTROL` plumbing directly), "Brake2", "Brake3" (`BK2_FN`/`BK3_FN`) — mapped to
e.g. ESU LokSound/LokPilot V5 brake functions that stack, instead of pulsing/stepping a single function. Combos
track the lever symmetrically in both directions, and each combo persists (holds continuously) rather than
pulsing, matching the standard/pulse mode idiom rather than the stepped mode one.

Two step-count variants, selected by a `STEPS` toggle (3-STEP default, 5-STEP option; existing
combo config for whichever variant is not active stays stored and reactivates if switched back): the lever
divides into `stackBandCount()` equal-width bands (4 for 3-STEP, 6 for 5-STEP, both counts including band
0). Band 0 (full-left) and the top band (full-right) are pinned to the lever extremes; band 0 is
permanently fixed to "no combo" and asserts `BRAKE_OFF_FN` continuously (like the standard/pulse mode
release, not the stepped mode one-tick pulse) — it is not stored or editable. An interior band combo of
`0x00` ("none active") is distinct from band 0: it asserts neither a brake-on combo nor `BRAKE_OFF_FN`.
Band cut-points live in the explicit ordered array in `stackThresholds()` rather than a formula, since
boundaries could change to become uneven in the future.

Combo evaluation is **stateless per loop** (`evaluateStackBrake(brakePcnt)`, called from the main
brake-mode dispatch) — not a graft onto the `BrakeStates` state machine, which is deliberately asymmetric
(advance-only, TCS-style) and the wrong shape here. It re-derives the correct band fresh from `brakePcnt`
on every call: escalate immediately on crossing a threshold going up, de-escalate only once
`BRAKE_HYSTERESIS` below that same threshold coming back down — the same dead-band idiom basic on/off mode
uses at its one boundary, generalized to as many boundaries as the active variant has. All 3 combo bits
live directly in the `controls` byte — `BRAKE_CONTROL` (reused) for Brake1, `BK2_CONTROL`/
`BK3_CONTROL` for Brake2/Brake3. A guard right after the brake-mode dispatch clears
`BK2_CONTROL`/`BK3_CONTROL` (and resets the sticky `currentStackBand`) whenever STACK is not active. Because
evaluation resolves however many band-boundaries got crossed within a single call, a fast lever sweep drops
the combos of intermediate bands — only the band the lever is actually in when sampled is ever asserted.

The band→combo mapping is edited live inside `OPTION_SCREEN`, revealed only when `BRK TYPE = STACK` (item
numbering shifts dynamically based on the editable band count of the active variant). `stackBandCombos3Step[]`/
`stackBandCombos5Step[]` are RAM arrays populated from separate EEPROM blocks (band 0 hardcoded to `0x00`,
never stored). `UP`/`DOWN` cycle each band through all 8 possible 3-bit combos (none, each single, each
pair, all three) via a `stackComboSequence[8]` lookup table. Display is `STEP1`…`STEPn` / `BRAKE---`…
`BRAKE123` — "band" is the internal term (includes the off band); "step" is the on-device end-user term for
the editable bands only.

The pre-existing Brake Test screen (the pressure-gauge subscreen of `SPECFN_SCREEN`, reached via a control
configured to `FN_BRKTEST`) suppresses `BRAKE_CONTROL`/`BRAKE_OFF_CONTROL`/`BK2_CONTROL`/`BK3_CONTROL` and
the lever-triggered e-stop while its simulated pressure/sound sequence is active, so moving the lever
during a test does not also fire real brake functions or a genuine e-stop.

**DCC packet economy note**: NMRA DCC groups function numbers into fixed packet groups (F0-F4, F5-F8,
F9-F12, F13-F20, F21-F28); assigning Brake1/2/3 within one group (F9-F12 fits all three) lets a downstream
command station fold a simultaneous multi-brake transition into one DCC packet instead of up to three. This
is a Configure Function choice the user makes — the firmware always sends the full function bitmask as one
MRBus/MRBee packet regardless.

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
