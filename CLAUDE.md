# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

Hardware + firmware source for Iowa Scaled Engineering's **MRBW-CST** ("Control Stand Throttle") — the
wireless throttle sold as the **ProtoThrottle**. The repo bundles the schematics/PCB/mechanical CAD
alongside the AVR firmware; `src/` is the only directory with code in the normal sense. Everything else
(`sch/`, `pcb/`, `fp/`, `sym/`, `mech/`, `doc/`, `pg/`) is gEDA schematic/PCB CAD, 3D-printable parts, and
datasheets/manuals — only relevant if the task is about the physical hardware design, not the firmware.

## Build / flash commands

All commands run from `src/`.

```bash
make setup      # one-time: fetches the src/mrbus git submodule (shared MRBus/MRBee radio library)
make hex        # compile -> mrbw-cst.hex, also copies a versioned copy into src/hex/
make size       # show flash/RAM usage (avr-size)
make disasm     # objdump disassembly of the built .elf, for low-level debugging
make clean      # remove build artifacts
```

Flashing requires a physical AVR programmer wired to the board's ISP header (default `PROGRAMMER_TYPE =
iseavrprog`; `usbtiny`/`avrispmkii` also supported — edit the top of `src/Makefile` to switch):

```bash
make fuse       # write fuse bits (only needed once per chip / after a chip erase)
make flash      # flash mrbw-cst.hex
make program    # fuse + flash in one step
make terminal   # open an avrdude terminal against the programmer
```

Target: **ATmega1284P** @ 11.0592 MHz / 3.3V, compiled with `avr-gcc` (`-std=gnu99`), version string and
git hash are baked into the build from `git describe` via `src/git-revision.sh` — the working tree must be
a git checkout (not a tarball) for `make hex` to compute a correct version.

There is no unit test suite and no linter configured. `src/eep-test/*.py` are standalone Python scripts
(`mrbus.py`, `dumppkts.py`, `test.py`) for sniffing/decoding MRBus/MRBee packets off the radio for manual
debugging — not an automated test harness.

### macOS build environment (confirmed working, 2026-08-11)

A stock macOS machine has neither `avr-gcc` nor `avrdude` preinstalled, and there's no standalone
(non-package-manager) installer for `avr-gcc` on macOS — Homebrew is the practical path:

```bash
brew tap osx-cross/avr   # requires `brew trust osx-cross/avr` first on newer Homebrew versions
brew install avr-gcc     # installs avr-gcc@9 + avr-binutils
brew install avrdude     # official avrdudes/avrdude formula
```

`avrdude` (v8.2, installed via the formula above) already ships with an `iseavrprog` entry in its stock
`avrdude.conf` (VID `0x1209`/PID `0x6570`) — no manual `avrdude.conf` edit is needed to use the ISE AVR
Programmer; the note in earlier versions of this doc about adding that entry by hand is now stale.

`git describe` fails with "No names found, cannot describe anything" on a checkout with no tags, which
makes `VERSION_STRING` come out as `.0+` instead of a real version — cosmetic only, doesn't block the
build, but means the on-device version string is meaningless until the repo has at least one git tag.

End-to-end `make hex` → `make flash` against real MRBW-CST hardware via an ISE AVR Programmer has been
verified working with this setup (flash write + read-back verify both succeeded).

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
   component block immediately surrounding a net's coordinates is normally the one driving/reading it —
   e.g. `VLIGHT_F`/`VLIGHT_R` runs to two `kc14.sym` instances, `refdes=SW6`/`SW7`, `device=KC14`.
3. **Match `device=`/`footprint=` to a datasheet.** `doc/datasheets/*.pdf` filenames are usually the exact
   manufacturer part number or a close prefix (e.g. `device=KC14` → `KC14A10.001NPS.pdf`, the full E-Switch
   part number; `doc/datasheets/KC.pdf` is the same manufacturer's general series catalog page, useful when
   the exact-match PDF is silent on something like timing behavior). Read PDFs directly with the Read tool.
4. **Gerbers and fab outputs** live per-hardware-revision under `pg/<board>-v<rev>-<gitrev>/gerber/`, not
   in `pcb/` (which holds the live/editable `.pcb` gEDA PCB files). `mech/` has 3D-printable enclosure
   parts (`.stl`/`.f3d`) — physical/mechanical rather than electrical specs.

This path is how the KC14A10.001NPS rotary switch (used for both light knobs) was identified and confirmed
as break-before-make directly from its datasheet — see "Planned: light-knob debounce fix" below.

## Architecture

**`src/mrbw-cst.c` is a single ~4,200-line file** containing `main()`, the main polling loop, the entire
on-device LCD menu system, and the brake/reverser/throttle state machines. There's no RTOS — it's a
bare-metal `while(1)` loop: read hardware inputs → run state machines → build a DCC function bitmask →
push a packet onto the MRBee transmit queue → sleep until the next tick. Supporting `.c`/`.h` pairs factor
out specific subsystems (LCD driver, battery monitoring, EEPROM config, brake-pipe pressure simulation,
tonnage/load sound, fast-clock time sync) but the orchestration logic all lives in `mrbw-cst.c`.

**Function abstraction (`cst-functions.c`/`.h`)**: physical controls (brake lever, horn button, headlight
switch, etc.) are decoupled from DCC output via a `Functions` enum (`BRAKE_FN`, `HORN_FN`, `BELL_FN`, ...).
Each entry is user-configurable through the on-device menu to any DCC function number, in momentary or
latching mode, or special values (`FN_OFF`, `FN_EMRG`). `getFunctionMask()` / `isFunctionOff()` /
`isFunctionLatching()` etc. translate between the logical function and its configured DCC meaning. When
adding new triggerable behavior, add a `Functions` enum entry and an EEPROM address rather than hardcoding
a DCC function number.

**`functionMask` is rebuilt from scratch every main-loop pass** (`mrbw-cst.c`, starts at `functionMask = 0;`
around line 3819): every logical function's bit is independently re-derived each iteration from whatever
the current live state is (`controls` bits, `activeReverserSetting`, `engineState`, etc.), then transmitted.
There's no "pending pulse" bookkeeping in this stage — a function stays asserted for exactly as long as its
underlying condition stays true, and drops the instant it doesn't. `NEUTRAL_FN` ("reverser center") is the
clearest example: `if(NEUTRAL == activeReverserSetting) functionMask |= getFunctionMask(NEUTRAL_FN);`
([mrbw-cst.c:3895-3898](src/mrbw-cst.c#L3895-L3898)) — continuously held while the reverser sits centered,
same *continuous-hold-while-condition-is-true* idiom as standard/pulse brake mode's `BRAKE_CONTROL`/
`BRAKE_OFF_CONTROL`, just checked directly against live state instead of through an intermediate `controls`
bit set by a state machine. This is the dominant pattern for persistent (non-pulsed) behavior in this
codebase — relevant precedent for STACK mode's release logic and combo outputs, both of which should follow
it rather than stepped mode's deliberate one-tick-pulse pattern.

**EEPROM layout (`cst-eeprom.h`)**: a small fixed global block, plus up to `MAX_CONFIGS` (20) numbered
128-byte config slots so one throttle can store multiple loco/road-name profiles. The currently active
profile is copied into a scratch "working config" slot (`WORKING_CONFIG = 31`) at the end of EEPROM, and
all the `EE_*_FUNCTION`, threshold, and option-byte addresses are computed relative to
`CONFIG_OFFSET(WORKING_CONFIG)`. Adding a new persisted setting means picking an unused offset here and
wiring up read/write calls in `mrbw-cst.c`'s `readConfig()`/save paths.

**Brake logic** (`mrbw-cst.c`, roughly lines 1180–1363, mirrored again around line 3340 for menu display):
a `brakeState` state machine (`BrakeStates` enum: `BRAKE_LOW_BEGIN` ... `BRAKE_FULL_WAIT`) drives two
outputs, `BRAKE_CONTROL` and `BRAKE_OFF_CONTROL`, which map to the `BRAKE_FN`/`BRAKE_OFF_FN` logical
functions. Which variant runs is selected by two bits in `optionBits` (set via the on-device menu, no
firmware change needed):
- `OPTIONBITS_VARIABLE_BRAKE` off → **standard**: on/off threshold with hysteresis.
- `OPTIONBITS_VARIABLE_BRAKE` on + `OPTIONBITS_STEPPED_BRAKE` off → **pulse**: duty-cycle pulses
  `BRAKE_CONTROL` proportional to lever percentage, period set by `brakePulseWidth` (`BRK RATE` in menu).
- `OPTIONBITS_VARIABLE_BRAKE` on + `OPTIONBITS_STEPPED_BRAKE` on → **stepped**: advances through
  20/40/60/80/100% bands only as the lever *increases*; only a full return to the bottom resets it (this
  is the TCS-decoder-friendly mode — comments in the code call this out explicitly).

`EE_BRAKE_THRESHOLD` / `EE_BRAKE_LOW_THRESHOLD` / `EE_BRAKE_HIGH_THRESHOLD` are per-profile calibration
values set through the device's threshold-calibration menu screens, not compile-time constants.

**`src/mrbus/` is a git submodule** (github.com/IowaScaledEngineering/mrbus) providing the shared
MRBus/MRBee packet queue, CRC, and radio driver (`mrbee-avr.c`) used across ISE's whole product line.
Treat it as vendored/external — it must be fetched with `make setup` before building, and changes to it
belong upstream, not in this repo.

## Implemented: "STACK" combo brake mode (confirmed working on hardware, 2026-08-12)

A fourth brake mode where the lever's percentage drives combinations of **three new DCC functions**
(`BK1_FN`/`BK2_FN`/`BK3_FN`, menu names "BRAKE1"/"BRAKE2"/"BRAKE3", mapped to ESU LokSound V5 brake
functions that stack) instead of pulsing/stepping a single function. Unlike stepped mode, combos track the
lever **symmetrically in both directions**, and each combo **persists** (holds continuously) rather than
pulsing — same persistence style as standard/pulse mode, just with 3 outputs instead of 1.
`OPTIONBITS_ESTOP_ON_BRAKE` is never enabled alongside this mode, so no interaction to design around there;
full-lever BK1+BK2+BK3 is deliberately reserved by the user for a separate, unrelated emergency-stop
function, not part of this mode.

The lever is divided into **6 equal 16.67%-wide bands** (`STACK_BAND_COUNT`, `mrbw-cst.c`). Band 0
(0–16.67%, full-left) and band 5 (83.3–100%, full-right) are pinned to the lever extremes by design. Band
cut-points live in `stackBandThresholds[STACK_BAND_COUNT - 1]`, an explicit ordered array rather than a
formula or hardcoded conditionals, since boundaries are expected to become uneven later (e.g. band 0
narrowing to 10%) — that stays a one-line data change. The band → combo mapping
(`stackBandCombos[STACK_BAND_COUNT]`) is currently a **placeholder** (confirmed mechanically working in
on-device testing, but the actual combo-per-band assignments haven't been finalized against real
decoder/consist behavior yet):

| Band | Lever range | BK1 | BK2 | BK3 |
|---|---|:---:|:---:|:---:|
| 0 | 0–16.67% | – | – | – |
| 1 | 16.67–33.3% | ✓ | – | – |
| 2 | 33.3–50% | – | ✓ | ✓ |
| 3 | 50–66.7% | ✓ | ✓ | – |
| 4 | 66.7–83.3% | ✓ | – | ✓ |
| 5 | 83.3–100% | ✓ | ✓ | ✓ |

Implementation, as built:
- `BK1_FN`/`BK2_FN`/`BK3_FN` added to the `Functions` enum + `functions[]` table (`cst-functions.h`/`.c`),
  "plain" (no `.attributes`) like `BRAKE_FN` — they get a working "Configure Function" menu page for free,
  no new menu screen code, since that screen drives generically off `functions[]`/`LAST_FN`.
- New EEPROM addresses `EE_BK1_FUNCTION`/`EE_BK2_FUNCTION`/`EE_BK3_FUNCTION` at offsets `0x28`-`0x2A`
  (`cst-eeprom.h`), inside the previously-free 8-byte gap; 5 bytes remain free there afterward.
- New mode is **stateless per loop** (`evaluateStackBrake(brakePcnt)` in `mrbw-cst.c`, called from the main
  brake-mode dispatch) — *not* a graft onto the existing `BrakeStates` state machine, which is deliberately
  asymmetric (advance-only, TCS-style) and the wrong shape here. It re-derives the correct band fresh from
  `brakePcnt` on every call (escalate immediately crossing a threshold going up; de-escalate only once
  `BRAKE_HYSTERESIS` below that same threshold coming back down — the same dead-band idiom **basic on/off
  mode** uses at its one boundary, generalized from 1 boundary/2 states to 5 boundaries/6 bands; **not**
  pulse mode's idiom, which PWM-modulates a single output against a free-running counter and was never
  banded in the first place — see `mrbw-cst.c:1355-1364` for that mechanism if it comes up again). Writes to
  a **new dedicated global** `brakeComboControls` (bits `BK1_CONTROL`/`BK2_CONTROL`/`BK3_CONTROL`), not the
  existing `controls` byte, which only had 2 free bits. A guard clears `brakeComboControls`/
  `currentStackBand` whenever STACK isn't the active mode, so switching away mid-combo can't leave a stale
  combo stuck asserted.
- Because the evaluation function resolves however many band-boundaries got crossed within a single call,
  a fast lever sweep **drops** intermediate bands' combos rather than queuing/transmitting them — only the
  band the lever is actually in when sampled ever gets asserted or transmitted. The real sampling interval
  for `brakePcnt` is the shared ADC round-robin refresh (~60-100ms per channel, same one used for the light
  knobs/reverser), not the raw main-loop rate.
- `BRK TYPE` converted from a single boolean bit (`OPTIONBITS_STEPPED_BRAKE`) into a genuine 3-way cycle
  (`PULSE`/`STEP`/`STACK`) using a new 2-bit `optionBits` field (`OPTIONBITS_BRK_TYPE_LSB`, bits 3-4;
  `GET_BRK_TYPE`/`SET_BRK_TYPE` macros) — `optionBits` only used bits 0-3 before, so this was a pure
  bit-budget non-issue, just new UI cycling logic in `OPTION_SCREEN` (no N-way cycling widget existed to
  reuse anywhere in the menu system beforehand). Existing on-device configs stay valid across this change
  without a reset, since the old boolean's bit becomes the new field's LSB and the new MSB bit was always 0.
- `BRK RATE` is now hidden for both `STEP` and `STACK` (meaningless for either non-pulsing mode).
- Wired into `functionMask` alongside the existing brake lines, and mirrored on the diagnostic display
  (`OFF `/`BND1`…`BND5`, using `currentStackBand`).
- Strictly additive: existing standard/pulse/step code paths were not modified, and the new mode defaults
  off, so it can't affect existing behavior until deliberately selected on-device.
- **Brake release (`BRAKE_OFF_FN`) in STACK mode behaves like standard/pulse mode, not stepped mode**: held
  continuously for as long as the lever sits in band 0, cleared otherwise — not a one-tick pulse like
  stepped mode's release. Reuses the existing `BRAKE_OFF_CONTROL`/`BRAKE_OFF_FN` plumbing; `evaluateStackBrake()`
  just sets/clears that bit itself each call.

**DCC packet economy note** (informational, not implemented here — a Configure Function choice the user
makes, not a firmware concern): NMRA DCC groups function numbers into fixed packet groups (F0-F4, F5-F8,
F9-F12, F13-F20, F21-F28). Assigning BK1/BK2/BK3 within a single group (F9-F12 fits all three with a bit to
spare) lets a downstream command station potentially fold a simultaneous multi-brake transition into one
DCC packet instead of up to three. This repo's firmware always sends the full function bitmask as one
MRBus/MRBee packet regardless of function number choice — the DCC-side economy depends entirely on the
downstream command station/gateway, not on this codebase.

**Not yet built** (architected for, not built): an on-device band→combo editor. `evaluateStackBrake()`
already reads `stackBandCombos[]` as a data table rather than inline logic, so this would only need the
table moved into a new EEPROM block plus a new bespoke menu screen (modeled on `FORCE_FUNC_SCREEN`) — not a
rewrite of the evaluation logic itself.

Flashing hardware confirmed compatible: the user's [ISE AVR Programmer](https://www.iascaled.com/store/CKT-AVRPROGRAMMER)
is a USBtinyISP-protocol device (VID `0x1209`/PID `0x6570`) — exactly what `PROGRAMMER_TYPE = iseavrprog`
in the Makefile already targets. Works out of the box with ISE's MRGui utility, or with `make flash`
directly — modern `avrdude` (v8.2+) ships an `iseavrprog` entry in its stock `avrdude.conf` already, see
"macOS build environment" above. End-to-end `make hex` → `make flash` verified working against real
hardware.

## Implemented: light-knob debounce fix (confirmed working on hardware, 2026-08-12)

Bug: twisting either light knob (front or rear) can briefly cause **no light function to be asserted at
all** — a visible dark flicker at the decoder. User was working around this with logic on the decoder side
(ignore a function change, wait ~1s, recheck, only act if it held); confirmed in testing that this decoder
workaround is no longer needed once fixed at the source (below).

Root cause (see "Architecture" above for the underlying read mechanism): `frontLight`/`rearLight`
(`cst-hardware.c:378-387` / `:411-420`) are classified from a raw ADC reading into one of 4 discrete bands
(`LIGHT_OFF`/`LIGHT_DIM`/`LIGHT_BRIGHT`/`LIGHT_BRIGHT_DITCH`) with **hard thresholds and no hysteresis**,
refreshed roughly every 60-100ms. The light knob is a detented rotary switch read via a resistor ladder
(same technique as the reverser), not a continuous pot like brake/horn — twisting between detents involves
a real break-before-make moment where the ADC input floats and can transiently read as *any* band,
including non-adjacent ones like `LIGHT_OFF`. Since `LIGHT_OFF` asserts zero function bits
(`mrbw-cst.c:3902-3936`) and there's no filtering, a single bad sample gets transmitted as-is.

**Hardware confirmation (not just inferred from firmware behavior):** tracing `VLIGHT_F`/`VLIGHT_R` in
`sch/mrbw-cst.sch` identifies the physical parts as `refdes=SW6` (front) and `refdes=SW7` (rear),
`device=KC14`, matched to `doc/datasheets/KC14A10.001NPS.pdf` — an **E-Switch KC14A10.001NPS**, a
4-position/30°-index rotary switch. Its contact configuration is labeled on the manufacturer drawing itself
as **"TIMING (BBM) NON-SHORTING"** (break-before-make). That's a direct datasheet confirmation that the
open-circuit transition between detents is a designed-in property of this exact part, not just noise —
strengthens the case for debounce over hysteresis/EMA (see below). The datasheet gives no numeric contact
transition/bounce duration (mechanical rotary switch specs generally don't, since it's actuation-speed
dependent), so the ~150-300ms / 2-3 sample debounce window below is still an engineering estimate, not a
spec'd figure. See "Reading electrical specifications in this codebase" above for how this was traced.

Two approaches considered and rejected in favor of debounce:
- **Hysteresis** (like `BRAKE_HYSTERESIS`/`HORN_HYSTERESIS`) only stops a value oscillating *at* a
  boundary between two adjacent states — it doesn't stop a floating/transient reading from landing in a
  non-adjacent band, which is the actual failure mode here.
- **Reviving the disabled EMA smoothing filter already sitting commented-out in the code**
  (`cst-hardware.c:388-401`, `:421-434`) — rejected because it's the wrong model for a switch with a real
  open-circuit transition (it's suited to continuously-noisy analog signals like the brake/horn pots), it
  adds latency to *every* light change proportionally rather than just the glitchy ones, and its constants
  are hand-tuned ("determined experimentally") rather than derived from the actual failure mode.

**Implemented approach: debounce/confirm on the already-classified `LightPosition` state, not the raw
voltage.** A new `frontLight`/`rearLight` value is only committed (and so only transmitted) once the same
classification has been read for `LIGHT_DEBOUNCE_THRESHOLD` (3) consecutive ADC cycles; a disagreeing
candidate reading is held, not acted on, until it either repeats enough times to replace the committed value
or another reading overrides the candidate. A single-cycle transient glitch never survives long enough to be
transmitted; a genuine settled knob position does. Applied identically to both channels via a
candidate+counter static pair per channel inside `processADC()`, in the `ADC_STATE_READ_VLIGHT_F` /
`ADC_STATE_READ_VLIGHT_R` cases in `cst-hardware.c`; the previously-commented-out EMA smoothing code in
those cases was deleted rather than left alongside the new logic. **Confirmed in on-hardware testing to
fully resolve the flicker.**

Related but out of scope for now: the reverser (`ADC_STATE_READ_VREV`, `cst-hardware.c:333-348`) uses the
identical raw-threshold-no-smoothing pattern and could in principle exhibit the same class of transient
glitch — not reported as an issue, not being touched as part of this fix, just noted as the same pattern
existing elsewhere.

Minor unrelated cleanup noticed along the way: `cst-hardware.h:57` and `:59` declare `extern uint8_t
frontLightPot;` / `rearLightPot`, which are never defined or used anywhere — dead leftovers from a rename
to `frontLightValue`/`rearLightValue`.
