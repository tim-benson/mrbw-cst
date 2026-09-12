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
make setup           # one-time: fetches the src/mrbus git submodule (shared MRBus/MRBee radio library)
make hex             # compile -> mrbw-cst.hex, also copies a versioned copy into src/hex/
make size            # show flash/RAM usage (avr-size)
make disasm          # objdump disassembly of the built .elf, for low-level debugging
make speedtest       # host-compile cst-speed.c and diff its output against the reference traces (see SPEED)
make speedtest-accept  # regenerate those reference traces from the current cst-speed.c
make pressuretest    # same, for the cst-speed.c counterpart cst-pressure.c (the AIRBRAKE model)
make pressuretest-accept  # regenerate the AIRBRAKE reference traces from the current cst-pressure.c
make eepromtest      # same, for the EEPROM layout migrations in cst-eeprom.c (applyEepromMigrations)
make eepromtest-accept  # regenerate the migration reference images from the current cst-eeprom.c
make clean           # remove build artifacts
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

The ATmega1284P (128 KB flash, 16 KB SRAM) has no pin-compatible successor with more memory, so this is a
one-way door. `make size` currently shows ample headroom — roughly 42% flash, 20% static RAM (leaving
~13 KB for the stack, against a deepest frame of a few hundred bytes) — and the fork has held near there
across the whole feature set. Check it before adding a large non-`PROGMEM` table, a wide LCD/canvas
buffer, or another large `switch(screenState)` branch. The `int64` scale-speed math is a latency concern
(hence out of the ISR — see SPEED), not a footprint one.

The automated firmware tests are `make speedtest`, `make pressuretest` and `make eepromtest` - host-compiled
reference-trace harnesses for the scale-speed model (`src/cst-speed-test/`, see the SPEED section), the
AIRBRAKE air-brake model (`src/cst-pressure-test/`, see the AIRBRAKE section) and the EEPROM layout
migrations (`src/cst-eeprom-test/`, see "EEPROM layout" below). Nothing else in the firmware has a test or
a linter. `src/eep-test/*.py` are standalone Python scripts (`mrbus.py`, `dumppkts.py`,
`test.py`) for sniffing/decoding MRBus/MRBee packets off the radio for manual debugging — not an
automated test harness.

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
out specific subsystems (LCD driver, battery monitoring, EEPROM config decode + layout migrations,
brake-pipe pressure simulation, tonnage/load sound, fast-clock time sync) but the orchestration logic all
lives in `mrbw-cst.c`. `readConfig()` decodes the current EEPROM layout into RAM globals and calls
`applyEepromMigrations()` (`cst-eeprom.c`) once per boot to rewrite an older layout forward;
`resetConfig()` (factory reset) delegates the per-profile SPEED/AIRBRAKE/STACK model defaults to
`eepromResetProfileModel()` in the same file.

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
*continuous-hold-while-condition-is-true* idiom used by `BRAKE_CONTROL`/`BRAKE_REL_CONTROL` in
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

**EEPROM layout migrations (`cst-eeprom.c`)**: one bespoke per-slot byte-remapping transform per
`EEPROM_LAYOUT_VERSION` bump (currently 5), factored out of `readConfig()` into
`applyEepromMigrations(uint8_t oldLayoutVersion)` — the highest-risk, least-verifiable firmware code (a
wrong offset silently corrupts every stored profile on upgrade). It touches only the EEPROM (no globals,
no LCD, no radio) via the byte-at-a-time `eeprom_*` API and is self-limiting: `readConfig()` calls it once
with the pre-stamp `EE_LAYOUT_VERSION` byte, and it stamps the current version so any later call the same
boot is a no-op. `make eepromtest` (`src/cst-eeprom-test/`) is a golden-master harness that drives it over
synthetic layout-N images and diffs the 4096-byte result — the only coverage of "an old-layout chip boots
newer firmware", since `slot_codec.py` does not model migrations. The migration *comments* (in
`cst-eeprom.c`) and the per-version narrative in the SPEED and AIRBRAKE sections are the authoritative
description of each transform.

`cst-eeprom.c` also holds **`eepromResetProfileModel(uint16_t configBase)`** — the factory-default writer
for the per-profile SPEED/AIRBRAKE/STACK "model" bytes (`0x28-0x62` minus the function slots), called by
`resetConfig()` for the working config, which then `copyConfig()`s it to all 20 profiles. It is the part
of a profile that grows as decoder families and sim parameters are added, so it is centralized here (one
list) and `make eepromtest` asserts it covers every model offset (`sc_reset_model` + `inv_reset_*`). The
non-model per-profile bytes (loco address, force-func, brake pulse, optionBits, notch table) and the
function-assignment bytes stay in `resetConfig()` / `cst-functions.c`.

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

## Light-function trailing lag

Some ESU decoders take measurable time to process the in-decoder logic for a newly asserted light
function while dropping a de-asserted one instantly. When the light knob moves to a new position, the
outgoing packet clears the old function bit(s) and sets the new one(s) in the same packet — correct at
the wire level, but the decoder asserting side lags its de-asserting side, producing a momentary dark
flicker on the physical light output during the crossfade (and, symmetrically, when the knob returns to
`LIGHT_OFF`). This is independent of the anti-glitch debounce described above, which already settles
`frontLight`/`rearLight` to a stable value before this mechanism ever sees a transition.

Each light channel (front, rear) holds a de-asserting function bit on for `LIGHT_LAG_HOLD_DECISECS`
(`mrbw-cst.c`) past a genuine `frontLight`/`rearLight` position change: on such a change, whichever
function bits were asserted for that channel last pass and are not part of the new position — the same
computation whether a bit is being replaced by a different one or the knob went fully to `LIGHT_OFF` —
are held asserted in `functionMask` for the lag window, on top of whatever the new position asserts on
its own. `frontLightLagTimeout_decisecs`/`rearLightLagTimeout_decisecs` (`volatile uint8_t`, decremented
in `TIMER0_COMPA_vect`'s 100ms block, the same idiom as `backlightTimeout_decisecs` — see "Menu
backlight hold") gate the hold; the held mask and the previous-position tracking are ordinary `main()`
locals, persistent across passes the same way `functionMask` itself is. The lag only adds bits, never
suppresses one, so `FORCE FUNC` and any other independent source of the same DCC bit are unaffected and
still take final precedence.

Applies uniformly to all eight light-knob-driven functions (`FRONT_HEADLIGHT_FN`/`FRONT_DITCH_FN`/
`FRONT_DIM1_FN`/`FRONT_DIM2_FN` and the `REAR_*` equivalents) — front and rear run independent timers,
since they are independent physical controls.

`LIGHT_LAG_HOLD_DECISECS` is a compile-time constant, not an on-device or EEPROM-configurable value — no
`EEPROM_LAYOUT_VERSION` bump, migration, or PC-tooling change is involved. Confirmed working on real
hardware against an ESU decoder at the current default of 1 decisecond (100ms); 2 and 3 deciseconds
(200ms/300ms) were also confirmed to eliminate the flicker during bench testing before the value was
lowered to its current minimum, so there is comfortable margin above 100ms if a different decoder needs
more.

## Two-stage horn ("Horn2")

A second, independently-configurable DCC function (`HORN2_FN`, menu name "HORN2") tied to its own calibrated
lever position (`hornThreshold2`), with a `HORNTYPE` option (`OPTION_SCREEN`) selecting how it relates to the
primary horn: Additive, default (Horn2 fires on top of Horn1 once the lever passes the Horn2 threshold) or
Exclusive (Horn2 replaces Horn1). On-screen the option reads `1 ←→ 1+2` (Additive) or `1 ←→ 2` (Exclusive).
Both thresholds use the standard hysteresis dead-band independently. Exclusive mode is a stateless one-line
override applied after both independent checks (`controls &= ~HORN_CONTROL` whenever `HORN2_CONTROL` is set
and `HORNTYPE` is Exclusive) — correct as long as the two calibration points are separated by more than the
hysteresis margin. `HORNTYPE` is `optionBits` bit 6 (per-profile); it round-trips through
`cst_cfgtransfer.py` / `cst_cfgnetwork.py` as `options.horn_type` (`"ADDITIVE"`/`"EXCLUSIVE"`).

**Calibration is optional.** `hornThreshold2 == 0xFF` — a throttle that never calibrated Horn2, or one
upgraded from stock firmware — simply means Horn2 is disabled. It is deliberately left out of the
`THRESHOLD CAL` auto-skip gate (unlike `hornThreshold`/`brakeThreshold*`), so an upgraded throttle is not
forced back through calibration for it. The HORN2 threshold-calibration subscreen shows a non-blocking
`<H1` cue whenever the captured `hornThreshold2` sits at or below `hornThreshold` (the Horn1 point) — a
misconfiguration that would let Horn2 fire before Horn1.

**Diagnostic display**: the function-status row of `DIAG_SCREEN` shows `HORN_CHAR` (the stock horn/trumpet icon,
unchanged) at column 5 when Horn1 is active and the same `HORN_CHAR` at column 6, immediately to its right,
when Horn2 is active — the two columns are independent, so both can be lit at once.

**FORCE FUNC audition**: the stock feature (upstream `26199c7`) where holding the horn lever in the FORCE
FUNC menu momentarily fires the on-screen function number — for auditioning it on the loco before forcing
it on/off — now covers both horn stages and is confined to the F## editing subscreen. Crossing either
`hornThreshold` or `hornThreshold2` asserts `1 << functionNumber` in place of `HORN_FN` / `HORN2_FN`, so
the audition works across the whole lever travel in Additive or Exclusive HORNTYPE (Exclusive clears
`HORN_CONTROL` at full lever, so without the Horn2 branch the audition would otherwise drop out there). On
the FORCE FUNC landing page (`subscreenState == 0`, not yet SELECT-ed in) the horn behaves normally — the
`&& subscreenState` guard keeps a stale `functionNumber` from firing, which the original `screenState`-only
check did not.

## Brake logic

`brakeState` (`BrakeStates` enum: `BRAKE_LOW_BEGIN` ... `BRAKE_FULL_WAIT`, `mrbw-cst.c`) drives two
outputs, `BRAKE_CONTROL` and `BRAKE_REL_CONTROL`, which map to the `BRAKE_FN`/`BRAKE_REL_FN` logical
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
threshold-calibration menu screens, not compile-time constants. `resetConfig()` deliberately does not
touch them (they are per-physical-device). The lever percentage is
`100 * (pos - low) / (high - low)`; that divisor is guarded — `high <= low` (an uncalibrated chip has
both `0xFF`, or a mis-calibration can put the two capture points within `2 * BRAKE_DEAD_ZONE`) falls
back to an all-or-nothing `brakePcnt`, and the `THRESHOLD CAL` `SELECT`-save refuses a `HIGH <= LOW`
calibration (`BRK CAL` / `HI<=LO` screen).
`OPTIONBITS_ESTOP_ON_BRAKE` ("BRK ESTP") is a mode-independent check that runs *before* the brake-mode
dispatch, comparing raw `brakePosition` against `brakeLowThreshold`/`brakeHighThreshold` directly — pushing
the lever to max triggers the throttle built-in emergency stop (if enabled) regardless of `BRK TYPE`,
clearing only once the lever returns fully to the bottom.

`BRK TYPE`, `BRK RATE`, `VAR BRK`, and `BRK ESTP` only take real effect once saved — every check above
reads a committed snapshot of `optionBits`/`brakePulseWidth`, not whatever `OPTION_SCREEN` is tentatively
editing on-screen; see "Committed, not live, config state" under "On-device config-screen pattern" below.

### STACK combo brake mode

A fourth mode where lever percentage drives combinations of **three DCC functions** — "Brake1" (reuses the
existing `BRAKE_FN`/`BRAKE_CONTROL` plumbing directly), "Brake2", "Brake3" (`BK2_FN`/`BK3_FN`) — mapped to
e.g. ESU LokSound/LokPilot V5 brake functions that stack, instead of pulsing/stepping a single function. Combos
track the lever symmetrically in both directions, and each combo persists (holds continuously) rather than
pulsing, matching the standard/pulse mode idiom rather than the stepped mode one.

Two step-count variants, selected by a `STEPS` toggle (3-STEP default, 5-STEP option; existing
combo config for whichever variant is not active stays stored and reactivates if switched back): the
lever divides into `stackBandCount()` equal-width bands (4 for 3-STEP, 6 for 5-STEP, both counts
including band 0). Band 0 (full-left) and the top band (full-right) are pinned to the lever extremes;
band 0 is permanently fixed to "no combo" and asserts `BRAKE_REL_FN` continuously (like the standard/
pulse mode release, not the stepped mode one-tick pulse) — it is not stored or editable. An interior
band combo of `0x00` ("none active") is distinct from band 0: it asserts neither a brake-on combo nor
`BRAKE_REL_FN`.
Band cut-points live in the explicit ordered arrays `stackBandThresholds3Step`/`5Step` rather than a
formula, since boundaries could change to become uneven in the future.

Combo evaluation is **stateless per loop** (`evaluateStackBrake(brakePcnt)`, called from the main
brake-mode dispatch, reading the committed band count/threshold table/combo array selection rather than
whatever `OPTION_SCREEN` is live-editing — see "Committed, not live, config state" below) — not a graft
onto the `BrakeStates` state machine, which is deliberately asymmetric
(advance-only, TCS-style) and the wrong shape here. It re-derives the correct band fresh from `brakePcnt`
on every call: escalate immediately on crossing a threshold going up, de-escalate only once
`BRAKE_HYSTERESIS` below that same threshold coming back down — the same dead-band idiom basic on/off mode
uses at its one boundary, generalized to as many boundaries as the active variant has. All 3 combo bits
live directly in the `controls` byte (`uint16_t`) — `BRAKE_CONTROL` (reused) for Brake1, `BK2_CONTROL`/
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
the editable bands only. `committedStackBandCombos3Step[]`/`5Step[]` mirror whichever combo set was last
saved and are what `evaluateStackBrake()` actually reads — see "Committed, not live, config state" below.

The AIRBRAKE screen (see "AIRBRAKE" below) is a read-only viewport onto the air-brake sound model and
suppresses nothing — working the lever there drives `BRAKE_CONTROL`/`BK2_CONTROL`/`BK3_CONTROL` and the
real lever e-stop exactly as from the main screen.

**DCC packet economy note**: NMRA DCC groups function numbers into fixed packet groups (F0-F4, F5-F8,
F9-F12, F13-F20, F21-F28); assigning Brake1/2/3 within one group (F9-F12 fits all three) lets a downstream
command station fold a simultaneous multi-brake transition into one DCC packet instead of up to three. This
is a Configure Function choice the user makes — the firmware always sends the full function bitmask as one
MRBus/MRBee packet regardless.

## SPEED — scale-speed simulation

Replaces the fast clock on the main screen with a locally-computed **scale miles-per-hour (or km/h)**
readout that models ESU LokSound/LokPilot (both V4 and V5, selected via the `TYPE` field) momentum and
three-brake deceleration behavior, so the displayed speed stays in visual sync with what the decoder is
actually doing rather than just the instantaneous commanded speed step. Local computation was chosen over
an earlier wireless-received-speed design that was evaluated and not adopted.

Commanded speed is `notchSpeedStep[activeThrottleSetting-1]` — the same 0-126 DCC speed step already
transmitted, no new input needed. Scale speed is `speedStep/126 × speedMaxMph`, with `speedMaxMph` a
per-profile configurable value (`MAXSPEED`), converted to km/h at display time if that unit is selected.
`src/cst-speed.c`/`.h` (mirroring the shape of `cst-pressure.c`: EEPROM-backed per-profile config, 10Hz
update, own display printer) holds `simSpeedStepQ8`, an 8.8 fixed-point simulated speed step — chosen
over a plain integer to avoid per-tick truncation stalling at slow rates, the same class of problem the
AIRBRAKE brake-pipe recharge works around with its own fixed tail-crawl floor
(`AIRBRAKE_RECHARGE_TAIL_MPSI`, see "AIRBRAKE" below). `updateSpeed10Hz()` runs once per 100ms tick, but
from the **main loop**, not the timer ISR: `TIMER0_COMPA_vect` only sets a `speed10HzTick` flag and the
main loop consumes it. The standing-start ramp does `int64` multiply/divide that must not run inside an ISR
— it would stall the radio-UART and quadrature-encoder interrupts for its duration (a latency concern, not
a CPU-load one — the math is a ~0.25% duty cycle). `updateBrake10Hz()` runs the same way, from the main
loop via its own `brake10HzTick` flag — only `updateTime10Hz()` stays in the ISR: cheap, and it needs
precise timing. Brake-active flags and the other inputs are passed to `updateSpeed10Hz()` explicitly
rather than read via `extern`, so the module has no dependency on the internal bit layout of
`mrbw-cst.c`.

**ESU LokSound/LokPilot formulas modeled** (from ESU community documentation + the times LokProgrammer computes,
times — covers both V4 and V5, see `TYPE` below):
- Accel/decel: time to cross the full speed range = `(CV + CVadj) × multiplier` seconds (CV3 + CV23
  for accel, CV4 + CV24 for decel). The multiplier is decoder-family-dependent — see `TYPE` below.
  ESU does not clamp `CV3+CV23` / `CV4+CV24` at 255; the model allows the sum up to 382.
- Brake override: `stopSeconds = (255-CVbrakeSum)/255 × ((CV4 + CV24) × multiplier)`.
- Start delay (decoder CV167): `delaySeconds = CV167 × 0.25`.

**Brake semantics**: Brake1/2/3 (`CV179`/`180`/`181`, mirroring the same three DCC functions STACK brake
mode drives) sum when stacked, capped at 255 (near-instant stop), rather than "fastest wins." Any brake
asserted overrides throttle demand entirely — the simulation always heads toward a full stop while a brake
is held. Start Delay only arms once the loco is both fully stopped *and* free of any asserted brake.
**Reverser neutral is a hard override**: `commandedSpeedStep` is forced to 0 whenever the reverser is
centered, unlike the real outgoing DCC packet (which relies on the decoder separately-sent `NEUTRAL_FN`)
— a deliberate simulation-only difference, since the loco is physically stationary in neutral regardless of
notch position.

### On-device editor

`SPEED_CONFIG_SCREEN` (landing page `SPEED CFG`). Five items are decoder-type-agnostic and always
lead, in this order: `TYPE`, `MAXSPEED`, `UNIT`, `ACCEL`, `DECEL` — except that `ACCELADJ`/`DECELADJ`
(`SPEED_ITEM_ACCEL_ADJ`/`_DECEL_ADJ`, V5 only) are spliced into the menu immediately after the
`ACCEL`/`DECEL` they adjust, so on V5 the run is `... ACCEL, ACCELADJ, DECEL, DECELADJ, ...`. After
them come the rest of the model parameters the current `TYPE` uses, in the order fixed by the
per-family descriptor in `cst-speed.c` (`speedTypeDesc[]`): `speedItemAt(subscreenState, advFunc)`
(the shape of `optionItemAt()`) resolves a menu position to a `SPEED_ITEM_*` — it walks the agnostic
spine with that adjust splice, then the descriptor model list (skipping `ACCELADJ`/`DECELADJ`, already
emitted, and the ADV-FUNC-gated tunables while `ADV FUNC` is off), returning the `SPEED_ITEM_COUNT`
sentinel once past the last visible item. `V5DCC` and `V5MULT` expose all 16 model parameters —
`ACCELADJ`/`DECELADJ` (shown next to `ACCEL`/`DECEL` as above), then `BRK1`/`BRK2`/`BRK3`/`DELAY`,
`HOLDFN`/`STOPFN`, `OPLOAD`/`OPLOADFN`/`PRLOAD`/`PRLOADFN`, and the four correction tunables; `V4`
exposes 8 (`BRK1`, `DELAY`, `HOLDFN`, `STOPFN`, and the four tunables — it drops `ACCELADJ`/`DECELADJ`,
`BRK2`/`BRK3` and the load CVs, see "How the simulation math works" below). `BRK1` is present for every
family but sits in the model list (contiguous with `BRK2`/`BRK3`), not the agnostic block — its EEPROM
offset `0x2B` and save/read path are unchanged, only its menu slot. The four correction tunables
(`ACCPCT`/`ACCTGT`/`DECPCT`/`DECTHR`) stay last in every family so the `ADV FUNC` (`SYSTEM` screen) gate
that hides them is a tail skip; they are always read via `readByteOrDefault()` with real defaults
regardless of visibility, so hiding them never risks an unset field. `UNIT` is a two-way toggle;
`TYPE` cycles the three families and calls `speedResetModel()` on each change;
`HOLDFN`/`STOPFN`/`OPLOADFN`/`PRLOADFN` show `OFF` or `F##`.

**Editor ranges and the `0xFF` sentinel**: most plain-numeric SPEED bytes self-heal from `0xFF` via
`readByteOrDefault()`, so a stored 255 would silently revert on the next load — those items
(`BRK1`/`BRK2`/`BRK3`/`DELAY`/`OPLOAD`/`PRLOAD`/`ACCPCT`/`ACCTGT`/`DECPCT`/`DECTHR`) cap at **254** in
the editor, matching the AIRBRAKE screen. `ACCEL`/`DECEL` are genuine **0-255** (a decoder CV3/CV4 can
itself be 255): `readConfig()` reads `0x28`/`0x2E` raw, and the `EEPROM_LAYOUT_VERSION` → 4
migration seeds any never-written byte (see "EEPROM layout" below). `HOLDFN` is likewise read raw so
setting it to `OFF` persists (its `readByteOrDefault` default is F09, which would otherwise revert a
user-set OFF). UP/DOWN on all four watched-function items (`HOLDFN`/`STOPFN`/`OPLOADFN`/`PRLOADFN`)
wrap circularly at both ends (`OFF <-> F00 ... F28 <-> OFF`) rather than clamping, matching CONFIG
FUNC's `F00..F28` cycle convention for a plain function slot with no momentary/latching axis.
`MAXSPEED` is **1-254** (0 is a divisor in the standing-start ramp — `updateSpeed10Hz()`
also guards it). `ACCELADJ`/`DECELADJ` are a signed `-127..+127` (the full ESU CV23/CV24 magnitude
range) shown as a fixed 4-char field `   0` / `+063` / `-127`; the stored byte uses the decoder
sign-magnitude encoding (bit 7 = subtract), so `-127` is byte `0xFF` and `0x61`/`0x62` are read raw
too. Their menu label is `ACCEL ` / `DECEL ` followed by a `±` CGRAM glyph — the NHD-0208AZ font ROM
has no `±`, so `LCD_SPEED_ADJ` (a `LcdMode`) loads the glyph into the `AUX_CHAR` slot (main-screen
only) while an adjust item is shown; the menu-exit `setupLCD(LCD_DEFAULT)` restores `AUX_CHAR`.

**`speedResetModel(oldType, newType)`** (from the `TYPE` edit) and **`speedApplyTypeInert()`** (from
`readConfig()`) keep the parameters a family does not use out of the model. `speedResetModel()` sets a
parameter the new family drops to its inert value (`ACCELADJ`/`DECELADJ`/`BRK2`/`BRK3` to 0, the load
CVs to 128, their watch functions to `OFF`) and one it gains back to its default;
`speedApplyTypeInert()` does the same one-shot on load for whatever is stored. The inert values are
exactly where `updateSpeed10Hz()` already ignores a parameter, so the model runs one code path for
every family — there is no `switch(speedType())` in the simulation math. `V5DCC` <-> `V5MULT` is a
no-op (identical parameter sets). The `SELECT`-save still writes all 21 `SPEED_ITEM_*` bytes
regardless of family, so a `V4` profile persists inert zeros in the dropped slots.

`printSpeed()` converts the configured `MAXSPEED` into km/h once before computing the displayed value
(rather than converting an already-rounded mph figure) to avoid compounding rounding error. The
readout is one fixed 6-character field at every `MAXSPEED` — a right-justified 3-digit number
(space-padded below 100), then `MP` / `KM`, then the narrow-H unit glyph (`SPEED_H_CHAR`, see "OPS
MODE screen") — so the layout never shifts as the max or the live value crosses 100.

**Main-screen display toggle, "DISPLAY" in PREFS**: `CONFIGBITS_MAIN_SCREEN_SPEED` toggles the main screen
between `printTime()` and `printSpeed()`. The clock is the default — the bit is clear on a fresh chip
(not in `CONFIGBITS_DEFAULT`) and clear in the stored config byte of any throttle upgrading from stock
firmware, so both land on the clock; showing speed is an explicit opt-in. While the bit is clear (clock),
`SPEED CFG` is silently skipped in the top-level menu cycle.

### Watched-function e-stop and Drive Hold

Two independent conditions snap the display straight to zero (held there for as long as either stays true)
instead of ramping down at the normal/brake rate: `STOPFN` (a per-profile watched DCC function number,
0-28 or OFF — whenever that function number is present *anywhere* in the outgoing `functionMask`,
regardless of which physical control put it there) and the throttle built-in e-stop
(`THROTTLE_STATUS_EMERGENCY`). Both go through `snapToStop()`, which also drops any in-progress
standing-start ramp and re-arms the Start Delay, so releasing e-stop/`STOPFN` resumes from a genuine
standing start — not mid-ramp. `HOLDFN` (default F09) freezes the simulation exactly as-is while asserted —
no state is touched, since `functionMask`/`commandedSpeedStep` are recomputed fresh every pass, so freezing
is sufficient by construction for "resume based on current state once released." While `HOLDFN` is
asserted, the loco-address line of the main screen is also replaced with a literal `"HOLD"` on-screen cue
— the same priority slot the throttle existing reverser-mismatch indicator already uses, checked in order:
alerter-timeout backlight blink → reverser mismatch → `"HOLD"` → normal loco address/speed display.
When the `AUX` function is assigned to the same DCC function number `HOLDFN` watches, the `AUX`
indicator glyph is suppressed while `AUX` is active (the `HOLD` text beside it is already the cue) —
see `auxIndicatorChar()` under "OPS MODE screen".
`STOPFN` has no on-screen text of its own — its effect is only visible through the speed readout itself
snapping to 0mph.

`THROTTLE_STATUS_EMERGENCY` is a live combinational OR of three sources, recomputed every main-loop pass:
brake lever at max (gated by `OPTIONBITS_ESTOP_ON_BRAKE`), a control configured to `FN_EMRG`, and alerter
timeout specifically when `ALERTER_FN = FN_EMRG` — an ordinary alerter timeout always forces the throttle
to zero and applies the brake as a fail-safe regardless of how `ALERTER_FN` is configured, but only feeds
`THROTTLE_STATUS_EMERGENCY` (and so the display snap-to-zero) in the `FN_EMRG` case. Either way the
alerter timeout asserts the `BRAKE_FN` bit directly in `functionMask` (the alerter-timeout block in
`mrbw-cst.c`), so the simulation ordinary `functionMask`-based Brake1 detection (see "Brake-function
detection" below) picks it up through the same path as every other brake source — no separate
mechanism is needed.

### How the simulation math works

**Decoder family (`TYPE`)**: three values, selecting the momentum-CV time convention `ACCEL`/`DECEL`/
`BRK1-3` use and which model parameters apply. `V5DCC` — 0.896s per CV unit, ESU LokSound 5 DCC only,
the NMRA S9.2.2 multiplier. `V5MULT` — 0.25s per CV unit, ESU LokSound/LokPilot V5 MultiProtocol; the
same 16-parameter model as `V5DCC`, only the multiplier differs. `V4` — 0.25s per CV unit,
LokPilot/LokSound V4; the identical model math and multiplier as `V5MULT`, but a V4 decoder has no
CV23/CV24, no CV180/CV181 and no CV103/CV104, so `ACCELADJ`/`DECELADJ`, `BRK2`/`BRK3` and the load CVs
are dropped from its menu and forced inert (see the editor section above), making `V4` behave as
`V5MULT` with the adjusts, stacked braking and load scaling off. `SPEED_TYPE_V5MULT` keeps the byte
value (1) the old combined `V4V5MULT` used, so a stored `TYPE` of 1 upgrades to `V5MULT` with no
behavior change; `V4` is the new value 2, and there is no `EEPROM_LAYOUT_VERSION` bump for the split
(no byte moves, value 1 keeps its meaning).

**Acceleration/deceleration adjust (`ACCELADJ`/`DECELADJ`, V5 only)**: mirrors decoder CV23/CV24 — a
signed factor `-127..+127` added to `ACCEL`/`DECEL` before the family multiplier, so the effective CV
is `CV3 + CV23` / `CV4 + CV24`. ESU does not clamp the sum at 255, so `speedEffAccelCV()` /
`speedEffDecelCV()` allow it up to `255 + 127 = 382` (`uint16_t`), floored at 0. These two helpers
replace every raw `ACCEL`/`DECEL` read in `updateSpeed10Hz()`; on a V4 profile the adjust byte is 0
(forced inert) so they are plain pass-throughs — no `speedType()` check inside the model. `V5MULT`
uses the same `-127..+127` semantics as `V5DCC` (only the base multiplier differs); LokProgrammer
confirms CV24 scales with the family multiplier just like CV4.

**Load simulation (`OPLOAD`/`PRLOAD`/`OPLOADFN`/`PRLOADFN`)**: mirrors decoder CV103 (Optional Load)/CV104
(Primary Load) — each a 0-255 value (128 = neutral) that scales the base `ACCEL`/`DECEL` CV while its
watched DCC function is active, `time = CV × loadValue/128`. Primary Load wins if both are active
simultaneously, per the ESU manual. `OPLOADFN`/`PRLOADFN` can also be asserted directly by a button
configured to the LOAD function — see "LOAD button function" below.

**Brake-function detection**: the simulation reads whether Brake1/2/3 are active from the actual outgoing
`functionMask`, not from the lever own state-machine bits — so any control mapped to the same DCC
function number is detected regardless of source. Step brake mode is deliberately excluded (forced
inactive) — checked against the same committed `BRK TYPE` the main dispatch uses, not a live
`OPTION_SCREEN` edit (see "Committed, not live, config state" under "On-device config-screen pattern"
below) — its pulses are a one-directional decoder-side ratchet the sum-based brake model cannot
represent; Step remains the one brake mode where the simulated behavior can diverge from a real
Step-braking locomotive.

**Deceleration lag correction (`DECTHR`/`DECPCT`)**: in the normal momentum range a real ESU V5 decoder
decelerates *faster* than the plain `DECEL` (or summed brake) linear model predicts — a BEMF-regulator
control-loop artifact. `DECPCT` (0-255 = 0-100% strength) *subtracts* ticks from the coast/brake to
match: `lag(speed) = slope × max(0, speedStep − DECTHR)`, slope proportional to the reference time
`ticksToCross(effectiveDECEL)`. Captured once at the transition into decelerating, held for the run,
applied via one unconditional path — coast or brake, steady-state or interrupting an active climb.
Higher `DECPCT` → display reaches 0 sooner; higher `DECTHR` (~0.4 mph/unit) → the correction covers
less of the run, display reaches 0 later.

Calibrated (`DECTHR ≈ 11`, `DECPCT ≈ 22`) from hardware against `DECEL` 192/216/230. The correction
is required for accurate sync at effective `DECEL` (`CV4 + CV24`) up to ~230; above that a real
decoder deceleration goes genuinely linear at the momentum-register ceiling (the BEMF-regulator
artifact washes out and the programmed ramp dominates), so `DECPCT` is faded out automatically —
see "Momentum-ceiling linearization" below. `ticksToCross()` itself is strictly linear in the CV at
every value — this is a decoder characteristic, not a firmware bug.

**Standing-start acceleration (`ACCPCT`/`ACCTGT`)**: real locomotives reach any speed below 15mph faster
than the plain `ACCEL` model predicts, by a roughly constant amount of time (not proportional to distance).
`ACCPCT` (0-255 = 0-100% of the full-range crossing time of `ACCEL` itself) is that lead-time budget, spent
as a smooth cubic-Hermite ramp from a genuine stop up to the 15mph-equivalent speed, calibrated from a
30-point standing-start hardware sweep. `ACCTGT` (0.1s/unit) is a target time-to-1mph, achieved by
generalizing the zero-rate starting boundary condition of the ramp to a configurable nonzero initial slope
chosen so the ramp hits the target tick exactly — both ramp endpoints are algebraically unaffected by this,
so the calibrated total time of `ACCPCT` to 15mph never changes regardless of `ACCTGT`. `ACCTGT` can only
*shorten* the onset from its own natural baseline, never push it later. Very aggressive `ACCTGT` values can make the
ramp briefly non-monotonic later in its climb (confirmed tiny, below display resolution, at the one extreme
tested).

**Momentum-ceiling linearization**: the two BEMF-regulator corrections above — the standing-start cubic
ramp and the `DECPCT` deceleration lag — were fitted only to an effective momentum CV of ~230. At the
8-bit register ceiling a real ESU V5 decoder runs its programmed ramp linearly, so both are faded to
their linear limit across a shared window on the *raw* effective CV (`speedEffAccelCV()` /
`speedEffDecelCV()`, `SPEED_CEIL_FADE_LO` 230 → `SPEED_CEIL_FADE_HI` 255, `cst-speed.c`
`ceilFadeNum()`). Hardware-confirmed on both sides: with the fade active the display tracks the loco
at and above effective `DECEL` / `ACCEL` 255 (`CV4 + CV24` / `CV3 + CV23` past 255 via the adjusts
included), where the pre-fade model drifted — the coast display reached 0 seconds early and the
standing start rushed to ~14mph then dwelt.
- **DECPCT** scales by the fade weight before the `steadyStopExtraMs` capture — full strength at
  effective `DECEL` ≤ 230, zero at ≥ 255 (pure `ticksToCross()` coast/brake).
- **Standing-start ramp**: the blend `f·cubic + (1−f)·(v·τ)` folds exactly into the precomputed
  `rampP`/`rampQ`/`rampR0` (so `cubicRampPosition()` and the per-tick path are untouched). `f = 1`
  (effective `ACCEL` ≤ 230) is byte-identical to the calibrated ramp; `f = 0` (≥ 255) sets
  `rampP = rampQ = 0`, `rampR0 = v`, making the onset an exact linear `v·τ` crawl that hands off
  seamlessly to the plain-rate climb at `rampT` — no S-curve, no dwell near 15mph, no head-start.

`ACCELADJ`/`DECELADJ` feed the effective CV, so a large CV23/CV24 crosses the ceiling on its own. The
230→255 transition sweeps smoothly with no visible discontinuity at either end; the exact interior
(231-254) blend is a linear interpolation between the calibrated (≤230) strength and the linear (≥255)
endpoint, not independently point-calibrated. `SPEED_CEIL_FADE_*` are compile-time `#define`s, one
shared window, split into `_ACCEL_`/`_DECEL_` pairs if a later hardware pass shows the accel side wants
a different band.

**Requires a linearized decoder speed table**: the simulation assumes real locomotive speed scales linearly
with the commanded DCC speed step — this only holds if the decoder own speed-table CVs are themselves
calibrated linear across the full 0-126 range. A non-linear/uncalibrated decoder speed table will make the
display diverge from the real locomotive regardless of `SPEED` tuning, since that is a decoder-side
characteristic entirely outside this codebase.

### Reference: `SPEED` field definitions and tested values

`ACCEL`/`DECEL`/`BRK1-3`/`DELAY` are named after their CV role, but the CV convention itself is inverted
from what the name suggests — a *bigger* number means *slower*/*weaker* (a time constant, not a rate); that
is an NMRA convention, not a naming choice made here. `ACCEL`/`DECEL` are genuine 0-255; the other
plain-numeric items cap at 254 in the editor (a stored 255 would collide with the `0xFF` self-heal
sentinel).

**Momentum CVs (direct decoder mirrors)**

| Item | Default | Tested | What it does | Increase | Decrease |
|---|---|---|---|---|---|
| `ACCEL` (CV3) | 60 | 60 | Time to cross the full speed range while speeding up. 0-255 | Slower acceleration | Faster acceleration |
| `DECEL` (CV4) | 230 | — | Same as `ACCEL` but for coasting down (no brake held). 0-255. The `DECPCT`/`DECTHR` lag correction is validated to effective `DECEL` (`CV4 + CV24`) ~230 and fades itself out to a plain linear coast by effective `DECEL` 255 — see "Momentum-ceiling linearization". | Slower coast-down | Faster coast-down |
| `ACCELADJ` (CV23) | 0 | — | Signed `-127..+127` added to `ACCEL` (V5 only). Effective CV3 = `ACCEL + ACCELADJ`, unclamped past 255 | Slower acceleration | Faster acceleration |
| `DECELADJ` (CV24) | 0 | — | Signed `-127..+127` added to `DECEL` (V5 only). Effective CV4 = `DECEL + DECELADJ`, unclamped past 255 | Slower coast-down | Faster coast-down |
| `BRK1` (CV179) | 130 | — | How strongly Brake1 shortens the stop when active — sums with `BRK2`/`BRK3` (capped at 255) | Faster/harder stop | Weaker braking |
| `BRK2` (CV180) | 70 | — | Same as `BRK1`, second stackable brake | Faster/harder stop | Weaker braking |
| `BRK3` (CV181) | 100 | — | Same as `BRK1`, third stackable brake | Faster/harder stop | Weaker braking |
| `DELAY` (CV167) | 13 | — | Mirrors the decoder own programmed prime-mover spool-up time. 0.25s/unit | Longer pause before movement | Shorter pause (0 = none) |

**Display calibration**

| Item | Default | Tested | What it does |
|---|---|---|---|
| `MAXSPEED` | 50 | 50 | Real-world scale speed (mph) at speed step 126 — the calibration anchor. 1-254 (0 is a divisor in the ramp math) |
| `UNIT` | MPH | — | MPH or KMH display |

**Watched-function triggers**

| Item | Default | What it does |
|---|---|---|
| `HOLDFN` | F09 | DCC function watched for ESU Drive Hold — display freezes while asserted. `OFF` persists (read raw, not `readByteOrDefault`) |
| `STOPFN` | OFF | DCC function watched to snap the display to 0mph |

**Load simulation (CV103/CV104)**

| Item | Default | What it does |
|---|---|---|
| `OPLOAD` | 128 | Scales `ACCEL`/`DECEL` while its watched function is active. 128 = neutral |
| `OPLOADFN` | OFF | DCC function watched to switch `OPLOAD` on |
| `PRLOAD` | 128 | Same as `OPLOAD`; wins if both active at once |
| `PRLOADFN` | OFF | DCC function watched to switch `PRLOAD` on |

**Decoder family**

| Item | Default | What it does |
|---|---|---|
| `TYPE` | V5DCC | `V5DCC` (0.896s/unit), `V5MULT` (0.25s/unit, same 16-parameter model), or `V4` (0.25s/unit, drops `ACCELADJ`/`DECELADJ`, `BRK2`/`BRK3` and the load CVs) |

**Correction tunables** (hidden behind `ADV FUNC`)

| Item | Default | Tested | What it does |
|---|---|---|---|
| `ACCPCT` | 8 | 8 | Standing-start head start (0-255 = 0-100% of the `ACCEL` full-range time). Faded to a plain linear onset as effective `ACCEL` approaches 255 — see "Momentum-ceiling linearization" |
| `ACCTGT` | 5 | 5 | Target time (0.1s/unit) for the display to first show 1mph |
| `DECPCT` | 22 | 22 | Strength (0-255 = 0-100%) of the deceleration-lag correction. Faded automatically to 0 across effective `DECEL` 230→255 (see "Momentum-ceiling linearization") |
| `DECTHR` | 11 | 11 | Speed (raw step, ~0.4 mph/unit) below which the `DECPCT` correction does not apply |

Confirmed working values for the calibration locomotive: `ACCEL=60`, `MAXSPEED=50`, `DECTHR=11`,
`DECPCT=22`, `ACCPCT=8`, `ACCTGT=5` (the shipped defaults already match). The momentum-ceiling fade
was additionally validated on that locomotive at effective `ACCEL` / `DECEL` 255 and above (via the
adjusts). Every other field above is still the shipped compile-time default, not independently
re-validated against that locomotive. `ACCELADJ`/`DECELADJ` were bench-checked against the times
LokProgrammer computes (V5DCC, and V5MULT for CV24 scaling), confirmed on a locomotive with a
non-zero CV23/CV24 in the normal momentum range, and their effect at the ceiling is covered by that
fade test.

### EEPROM layout and the reference-trace test

The 5 type-agnostic `SPEED` items (`TYPE`/`MAXSPEED`/`UNIT`/`ACCEL`/`DECEL`) keep their original
scattered addresses (`ACCEL`/`DECEL` in the gaps left by `EE_BK2_FUNCTION`/`EE_BK3_FUNCTION`,
`MAXSPEED`/`UNIT`/`TYPE` after `EE_STACK_BAND_COMBOS`); `BRK1` stays at `0x2B` but is a model-list item.
The decoder-type-specific model parameters live in one contiguous block, `EE_SPEED_MODEL_PAYLOAD`
(`0x54-0x63`, 15 used with `0x63` reserved — `ACCELADJ`/`DECELADJ` at `0x61`/`0x62`). Migrations
(`applyEepromMigrations()` in `cst-eeprom.c`, called once from `readConfig()`; covered by
`make eepromtest`, see "EEPROM layout migrations" in Architecture):

- **2 → 3** **relocates** the 13 scattered model parameters (`0x54-0x60`) into this block rather than
  resetting them: a layout-2 throttle keeps every tuned value (each byte copied from its old offset
  across all 20 profiles plus the working config), a stock or pre-guard chip gets model defaults, a
  blank chip self-heals. No backup or re-import needed.
- **3 → 4** — five SPEED bytes now leave `readByteOrDefault()` and are read raw so a stored `0xFF` is
  a real value — `ACCEL`/`DECEL` (`0x28`/`0x2E`, `0xFF` = 255), `HOLDFN` (`0x57`, `0xFF` = OFF),
  `ACCELADJ`/`DECELADJ` (`0x61`/`0x62`, `0xFF` = −127). The migration seeds `0x28`/`0x2E`/`0x57` where
  currently `0xFF` (a real 0-254 value is preserved) and inits `0x61`/`0x62` to 0. Non-destructive —
  no config byte is overwritten, only version stamp + `0xFF → default`. Gated `(oldLayoutVersion < 4)
  || (0xFF == oldLayoutVersion)` so it runs on a pre-4 chip **and** a blank/wiped chip but not a
  layout-4 chip crossing to 5 (whose `0x61`/`0x62` already hold real values that must not be reset).
- **4 → 5** — `0x2C`/`0x2D` become `EE_MENU_BUTTON_FUNCTION` / `EE_SEL_BUTTON_FUNCTION` (the OPS MODE
  function buttons — see "OPS MODE screen"). The migration seeds both to `FN_OFF` across all 20
  profiles plus the working config. Function bytes are read raw, so this seed is the only thing
  between an upgraded throttle and a garbage `MENU BTN` / `SEL BTN` assignment. Gated
  `!= EEPROM_LAYOUT_VERSION` so it also runs on a blank/wiped chip; non-destructive, since `0x2C`/
  `0x2D` carry nothing meaningful on any pre-5 layout (on a layout-2 chip the 2 → 3 block already
  relocated the real `BRK2`/`BRK3` values out of them earlier in the same call).

`make speedtest` runs `src/cst-speed-test/` — a host-compiled (`cc`, not `avr-gcc`) harness that
`#include`s `cst-speed.c` whole, drives `updateSpeed10Hz()` through a fixed scenario set, and diffs the
per-tick `simSpeedStepQ8` and `printSpeed()` output against checked-in reference traces
(`reference/*.txt`). It is the regression net for any change to the model — a diff means the output
moved, either intended (`make speedtest-accept` re-blesses the traces) or a regression. It also asserts
two invariants as `PASS`/`FAIL` lines (the `V4` model equals the `V5MULT` model with all its dropped
parameters no-oped — `ACCELADJ`/`DECELADJ`/`BRK2`/`BRK3` and the load CVs; `speedApplyTypeInert()`
neutralises a stale slot) and exits non-zero on failure. `.githooks/pre-commit` runs it whenever a
commit touches `cst-speed.c`, `cst-speed.h`, or that directory. The only host-build shim is
`cst-speed-test/stubs/avr/pgmspace.h`, needed because `lcd.h` includes `<avr/pgmspace.h>`. Scenarios
keep momentum CVs in non-zero ranges where AVR 16-bit `int` and host 32-bit `int` provably agree. The
`accel_cv255`/`decel_cv255` and `accel_ceil_fade_mid`/`decel_ceil_fade_mid`/`accel_adj_over_ceiling`
scenarios exercise the momentum-ceiling linearization: at effective CV 255 both `accel_cv255` (a clean
linear standing-start crawl) and `decel_cv255` (a plain linear coast) confirm the corrections are
fully faded out; the `*_mid` pair holds the interior blend (`0 < f < 16`); `accel_adj_over_ceiling`
proves `ACCELADJ` alone can cross the ceiling.

## AIRBRAKE — air-brake simulation

`src/cst-pressure.c` models a locomotive air-brake system — brake pipe + main reservoir — and drives
DCC sound functions from it. It replaces the old transient "Brake Test" pressure gauge (the
`PumpState` machine and `LCD_PRESSURE` are gone; the analogue-dial CGRAM canvas itself was later
revived as the `DISPLAY = SINGLE` gauge view — see below).

`updateBrake10Hz(leverPcnt, independentBrakeAtRest, emergencyBrakeEnabled)` runs once per 10 Hz tick
**from the main loop** (ISR sets `brake10HzTick`, the loop consumes it — same pattern as the speed
sim, so the model takes its inputs as parameters rather than reaching into locals inside `main()`).
It ticks **every pass regardless of the enable bit**; the bit only gates whether the outputs reach
the DCC packet.

**Automatic vs independent brake.** The ProtoThrottle has one brake lever that plays two real-world
roles: the *automatic* (train) brake — brake-pipe reductions, whose release recharges the whole
trainline and the auxiliary reservoir on every car from the main reservoir (a large,
compressor-running draw) — and the *independent* (loco-only) brake used for everyday graduated
braking (the Brake1/2/3 combos in STACK mode), which barely touches the reservoir. AIRBRAKE models
the lever as the automatic brake — the one brake whose pipe-and-reservoir physics this file simulates
— and every brake-pipe recharge draws the reservoir at the same `MR LOAD` rate regardless of screen
or brake mode; the AIRBRAKE screen (below) is a read-only viewport onto this model, not a separate
mode.

**Enable**: `CONFIGBITS_AIRBRAKE` (global `configBits` bit 2), a boolean in the `PREFS` menu after the
main-screen `DISPLAY` (clock/speed) toggle and the `OPS MODE` toggle, default off — an explicit
opt-in, the same way that toggle is. When off, the sound functions are simply not emitted and `AIRBRAKE`/`AIRBRAKE CFG`
are skipped in the top-level menu cycle (still reachable via an `AIRBRAKE`-bound button or
`AIRBRAKE DIAGS`, since the model itself always ticks).

**Automatic-brake rest point.** The automatic-brake apply/release point tracks whichever `BRK TYPE` is
actually active — the committed one, not whatever `OPTION_SCREEN` may be live-editing, so this always
agrees with which mode is really asserting `BRAKE_FN`/`BK2_FN`/`BK3_FN`; see "Committed, not live, config
state" under "On-device config-screen pattern" below — via `independentBrakeAtRest` (computed once per
pass in `mrbw-cst.c`, immediately before the `updateBrake10Hz()` call): Step reuses the 0-20 % rest zone
already tracked by
`brakeState` (`BRAKE_LOW_BEGIN`/`WAIT`), Stack reuses the real band-0 boundary already given by
`currentStackBand == 0` (25 % 3-STEP / 17 % 5-STEP), and Standard/Pulse — neither of which has a
reusable rest-boundary state of its own — share a raw `brakePcnt < 20` fallback.

**Model** (state in milliPSI; no ISR access so no `ATOMIC_BLOCK`):
- **Brake pipe** tracks a lever-derived target, modelled on a 26L automatic brake:
  - at rest (`independentBrakeAtRest`) → fully `BP CHARGE` (release).
  - service zone → at least a **minimum reduction** (`AIRBRAKE_MIN_REDUCTION_PSI`, 7 PSI — a 26L
    cannot make a smaller one), graduated linearly up to a full-service reduction by
    `AIRBRAKE_FULLSVC_PCNT` (70 %) lever. The full-service reduction itself (`FULLSVC`) is not a stored
    field — it is derived fresh every tick as `ceil(2/7 × BP CHARGE)`, so it can never drift out of
    sync with `BP CHARGE` (26 PSI at the default `BP CHARGE = 90`).
  - full service → `AIRBRAKE_EMERG_PCNT` → **laps** at `BP CHARGE − FULLSVC`; more lever does nothing
    (the car aux reservoirs have equalized with their brake cylinders).
  - `≥ AIRBRAKE_EMERG_PCNT` (95 %) → **emergency**: target dumps to 0 — but *only* when the throttle
    `BRK ESTP` option is on (`emergencyBrakeEnabled`). It is the option flag, not the `ESTOP_BRAKE`
    state, so the pipe still models an emergency on the AIRBRAKE screen (which does not suppress the
    real e-stop either, but the flag is what actually gates this). Hysteretic latch
    (`emergencyActive`), releases 5 % below.

  It **vents down** toward the target at a hardcoded `AIRBRAKE_VENT_RATE_PSI_S` (6 PSI/s, or
  `AIRBRAKE_EMERG_VENT_RATE_PSI_S` = 25 PSI/s while `emergencyActive`) while not at rest and above
  target, and **only recharges** once the independent brake is genuinely at rest again. The recharge
  is a **first-order taper** toward `BP CHARGE` (`fill = gap/K + AIRBRAKE_RECHARGE_TAIL_MPSI`, like the
  original ISE sim) rather than linear — `RECHARGE` (PSI/min) is the *initial* rate for a
  full-service-sized gap (`K = fullSvc/chargeRate`), so a bigger gap (recovering from an emergency)
  refills faster at first and everything eases as the pipe fills. The tail term is a small fixed
  crawl (`AIRBRAKE_RECHARGE_TAIL_MPSI`, 0.1 PSI/s, matching the `+10` used by the original), not
  scaled by `RECHARGE`, so the last PSI genuinely creeps in. Easing the lever back while still
  applied just holds — the pipe is not a continuous function of lever position.
- **`appliedLatch`** sets the instant a vent starts and clears only once the independent brake is
  genuinely at rest again — it drives `BRAKE_REL_FN` (below), not a separate apply threshold.
- **Main reservoir** leaks continuously at `LEAKRATE`, and every brake-pipe recharge draws it down by
  `fill × MR LOAD` per tick — a transient trainline-recharge load on the reservoir. Compressor
  governor: on at `MR LOW`, off at `MR HIGH`. Two defensive floors in `updateBrake10Hz()`: `PUMPRATE`
  is floored to always out-pace `LEAKRATE` (else the compressor could never cut out), and `MR HIGH` is
  floored to `MR LOW + 1 PSI` (a `MR LOW ≥ MR HIGH` misconfiguration would otherwise make the two
  governor comparisons fight every tick, toggling `COMPRESSOR_FN` at ~1-2 Hz and sending a status
  packet per edge; there is no on-device or import ordering guard). At the defaults: idle compressor
  cycle ~125 s off / ~24 s on (OFF = `MR HIGH − MR LOW` band ÷ `LEAKRATE` ≈ 120 s, a few seconds
  longer from milliPSI/tick truncation; ON ÷ `PUMPRATE − LEAKRATE`).

**Outputs** (emitted only while the PREFS bit is set):
- **`BRAKE_REL_FN`** = `airBrakeReleased()` = `!appliedLatch`. Its OFF edge (a reduction starting) is
  the counterpart of the brake-set sound; its ON edge (a genuine full release) is the brake-release
  sound.
- **`BRAKE_SET_FN`** ("BRK SET" in Configure Function): a ~1 s pulse (`airBrakeSetPulse()`) at the
  start of **every** brake-pipe reduction, the initial one included — the trainline exhaust hiss.
- **`COMPRESSOR_FN`** / **`COMPRESSOR2_FN`** — the reservoir governor state, split by `COMPMODE` (see
  below). Both cycle on their own timers even at idle, so `COMPRESSOR_FN`/`COMPRESSOR2_FN` and
  `BRAKE_SET_FN` are all excluded from the sleep / alerter activity check in `mrbw-cst.c`
  (`idleFnMask`) — those bits still go out in the status packet, but a throttle with AIRBRAKE on must
  still be able to nod off and still time its alerter. `BRAKE_REL_FN` is *not* excluded (it only
  flips on a real apply/release). The two compressor bits and `BRAKE_SET_FN` are also withheld from
  the last packet(s) before the throttle sleeps (`AIRBRAKE_SLEEP_QUIET_DECISECS`, ~3 deciseconds
  before the timeout, or immediately on a POWER DOWN force-sleep) — the compressor functions in
  particular drive a *continuous* loco sound, and once the radio sleeps the command station just
  holds the last state it heard, so a final "off" has to reach the loco while packets still flow.
  `BRAKE_REL_FN` is a released/applied state (an edge-triggered decoder sound, not continuous), so it
  is left as-is. The underlying model state is untouched by this, so on wake it re-asserts correctly
  if the reservoir is still low.
- `BRAKE_FN` / `BK2_FN` / `BK3_FN` are untouched — `BRK TYPE` still owns real decoder braking; AIRBRAKE
  is sounds only. The override lives in the function-mask assembly in `mrbw-cst.c`, right after the
  `BRK TYPE` machine runs.

**`COMPRSR`/`COMPRSR2` split (`COMPMODE`)**: `AIRBRAKE_COMP_MODE`, `NORMAL` (default) or `CONSIST`.
`NORMAL` always asserts `COMPRESSOR_FN`, matching stock single-loco behaviour exactly. `CONSIST`
splits by whether the current compressor run is servicing a genuinely deep, "synchronised" recharge
(`COMPRESSOR_FN`) versus a routine idle-leak cycle (`COMPRESSOR2_FN`) — the idea being that a deep
brake-pipe recharge is a real trainline-wide event the compressor on every loco should respond to
immediately, while an ordinary idle cycle should stagger per-loco on the decoder/consist side, which
the throttle cannot itself drive but can flag via a second function. Classification
(`airCompressorReleaseRun()`) is decided once, at the exact tick a compressor run starts, and never
changes for the rest of that run — a consist cannot be told "actually, respond as the other case"
partway through a sound that is already playing. It is driven by a decaying "pending consist-sync
credit" (`syncDrawMilliPsi`): each release credits its own full, deterministic reservoir draw
(`gap × MR LOAD`) once, at the instant the release begins — not gradually as the tapered recharge
happens to deliver it, so the credit does not depend on the reservoir level or on how long delivery
takes. It decays at `LEAKRATE`, but only during a genuine quiet gap (never while a recharge is
actively delivering), so several light releases made in quick succession can stack toward the
threshold while the same releases spaced apart are each judged alone. The accumulated credit is
capped at twice the governor band (anything past the threshold already counts as deep, so no
information is lost — a defensive bound against a pathological config where the compressor never cuts
out and the credit could otherwise creep toward a `uint32_t` wrap). A run counts as deep once the
pending credit reaches `AIRBRAKE_SYNC_BAND_PCT` (80 %) of the reservoir governor `MR HIGH − MR LOW`
band — tuned below the full band (which would be the worst-case guarantee regardless of a given
loco starting phase) so a real full-service release reliably clears it while routine idle cycling
does not. `airCompressorPendingRelease()` is a read-only, non-latching accessor exposing this pending
credit for diagnostics (see AIRBRAKE DIAGS below) without affecting the real classification. In
Configure Function, `COMPRESSOR2_FN` ("COMPRSR2") is hidden from the cycle whenever `AIRBRAKE` is off
or `COMPMODE` is `NORMAL` (`advanceCurrentFunction()` takes an explicit `airbrakeEnabled` parameter
rather than reading `configBits` directly, keeping `cst-functions.c` free of any dependency on the
internal bit layout of `mrbw-cst.c`).

**AIRBRAKE CFG** (`AIRBRAKE_CONFIG_SCREEN`, a top-level menu screen modelled on `SPEED CFG`, in the
menu cycle right after it): 9 per-profile bytes, all self-healing via `readByteOrDefault()`, all
always visible (no `ADV FUNC` gating left in this menu) and right-justified to column 7. Items in
menu order — on-screen name (internal enum), default, display suffix:

| On-screen | Internal | Default | Suffix |
|---|---|---|---|
| `BPCHARGE` | `AIRBRAKE_CHARGED` | 90 PSI (range 70-110) | PSI glyph |
| `MR LOAD` | `AIRBRAKE_MR_LOAD` | 35 % (range 0-100) | `%` |
| `MR LOW` | `AIRBRAKE_MR_CUTIN` | 130 PSI | PSI glyph |
| `MR HIGH` | `AIRBRAKE_MR_CUTOUT` | 140 PSI | PSI glyph |
| `RECHARGE` | `AIRBRAKE_CHARGE_RATE` | 180 PSI/min | PSI glyph + `/m` |
| `LEAKRATE` | `AIRBRAKE_LEAK_RATE` | 5 PSI/min | PSI glyph + `/m` |
| `PUMPRATE` | `AIRBRAKE_PUMP_RATE` | 30 PSI/min | PSI glyph + `/m` |
| `DISPLAY` | `AIRBRAKE_DISPLAY` | DUAL | text (`DUAL   `/`SINGLE `) |
| `COMPMODE` | `AIRBRAKE_COMP_MODE` | NORMAL | text (`NORMAL `/`CONSIST`) |

`DISPLAY` selects which of the two `AIRBRAKE screen` renderings to show — `DUAL` (the `BP:`/`MR:`
two-pressure glyph view) or `SINGLE` (the analogue BP dial). Per-profile, like everything else here.

`BPCHARGE` (70-110) and `MR LOAD` (0-100) are the only two items with a meaningful range restriction
(UP/DOWN clamp on-device; PC tooling enforces the same range on import). The other numeric items
(`MR LOW`/`MR HIGH`/`RECHARGE`/`LEAKRATE`/`PUMPRATE`) clamp UP at 254, not 255 — a stored `0xFF` is the
"unset" sentinel `readByteOrDefault()` resets to the default, so a value cranked to 255 would silently
revert on the next load. `FULL SVC`/`VENT`/`EMRG VNT`/`APPLY`/`DRV LOAD` used to be items here too —
all removed (see above and the model description) since none were ever usefully varied per-loco, or
were superseded by deriving/reusing a value that already exists elsewhere.

A layout change here needs the usual `EEPROM_LAYOUT_VERSION` bump (see the maintenance checklist). The
`AIRBRAKE CFG` bytes occupy `0x4A-0x53` (with the `HORN2` and `COMPRESSOR2` function slots at `0x49`/
`0x52`); the SPEED model payload follows at `0x54-0x63`. The layout 1 -> 2 migration in
`applyEepromMigrations()` (`cst-eeprom.c`) force-resets the `AIRBRAKE` region to defaults on a version
upgrade, so a configured throttle should be exported with `cst_cfgtransfer.py` before an
`AIRBRAKE`-layout upgrade and re-imported afterward — the later SPEED 2 -> 3 migration, by contrast,
relocates rather than resets (see the SPEED section).

**AIRBRAKE screen** (`AIRBRAKE_SCREEN`; reached from the top-level menu when `AIRBRAKE` is on, or any
time via a control set to `FN_AIRBRAKE`): a read-only viewport into the always-running model — it
blocks nothing, so the brake lever drives real decoder braking and the real e-stop from here exactly
as from the main screen, and the speed sim keeps running in parallel. No landing page/subscreen: it
renders straight away, so the top-level MENU handler keeps cycling the menu past it. UP/DOWN do
nothing here; the rendering is set by the `DISPLAY` item in `AIRBRAKE CFG` (`DUAL` default /
`SINGLE`), read fresh on every render pass:
- **`DISPLAY = DUAL`** (default): row 0 `BP:` + 3-digit brake-pipe PSI + a 2-cell hand-drawn "PSI"
  glyph (`PSI_CHAR_L`/`R`, CGRAM slots 6-7 under `LCD_DEFAULT`; the base and OPS MODE screens borrow
  those two slots for the "A" and "Fn active" glyphs, so this screen reloads `LCD_DEFAULT` on entry);
  row 1 `MR:` + 3-digit reservoir PSI +
  the same glyph. Both readouts (`airBrakePipePsi()`/`airMainResPsi()`) round to the nearest whole PSI
  rather than truncating, so `MR HIGH`/`MR LOW` visibly dwell at the top/bottom of each idle
  compressor cycle instead of flashing past — the model never overshoots a governor setpoint by more
  than a fraction of a PSI, which a truncating display would barely show.
- **`DISPLAY = SINGLE`**: the original ISE analogue pressure gauge (`Gauge[2][4][8]` dial artwork +
  Bresenham-plotted needle, recovered from the last pre-AIRBRAKE commit and now wired to the brake
  pipe instead of the old `PumpState` model) plus a 3-digit BP readout and literal `" PSI"` text —
  unchanged from the original. Needle sweep range maps `0..BP CHARGE` (the *configured* max,
  not a hardcoded value) across the dial. Runs in its own CGRAM mode (`LCD_AIRBRAKE_ALT`, all 8 slots
  — `setupGaugeChars()` rebuilds and re-uploads the needle glyph every render pass, since it moves
  live). Whichever style the config selects, `setupLCD()` is called with the matching CGRAM mode
  (`LCD_AIRBRAKE_ALT` / `LCD_DEFAULT`) on every render pass — its `currentMode` guard makes the repeat
  calls free, and changing `DISPLAY` and returning restores the right glyphs automatically.

How this screen is dismissed depends on how it was entered (see "OPS MODE screen" for the two
`airbrakeReturn*` flags): from the menu cycle, `SELECT` exits to the main screen and `MENU` advances
to the next menu screen; from a base-screen `UP` / `DOWN` button set to `AIRBRAKE`, any of the four
buttons dismisses to the main screen; from an OPS MODE button set to `AIRBRAKE`, any of the four
returns to the OPS MODE screen. Reached from either running-screen path the screen honours the
`backlight` toggle rather than forcing the panel on (see "Menu backlight hold"); the menu-cycle entry
stays always-lit.

The full text/diagnostic readout (lever %, per-function letters) lives on **AIRBRAKE DIAGS**, a
`DIAG_SCREEN` subscreen (page 14) shown only while `AIRBRAKE` is on — reached via `DIAGS` → `SELECT`
(lands on the throttle-status page) → `MENU` (advances straight to AIRBRAKE DIAGS, since it is
slotted in right after the status page) → `MENU` again continues the normal DIAGS cycle. Row 0:
`P<pipe> R<reservoir>` PSI — the pipe shows two digits with a blank column before `R`, or three
digits filling that column at 100 and above. Row 1: `L<lever%>` then one column per AIRBRAKE
function, its letter shown only while asserting — `R` = `BRAKE_REL_FN`, `S` = `BRAKE_SET_FN` pulse,
`E` = `airEmergencyActive()` (the AIRBRAKE model emergency pipe-dump latch, not the throttle-wide
`THROTTLE_STATUS_EMERGENCY` that `EMRG FN` tracks). The compressor column is 3-way: `C` while
`airCompressorOn()`, or `*` while off but `airCompressorPendingRelease()` says the pending
consist-sync credit already clears the deep/`COMPRSR` threshold (lets you watch it accumulate and
leak away between runs) — both only meaningful/shown when `COMPMODE = CONSIST`. An adjacent column
shows `1`/`2` while the compressor is on and `COMPMODE = CONSIST` (which classification the current
run got), blank in `NORMAL` mode.

### Reference-trace test

`make pressuretest` runs `src/cst-pressure-test/` — the AIRBRAKE counterpart of `make speedtest`.
`test_pressure.c` host-compiles `cst-pressure.c` whole (no include-path shims — `cst-pressure.h`
pulls only `<stdint.h>`), drives `updateBrake10Hz()` through a fixed scenario set (idle governor
cycle, initial full-service reduction and release, minimum-reduction floor, graduated service,
lapping, emergency with and without `BRK ESTP`, the `COMPMODE = CONSIST` deep/routine split, and the
`PUMP <= LEAK` / inverted-`MR` guards), and diffs the per-tick model state (`bpMilliPsi`,
`mrMilliPsi`, the consist-sync credit) plus all six output accessors and the two rounded whole-PSI
readouts against `reference/*.txt`. `make pressuretest-accept` re-blesses; a diff is the
human-readable statement of how the model moved. `main()` also asserts three invariants as
`PASS`/`FAIL` lines (idle governor bounded and cycling, inverted-`MR` band does not stutter, the
brake pipe recharges to *exactly* `BP CHARGE`) and exits non-zero on failure. `.githooks/pre-commit`
runs it whenever a commit touches `cst-pressure.c`, `cst-pressure.h`, or that directory. Unlike the
scale-speed model, `updateBrake10Hz()` is width-identical between AVR 16-bit and host 32-bit `int` by
construction (`uint32_t` / `UL` throughout), so there is no "safe range" caveat — the one host/AVR
divergence, the `rand()` gauge jitter in `initAirBrake()`, is pinned out by the harness. The
`P`/`X` (pending-deep / run-is-deep) columns are computed every tick regardless of `COMPMODE`, so
they show up in `NORMAL`-mode traces too; `AIRBRAKE_DISPLAY` is not covered (it is a `mrbw-cst.c`
render selector, not model state). It locks the *current* behaviour so future tuning is safe — it
does not validate the model-shape constants below.

**Not yet done**: independent real-hardware validation of the fixed model-shape constants in
`cst-pressure.c` — `AIRBRAKE_MIN_REDUCTION_PSI`, `AIRBRAKE_FULLSVC_PCNT`, `AIRBRAKE_EMERG_PCNT`,
`AIRBRAKE_VENT_RATE_PSI_S`/`AIRBRAKE_EMERG_VENT_RATE_PSI_S`, `AIRBRAKE_RECHARGE_TAIL_MPSI`, and
`AIRBRAKE_SYNC_BAND_PCT` are carried over from real 26L brake-valve convention or the original ISE
sim rather than independently bench-tuned for this fork (see the comment on each `#define`). The
per-profile `AIRBRAKE CFG` defaults above, by contrast, have been bench-tuned against real hardware.

## Long-press Menu to cancel a subscreen edit

Every on-device config screen has two ways out: `SELECT` saves the edited values to EEPROM; a long-press
(~500ms — measured against real hardware, the same threshold and idiom already shared by UP/DOWN autorepeat
and the SELECT-long-press-to-power-down of the main screen) of the top-left Menu button discards them
instead and returns to the main screen. Implemented as one new `else` branch alongside the existing
top-level Menu-cycling logic in `mrbw-cst.c`: on a long-press while inside a subscreen, it calls
`readConfig()` (already a complete undo, since nothing reaches EEPROM without an explicit `SELECT`-save)
and resets navigation state back to the main screen.

Two kinds of state need explicit handling, since neither is backed by EEPROM: `systemBits`
(the menu-lock/advanced-function bits of `SYSTEM_SCREEN`) is a session-only global, restored from a
snapshot captured whenever `SYSTEM_SCREEN` is entered — taken *after* all the conditional-menu and
menu-lock skip logic, so no entry path misses it. And the `new*` staging locals of `PREFS_SCREEN`
(`SLEEP DLY`/`ALERTER`) and `COMM_SCREEN` (`THRTL ID`/`BASE ADR`/`TIME ADR`/`TX INTVL`) — which those
screens only push to the real values on a `SELECT`-save and never resync on entry — are resynced from
the (`readConfig()`-restored) real values on cancel, so an abandoned edit cannot linger and be
silently committed on a later visit.

Every exit back to the main screen (`case LAST_SCREEN`) also calls `setupLCD(LCD_DEFAULT)` to restore the
default LCD custom characters — a defensive backstop for any screen that reprograms CGRAM and any exit
path (the generic cancel handler has no `currentMode` knowledge), free on every other exit since
`setupLCD()` already guards against a redundant call.

Screens with their own pre-existing short-press Menu escape (the power-down confirm subscreen, the network
CNF picker) already reset their subscreen state well before the long-press threshold, so this does not
interfere there; the blocking radio calls of the shared network CNF sync pause the whole main loop
(including button polling) for their duration, so they cannot race this either.

## Menu backlight hold

Most non-`MAIN_SCREEN` cases in the `switch(screenState)` call `enableLCDBacklight()` unconditionally each
pass, so the backlight is lit for as long as you sit on a menu screen. The running screens honour the
`backlight` toggle set by the user (flipped by SELECT) instead: the main screen, `OPS_MODE_SCREEN` (a
base-screen variant where every button is a function tap — see "OPS MODE screen"), and the `AIRBRAKE`
gauge when it was reached from either of those (its menu-cycle entry stays always-lit). With the toggle
off the main screen called `disableLCDBacklight()` immediately, every pass — so the instant a menu cycle
wrapped back through the main screen the light went dark, strobing off mid-navigation when starting
another lap.

`backlightTimeout_decisecs` is a hold countdown (`BACKLIGHT_HOLD_DECISECS`, ~3s) decremented in the 10Hz
block of `TIMER0_COMPA_vect`, next to `sleepTimeout_decisecs`/`alerterTimeout_decisecs` — a `uint8_t`,
so reads/writes are atomic on the AVR with no `ATOMIC_BLOCK`. It is re-armed once per main-loop pass
(beside the sleep/alerter timer resets) whenever the current button is `MENU` *or* the screen is not a
running screen: menu navigation and MENU presses keep it full, so the light survives the wrap back
through the main screen for the hold period. `OPS_MODE_SCREEN` and the running-entry `AIRBRAKE` gauge are
excluded from the re-arm the same way the main screen is (the hold armed while entering covers the
transition). The main screen two backlight branches (normal and `holdFunctionActive`) gate on
`backlight || backlightTimeout_decisecs`; `EMRG`/`ALERTER`-blink/`REV!` are unchanged. UP/DOWN
function taps on the main screen deliberately do *not* re-arm it (night-operation friendly), and an explicit
SELECT toggle-to-off also zeroes the countdown so the light drops at once.

**SELECT backlight toggle fires on release, not on the press edge.** Stock firmware toggled `backlight` the
moment SELECT went down, so beginning a SELECT-long-press to power down always flipped the backlight first
as a side effect. The toggle now happens when SELECT is released below the long-press threshold, gated by a
`selectShortPressArmed` flag set only on a genuine main-screen press edge — which also keeps a
wake-from-sleep SELECT (where `previousButton` is force-synced, so no edge is seen) from spuriously
toggling.

## OPS MODE screen

**OPS MODE** is an opt-in variant of the base screen that rebinds `MENU` and `SELECT` from menu
navigation to two more assignable DCC functions while running. It is entered by a long-press of `MENU`
from the base screen and left the same way; it is never part of the menu cycle, and it only exits back
to the base screen. Opt-in via the `PREFS` item **OPS MODE** (`CONFIGBITS_OPS_MODE`, bit 3), ordered
right after `DISPLAY`, default off — a stock or upgrading throttle has the bit clear and sees no change
on the base screen. `OPS_MODE_SCREEN` is a `Screens` value with no subscreens; `DIAG_SCREEN` cycling
skips straight past it to `LAST_SCREEN`.

**`MENU BTN` / `SEL BTN` functions** (`MENU_FN` / `SEL_FN` in the `Functions` enum, right after
`DOWN_FN`) carry the same `SOFTWARE_LATCH | SPECIAL_FUNC | MENU_FUNC` attributes as `UP BTN` /
`DOWN BTN`, so `CONFIG FUNC` walks them right after `DOWN BTN` and offers F00..F28 momentary/latching,
`EMRG BRK` and `AIRBRAKE`. They are driven through the same `optionButtonState` mechanism
(`MENU_OPTION_BUTTON` / `SEL_OPTION_BUTTON`) and folded into `functionMask` every pass regardless of
`screenState`, so a latched `MENU BTN` / `SEL BTN` keeps asserting after OPS MODE is left. Their bits
are only ever *set* while `screenState == OPS_MODE_SCREEN`; turning the pref off does not clear a
latched bit. `MENU_FN` / `SEL_FN` are not in `idleFnMask`, so they count as crew activity for the
sleep and alerter timers like `UP_FN` / `DOWN_FN`.

**Base-screen layout.** Grids below are 0-indexed `(col,row)` on the 8x2 LCD. With
`CONFIGBITS_OPS_MODE` **clear** the base screen is byte-identical to the pre-OPS-MODE firmware:
battery `(0,0)`, loco / `EMRG` / `ALRT` / `REV!` / `HOLD` `(2,0)`, `printSpeed()` / `printTime()`
`(1,1)`, `UP` glyph `(7,0)`, `DOWN` glyph `(7,1)`, `AUX` glyph or blank `(0,1)`. With the bit **set**,
the base screen and the OPS MODE screen share one aligned layout: battery moves to `(6,0)`, `AUX` to
`(1,0)`, and column 0 of both rows becomes a status-glyph cell — the `MENU` / `SEL` circle (hollow or
filled, exactly like the `UP` / `DOWN` glyphs) on the OPS MODE screen, or the "Fn active" reminder
glyph (blank unless that function is still latched) on the plain base screen. Loco, speed/clock and the
`UP` / `DOWN` glyphs do not move; a 3-digit speed still fits.

**Shared renderer.** `renderBaseScreen(opsScreen, ...)` in `mrbw-cst.c` draws the status area for both
`MAIN_SCREEN` and `OPS_MODE_SCREEN` so the two cannot drift; only the three cells above depend on the
pref and on which screen is showing. `printBattery()` takes an `x`-column parameter for this. Button
handling stays in each `case` (it is what actually differs). Two small helpers:

- **`buttonCornerGlyph(fn, asserting)`** — looks up whichever CGRAM slot `allocateSpecialGlyphSlots()`
  assigned this render pass (see "CGRAM" below) for the button's current special value: a bold custom
  "A" for `AIRBRAKE` (a screen jump rather than a DCC function, so the softkey circle would be
  meaningless), the `LOAD` glyph (see "LOAD button function"), or the `CLOCK` peek glyph (see "CLOCK
  Peek") — or otherwise the filled/hollow circle, filled while the function is asserting and
  configured, hollow otherwise (shown even when the function is `FN_OFF`). Used for all four button
  corners on the base and OPS MODE screens — `UP` / `DOWN` at `(7,*)` on every base/OPS screen, `MENU`
  / `SEL` at `(0,*)` on the OPS MODE screen only.
- **`auxIndicatorChar()`** — the `AUX` indicator glyph, or a blank. Suppressed when `AUX` drives the
  very DCC function `HOLDFN` watches for ESU Drive Hold: activating `AUX` then already replaces the
  loco address with `HOLD`, so the glyph would be noise. A static config comparison
  (`getFunctionMask(AUX_FN)` against `1 << HOLDFN`), not a runtime `holdFunctionActive` check —
  whenever `AUX` is both active and mapped to the `HOLDFN` number, Drive Hold is asserted by
  construction. The `DIAG` screen `AUX` indicator is unchanged.

**Entry and exit timing.** While `CONFIGBITS_OPS_MODE` is set the base-screen menu advance
(`screenState++`) is deferred to `MENU` release through a `menuAdvancePending` one-shot: a `MENU` tap
advances the menu on release, a `MENU` hold jumps straight to `OPS_MODE_SCREEN` with no intervening
screen shown. `opsMenuIgnoreUntilRelease` keeps the still-held `MENU` that entered OPS MODE from
immediately tripping the exit long-press. A `MENU BTN` set to `AIRBRAKE` opens the gauge on the
trailing edge of a short tap, so a long `MENU` hold still exits OPS MODE first. Every other screen, and
the whole OPS-disabled build, keep the stock press-edge advance.

**AIRBRAKE from OPS MODE / the base screen.** Two parallel `main()` flags distinguish the three ways
the `AIRBRAKE` gauge is reached and how it is dismissed — see the "AIRBRAKE screen" list under
AIRBRAKE. `airbrakeReturnToOps` (a `MENU` / `SEL` / `UP` / `DOWN` button set to `AIRBRAKE`, pressed in
OPS MODE) returns to `OPS_MODE_SCREEN` on any of the four buttons; `airbrakeReturnToMain` (a `UP` /
`DOWN` button set to `AIRBRAKE`, pressed on the base screen) dismisses to the main screen on any of the
four; the menu-cycle entry keeps its stock behaviour. The triggering button momentary bit is cleared
at every open site so a still-held button cannot re-open the gauge on return, and while either flag is
set the top-level `MENU` handler is bypassed.

**CGRAM.** `MAIN_SCREEN` and `OPS_MODE_SCREEN` each use their own `LcdMode` palette — `LCD_MAIN` /
`LCD_MAIN_SPEED` and `LCD_OPS` / `LCD_OPS_SPEED` respectively, selected by `baseScreenLcdMode(opsScreen)`
off `CONFIGBITS_MAIN_SCREEN_SPEED`. Both palettes are `LCD_DEFAULT`-shaped, and — under the `_SPEED`
variants only — slot 3 is reused as `SPEED_H_CHAR`, the narrow "H" of `MPH` / `KMH` in the running speed
readout; the non-speed variants hold `AMPM_CHAR` (the clock indicator, see below) in that same slot
instead. `LCD_MAIN`(`_SPEED`) additionally, statically loads slot 7 (`PSI_CHAR_R`) as
`OPS_FN_ACTIVE_CHAR`; `LCD_OPS`(`_SPEED`) does not, since `OPS_MODE_SCREEN` never draws that glyph
(only the plain base screen "still latched" reminder needs it) — that slot instead joins the pool
described next.

Every other button-corner glyph — the plain hollow/filled softkey circle, plus every icon-bearing
special button function (`AIRBRAKE`, `LOAD`, `CLOCK` peek — see "LOAD button function" and "CLOCK
Peek" below) — is resolved from a **dynamic pool** of CGRAM slots, reallocated fresh every render pass
by `allocateSpecialGlyphSlots()` (`mrbw-cst.c`), rather than each being fixed to one slot the way
`AIRBRAKE_GLYPH_CHAR`/`LOAD_CHAR` originally were. The underlying argument is a pigeonhole one, not a
policy, and it is what makes the architecture future-proof rather than tuned to today's specific 3
special functions: a screen that draws N button corners can never need more than N distinct corner
bitmaps at once, because each button resolves to exactly one concept (plain-hollow, plain-filled, or
one special function) per render pass — regardless of how many concept *types* exist in total.
- `MAIN_SCREEN` draws only 2 corners (`UP BTN`/`DOWN BTN` — `MENU BTN`/`SEL BTN`'s own corners are
  never drawn there, only their separate `OPS_FN_ACTIVE_CHAR` reminder is), so its dedicated 2-slot
  pool (slots 4 and 6, borrowing `PSI_CHAR_L`'s slot number) always covers its worst case, no matter
  how many special function types are ever added. The softkey circle stays at its fixed slots (1/2)
  there instead of joining the pool — 2 slots already suffice for 2 buttons, so there is nothing to
  gain by making the circle movable too.
- `OPS_MODE_SCREEN` draws all 4 corners. Its non-circle pool alone (slots 4, 6, and 7) happens to
  exactly match today's 3 special function types, which is a coincidence, not headroom: a 4th
  icon-bearing special function would need a 4th simultaneous slot the moment all 4 buttons held 4
  different values. Folding the softkey-circle slots (1, 2) into the *same* pool — 5 candidate slots
  total — removes that ceiling permanently: with 4 buttons, at most 4 distinct concepts are ever
  needed at once (out of hollow, filled, and however many special functions exist), so 5 candidates
  always leave at least one spare.

Adding a future icon-bearing special function needs no change to this pool architecture at all — only
the same three pieces every existing one already has: an `isFunctionXxx()` predicate
(`cst-functions.c`), a `needXxx` check plus a `setupXxxChar(slot)` glyph loader wired into
`allocateSpecialGlyphSlots()` (`mrbw-cst.c`) and `cst-lcd.c` respectively, and a case in
`buttonCornerGlyph()` to read the resulting tracking slot back. `cst-common.h` carries the full
derivation next to the slot `#define`s.

Every transition to a menu or `AIRBRAKE` screen forces `LCD_DEFAULT`, whose `currentMode` guard
reloads the displaced `PSI` glyphs and the fixed-slot softkey circle; changing the `DISPLAY` pref
always round-trips through `LCD_DEFAULT`. The battery `FULL` / `HALF` / `EMPTY` glyphs are narrow
3-pixel bitmaps.

**AM/PM clock indicator (`AMPM_CHAR`).** One dynamically-rewritten slot rather than the two static ones
(`AM_CHAR`/`PM_CHAR`) this replaced, since the clock is drawn at exactly one screen position and changes
at most twice a day. `invalidateAmPmChar()` (`cst-time.h`) is the only thing `setupLCD()` calls on
entering `LCD_MAIN` or `LCD_OPS` (the non-speed variants) — it resets a tracked sentinel rather than
writing a bitmap, since there is no live "current AM/PM" value available outside a render pass for
`cst-lcd.c` to write from. The guaranteed same-pass call to `printTime()` from `renderBaseScreen()` then
performs the actual write, inside `displayTime()` (`cst-time.c`), only when the AM/PM state differs from
what is already drawn — mirroring the change-detection in `printBattery()` (`cst-battery.c`), and the same
"runtime-only, never written by `setupLCD()`" contract `LOAD_CHAR` already uses.

**EEPROM layout 4 → 5.** `EE_MENU_BUTTON_FUNCTION` / `EE_SEL_BUTTON_FUNCTION` occupy the freed
per-profile holes `0x2C` / `0x2D` (former SPEED `BRK2` / `BRK3` scatter slots). The migration seeds
both to `FN_OFF`; see "EEPROM layout and the reference-trace test" under SPEED for the migration
detail and the paired `3 → 4` gate change. `0x2C` / `0x2D` are function bytes owned by
`cst-functions.c`, not model bytes, so `eepromResetProfileModel()` is unchanged.

**PC tooling.** `functions` gains `MENU_BUTTON` / `SEL_BUTTON` (EEPROM `0x2C` / `0x2D`, same value
vocabulary as `UP_BUTTON` / `DOWN_BUTTON`), `prefs.config_bits` gains `ops_mode`, and
`SLOT_SCHEMA_VERSION` moves 5 → 6. A pre-6 backup missing the two function keys needs `--import-old`
(they default to `RAW:0xFF`, like any never-written function slot).

Sleep: `screenState` is not reset on wake, so a throttle that sleeps in OPS MODE wakes back into it —
a base-screen variant is exactly where it should resume.

## LOAD button function

A fifth special value for the same four configurable buttons `AIRBRAKE` already occupies (`UP_FN` /
`DOWN_FN` / `MENU_FN` / `SEL_FN`) — `FN_LOAD`, menu name "LOAD", spliced into the value cycle directly
between `EMRG BRK` and `AIRBRAKE`. Each momentary press advances a 3-way state, OFF -> OPLOAD -> PRLOAD ->
OFF, asserting whichever DCC function `OPLOADFN` / `PRLOADFN` (see "Load simulation" under SPEED above)
is currently configured to — so one button can drive the ESU Optional/Primary Load simulation without
separately wiring another control to the same function number. Offered as a CONFIG FUNC choice only while
SPEED is enabled (`CONFIGBITS_MAIN_SCREEN_SPEED`) and the profile `TYPE` models the load CVs (`V5DCC` /
`V5MULT`, not `V4` — `speedTypeHasLoad()`).

**Committed, not live.** `OPLOADFN`/`PRLOADFN` are read from a committed snapshot
(`committedOploadFn`/`committedPrloadFn`, `mrbw-cst.c`), not the live `speedCfg[]` entries
`SPEED_CONFIG_SCREEN` edits directly — necessary because the `TYPE` `UP`/`DOWN` handler calls
`speedResetModel()` live, on every press, which forces both function numbers to the inert `OFF` sentinel
the instant `TYPE` is tentatively cycled to `V4`; without the committed indirection, an already-asserting
LOAD button would stop the moment `TYPE` was merely browsed past `V4`, before ever confirming the change.
See "Committed, not live, config state" under "On-device config-screen pattern" below for the general
pattern, which also covers whether LOAD is eligible to assert at all (`loadEligible()`).

**Restricted to one button at a time, enforced in firmware.** `loadUsedElsewhere()` (`cst-functions.c`)
skips `FN_LOAD` in the CONFIG FUNC value cycle for any button while another of the four already holds it.
This is a hardware constraint, not a style choice: the LOAD indicator needs its bitmap dynamically
rewritten to match whichever button currently holds it — unlike a purely static bitmap (`AIRBRAKE`'s or
`CLOCK` peek's), which can be assigned to any number of simultaneous corners with no rewriting at all,
since every holder wants identical content. A single dynamically-rewritten slot is only ever correct when
exactly one corner references it; two simultaneous holders in different states would show whichever was
written most recently at both corners, since CGRAM slot content is global, not per-cell.

**CGRAM.** The indicator (`cst-lcd.c`'s `setupLoadChar(slot, loadMode)`) draws from the same dynamic pool
as `AIRBRAKE`/`CLOCK` peek — see "OPS MODE screen" CGRAM above for the full pool architecture.
`setupLCD()` never writes to any pool slot in any mode — it stays exclusively managed by
`allocateSpecialGlyphSlots()`, called every render pass from `renderBaseScreen()`, which resolves
`loadModeUp`/`Down`/`Menu`/`Sel` (whichever single button currently holds LOAD, per the restriction
above) into whichever slot the pool assigns that pass. `loadActive(fn)` / `loadEligible()` (`mrbw-cst.c`)
still gate every runtime use of LOAD — the press-edge advance, the `functionMask` assembly,
`buttonCornerGlyph()`, and the pool allocation itself — but this is a product-behavior choice rather than
a CGRAM-safety requirement: an already-assigned LOAD button is deliberately left going fully inert the
moment `DISPLAY` drops back to `CLOCK`.

**Base-screen "Fn active" reminder.** A LOAD-configured `MENU_FN` / `SEL_FN` never sets the
`optionButtonState` bit (its press-edge handler advances the persistent `LoadMode` state directly instead
of the ordinary momentary/latching path), so the plain base screen "Fn active" reminder glyph
(`OPS_FN_ACTIVE_CHAR`, see "OPS MODE screen" above) — the only indication of MENU BTN / SEL BTN activity
outside `OPS_MODE_SCREEN`, since their corner is not drawn there — separately treats "currently OPLOAD or
PRLOAD" as the LOAD equivalent of "latched and on"; it does not fall out of the ordinary
`optionButtonState`/`isFunctionOff()` check every other latching function relies on.

**Stale-assignment cleanup.** `printCurrentFunctionValue()` (CONFIG FUNC) is unconditional, so a button
already holding `FN_LOAD` would otherwise keep showing "LOAD" indefinitely once LOAD becomes unreachable
— the value cycle only guards *selecting* `FN_LOAD`, not an existing stored value. `clearLoadFunctions()`
(`cst-functions.c`) resets any of the four buttons holding `FN_LOAD` to `FN_OFF` and persists directly
(not via `writeFunctionConfiguration()`, which would also re-persist every other slot at its current RAM
value), called from two `SELECT`-save sites: SPEED CFG, whenever the saved `TYPE` no longer models the
load CVs, and PREFS, whenever the saved `DISPLAY` no longer shows SPEED — the same reasoning either way,
since both conditions drop `loadEligible()` to false.

No `EEPROM_LAYOUT_VERSION` bump: `FN_LOAD` is a new value within the existing function-value byte range
(same encoding as `FN_OFF` / `FN_EMRG` / `FN_AIRBRAKE`), not a new field or repurposed byte — same
precedent as the `V4` split of `SPEED_TYPE_V5MULT`. No `SLOT_SCHEMA_VERSION` bump either, for the same
reason `"AIRBRAKE"` never needed one: a new legal string within an existing field type, not a JSON shape
change.

**PC tooling**: `cst_eeprom_layout.py` mirrors the firmware `LOAD_FUNC` attribute bit as `FUNC_LOAD`
(`0x08`), set on the same four `FUNCTION_FIELDS` entries — `UP_BUTTON`/`DOWN_BUTTON`/`MENU_BUTTON`/
`SEL_BUTTON` — as `FUNC_MENU` (`FN_AIRBRAKE`), and registers `FN_LOAD` (`0x82`) as `"LOAD"` in the
function-value maps. `slot_codec.py` own `_encode_functions()` rejects `"LOAD"` on any other field, and —
mirroring `loadUsedElsewhere()` (`cst-functions.c`), since two simultaneous holders would each overwrite
the one shared CGRAM slot with their own state (see "CGRAM" above) — also rejects it on more than one of
the four buttons at once, a check the on-device menu enforces live but a hand-edited JSON import could
otherwise bypass.

## CLOCK Peek

A sixth special value for the same four configurable buttons `AIRBRAKE`/`LOAD` occupy (`UP_FN` /
`DOWN_FN` / `MENU_FN` / `SEL_FN`) — `FN_CLOCK`, menu name "CLOCK", spliced into the value cycle directly
after `AIRBRAKE` (the last position before wrapping to `OFF`). While the configured button is
physically held, the `(1,1)` speed readout is replaced by the fast-clock readout (`printTime()`);
releasing the button reverts immediately — `printSpeed()`/`printTime()` draw an identical 6-character
field at the same position, so the swap is just "call the other function instead," no `lcd_gotoxy()`
bookkeeping needed. Offered as a CONFIG FUNC choice only while SPEED is enabled
(`CONFIGBITS_MAIN_SCREEN_SPEED` — `clockEligible()`, `mrbw-cst.c`), since there is nothing to "peek"
away from otherwise; unlike `LOAD` there is no decoder-`TYPE` dependency, and no committed/live
snapshot is needed either — `CLOCK` never reaches `functionMask` (see below), so there is no "edited
elsewhere, read here mid-edit" hazard the way `LOAD`'s `TYPE`/`DISPLAY` interaction has.

**Not a DCC function.** Like `AIRBRAKE`, `FN_CLOCK` never reaches `functionMask` — `getFunctionMask()`/
`isFunctionEstop()` already return 0/false for any value outside the `F00`-`F28` ranges and `FN_EMRG`,
so no code change was needed there. Unlike `AIRBRAKE` (a screen jump) or `LOAD` (a persistent 3-way
state advanced on the press edge), `CLOCK` needs no press-edge interception at all: it is not listed in
`isFunctionLatching()`'s switch, so it falls through to the default "momentary" case for free — the
existing generic `optionButtonState` press/release bookkeeping already tracks "is this button currently
held" for any plain momentary function, and `renderBaseScreen()` just reads that same bit
(`clockPeekHeld`, OR'd across all four buttons) to decide which readout to draw.

**CGRAM.** The corner icon (`cst-lcd.c`'s `setupClockPeekGlyphChar(slot)`) draws from the same dynamic
pool as `AIRBRAKE`/`LOAD` — see "OPS MODE screen" CGRAM above for the full architecture — and is shown
for as long as the assignment exists, not gated on the button being held (only the readout swap above
is momentary; matching `AIRBRAKE`'s always-on convention makes the corner a configuration reminder, not
a press indicator).

The readout swap itself touches a *different* CGRAM slot entirely (3, shared between `SPEED_H_CHAR` and
`AMPM_CHAR` — see "AM/PM clock indicator" above) and needs its own care, independent of the pool: while
peeking, `printTime()` is called without leaving `LCD_MAIN_SPEED`/`LCD_OPS_SPEED` (`DISPLAY` is still
SPEED), so if the fast clock is in 12-hour mode, `displayTime()` unconditionally overwrites slot 3 with
the real AM/PM bitmap — correctly, since showing AM/PM during a peek is the point. But nothing else
would then restore the "H" bitmap once the peek ends, so `renderBaseScreen()`'s non-peek path
explicitly calls `setupSpeedHChar()` (restoring "H") *and* `invalidateAmPmChar()` (resetting
`displayTime()`'s change-detection sentinel, `cst-time.c`'s `lastAmPm`) every pass `printSpeed()` runs.
The second call is not optional: `displayTime()` only rewrites slot 3 when the AM/PM *value* differs
from the last one it drew, so without invalidating that cache, a second peek within the same AM/PM
half-day would see "no change" and skip the rewrite — even though slot 3's physical content was
overwritten by `setupSpeedHChar()` in the meantime, leaving "H" stuck in the readout instead of the
correct AM/PM glyph.

**Stale-assignment cleanup.** `clearClockFunctions()` (`cst-functions.c`) mirrors `clearLoadFunctions()`
exactly — resets any of the four buttons holding `FN_CLOCK` back to `FN_OFF` (RAM and EEPROM directly),
called from the same `PREFS` `SELECT`-save site as `clearLoadFunctions()`, guarded by the same
`if(!(configBits & _BV(CONFIGBITS_MAIN_SCREEN_SPEED)))`: once `DISPLAY` drops back to `CLOCK`, there is
nothing left for `CLOCK` peek to peek away from.

No `EEPROM_LAYOUT_VERSION` bump: `FN_CLOCK` is a new value within the existing function-value byte
range (`0x83`, the next free byte after `FN_LOAD`'s `0x82`) — same precedent as `FN_LOAD`/`FN_AIRBRAKE`.

**PC tooling**: `cst_eeprom_layout.py` mirrors the firmware `CLOCK_FUNC` attribute bit as `FUNC_CLOCK`
(`0x10`), set on the same four `FUNCTION_FIELDS` entries as `FUNC_MENU`/`FUNC_LOAD`, and registers
`FN_CLOCK` (`0x83`) as `"CLOCK"` in the function-value maps. `slot_codec.py`'s own `_encode_functions()`
rejects `"CLOCK"` on any other field, mirroring the `"AIRBRAKE"`/`"LOAD"` field-restriction checks — but
unlike `"LOAD"`, it needs **no** multi-holder rejection, since `CLOCK` has no single-owner restriction
(any number of buttons may hold it at once, confirmed by `test_clock_fn_allowed_on_more_than_one_button`
in `test_slot_codec.py`). No `SLOT_SCHEMA_VERSION` bump, for the same reason `"AIRBRAKE"` never needed
one: a new legal string within an existing field type, not a JSON shape change.

## On-device config-screen pattern

Every editable config menu (`SPEED CFG`, `AIRBRAKE CFG`, `OPTIONS`, `SYSTEM`, `COMM CFG`, `PREFS`,
`FORCE FUNC`, `CONFIG FUNC`, `NOTCH`, `THRESHOLD CAL`) is one `case` in the `switch(screenState)` in
`mrbw-cst.c` and shares a two-level shape driven by `subscreenState` (a `main()` local):

- **`subscreenState == 0`** is the landing page — the title plus a `-` cue. `SELECT` sets
  `subscreenState = 1` to enter the item list.
- **`subscreenState >= 1`** selects one item. `MENU` advances (`subscreenState++`, wrapping back to
  `1` past the last item), `UP`/`DOWN` edit, `SELECT` writes every field to EEPROM and drops back to
  the landing page with a `SAVED!` flash. A long-press of `MENU` anywhere in the list discards the
  edit and exits to the main screen (see "Long-press Menu to cancel a subscreen edit").

**Editor ceiling for a `readByteOrDefault` byte is 254, not 255**: a stored `0xFF` is that helper
"unset → default" sentinel, so a value cranked to 255 silently reverts on the next load. `AIRBRAKE
CFG`, the non-full-range `SPEED CFG` numerics, and `TX HLDOF` (`COMM CFG`) all cap at 254 for this
reason; for `TX HLDOF` `readConfig()` additionally heals a stored `0xFF` to `TX_HOLDOFF_DEFAULT`. The
exceptions are the five raw-read `SPEED` bytes — `ACCEL`/`DECEL` (0-255), `HOLDFN` (`OFF`),
`ACCELADJ`/`DECELADJ` (−127…+127) — see the SPEED editor section.

Two item-dispatch styles are in use:

1. **Indexed accessor** (`AIRBRAKE_CONFIG_SCREEN`): a named-item enum in the module header
   (`AIRBRAKE_CHARGED` … `AIRBRAKE_COUNT`), one `xGet(item)` / `xSet(item, value)` pair over a
   private `static uint8_t xCfg[]` array, and `switch(item)` blocks for the label and the display
   format. Used where the config is a block of independent 0-255 bytes owned by one `cst-*.c` module.
2. **Named-item enum + switch** (`SPEED_CONFIG_SCREEN`, `PREFS_SCREEN`, `COMM_SCREEN`,
   `SYSTEM_SCREEN`, `OPTION_SCREEN`): a local `enum { X_ITEM_… }` (or `SPEED_ITEM_*` in the module
   header) in on-screen order, resolved from `subscreenState` — `item = subscreenState - 1` for the
   fixed-layout screens (`PREFS`/`COMM`/`SYSTEM`), or a resolver where the layout is not fixed:
   `optionItemAt()` for `OPTION_SCREEN` (STACK inserts N band-editor items, and it also yields the
   band number), `speedItemAt()` for `SPEED_CONFIG_SCREEN` (the item set after the five agnostic ones
   depends on `TYPE`, and `ACCELADJ`/`DECELADJ` are spliced into the agnostic run after `ACCEL`/`DECEL`).
   `switch(item)` blocks handle label, display, and per-kind edit behaviour, with
   small `xItemIsBit()` / `xItemIsAdvGated()` / `optionBitFor()` helpers. Used where
   the values are heterogeneous — bits of `configBits`/`optionBits`/`systemBits`, a 3-way field
   (`GET`/`SET_BRK_TYPE`), deterministic toggles (STEPS, HORNTYPE), the STACK band→combo cycle,
   `new*` staging locals whose on-screen format differs from storage, or values reached only through
   `cst-*.c` accessors (`getMaxDeadReckoningTime()`, `setBatteryLevels()`).

The stock ISE `if (N == subscreenState)` chain (magic-number branches setting a scratch pointer and
a `bitPosition` sentinel byte, with `0xFB..0xFE` sub-sentinels in `OPTION_SCREEN`) is fully retired —
`PREFS`/`COMM`/`SYSTEM`/`OPTION` were converted one screen per commit. A conversion is
behaviour-preserving and has no EEPROM-layout or PC-tooling impact, since only the UI code moves.

**Committed, not live, config state.** Most fields in these screens are edited straight into their real
RAM global (`optionBits`, `brakePulseWidth`, `speedCfg[]`, the STACK combo arrays, ...) for on-screen
display, with no separate staging — safe by construction, since nothing outside the screen itself reads
that global before a `SELECT`-save, and an abandoned edit is a long-press-Menu cancel away (see
"Long-press Menu to cancel a subscreen edit" above). A handful of fields break that assumption, because
the main loop reads them every pass *regardless of `screenState`*: whether LOAD is eligible to assert at
all (`CONFIGBITS_MAIN_SCREEN_SPEED` + `speedTypeHasLoad()`), which DCC function number it actually
asserts (`OPLOADFN`/`PRLOADFN` in `SPEED_CONFIG_SCREEN` — see "LOAD button function"), and the
brake-mode fields in `OPTION_SCREEN` (`BRK TYPE`, `STEPS` and the STACK band-combo table, `VAR BRK`,
`BRK ESTP`, `BRK RATE` — see "Brake logic"). Any of these taking live effect would change real,
transmitted brake or DCC-function behavior the instant the operator merely browses a new value with
`UP`/`DOWN`, before ever confirming it — most sharply visible with `TYPE`, since `speedResetModel()`
forces `OPLOADFN`/`PRLOADFN` inert on a live, unconfirmed switch to `V4`. For these, a small cluster of
`committed*` globals in `mrbw-cst.c` (`committedLoadEligible`, `committedOploadFn`/`committedPrloadFn`,
`committedOptionBits`/`committedBrakePulseWidth`/`committedStackBandCombos3Step`/`5Step`, plus committed
STACK accessors paired with the live ones used for on-screen editing) mirrors whatever was last actually
saved, refreshed only inside `readConfig()` — the same choke point a `SELECT`-save and a long-press-Menu
cancel both already route through. Every runtime-affecting read site consults the committed copy instead
of the live one — `loadEligible()`, LOAD own `functionMask` assembly, the brake-mode dispatch,
`evaluateStackBrake()`, the `TIMER0_COMPA_vect` pulse-width wrap, AIRBRAKE own `independentBrakeAtRest`
classification, and SPEED own `stepBrakeMode` exclusion — while the screen own display/edit code keeps
reading the live global, so on-screen browsing stays fully reactive with no visible change in behavior;
only the real, transmitted effect is deferred to save time. `HORNTYPE`/`REV SWAP` (`OPTION_SCREEN`),
`AIRBRAKE_CONFIG_SCREEN` own fields, and `CONFIG_FUNC_SCREEN` own function assignments have the same
live-edit-live-effect characteristic and are not (yet) covered by this pattern.

## Shared network CNF store

Lets one loco CNF (configuration profile: DCC function assignments, brake/STACK settings, notch table,
speed/momentum CVs — the existing 128-byte slot format) be shared wirelessly across multiple ProtoThrottle
units, instead of requiring physical ISP access to hand-copy JSON between them. **`mrbw-cabbus`** (the
sibling NCE Cab Bus gateway repo) is the authoritative store, since it is already layout infrastructure
that is powered whenever the layout is. Holds 20 independent network slots (`N01`-`N20`).

No usable time/versioning concept exists anywhere in this firmware family (the fast clock has no date
fields and is never persisted), so sync uses simple **last-write-wins with no version/generation checking
enforced** — two throttles saving the same network slot in the same short window will silently have the
second write win, with no warning to either user. An accepted tradeoff for a low-frequency,
likely-single-active-editor use case; a generation-counter byte is reserved in the storage layout for a
possible future optimistic-concurrency conflict *detection* pass.

**Wire protocol**: `'C'`(push, write)/`'D'`(pull, read), one transfer in flight at a time, `BEGIN`→`DATA`
(×N, 10 payload bytes/chunk)→`COMMIT`(push)/`DONE`(pull) — 13 chunks for the full 128-byte CNF. Push stages
into a scratch buffer on the receiver and only commits on a matching whole-payload CRC16. Pull snapshots
the live entry into its own read-scratch buffer at `BEGIN` time (not read live per chunk), so a concurrent
push from a different throttle cannot hand a puller a torn mix of pre-/post-push bytes; the throttle
verifies CRC16 locally once all chunks arrive. Busy-locking is table-wide (one transfer across the whole
table at a time); a `BEGIN` from a different source while busy is refused; no activity for 3s unilaterally
clears busy state so a throttle that loses power/range mid-transfer cannot wedge the store. Per-chunk ack
timeout is 300ms/3 retries; COMMIT gets its own larger 800ms budget, since it triggers a real ~132-byte
EEPROM write (~3.3ms/byte) before it can even queue its reply. These are bench-tuning starting points, not
precision-measured values.

**Throttle-side UI**: the slot picker of `LOAD_CONFIG_SCREEN`/`SAVE_CONFIG_SCREEN` gained `N01`-`N20` entries
immediately below local slot 1, using a disjoint internal range decoupled from on-screen order — a
deliberately distinct zone so there is always a visible cue that a LOAD/SAVE here does a network
round-trip. Needs zero new EEPROM fields on the throttle side — addressing reuses the existing "base
station" field as the sync target. Each `N` entry shows a loco-ID preview (`"N01:1823"`-style) before
committing to LOAD/SAVE, via a lightweight peek query rather than a full pull; the query is gated behind a
settle delay on UP/DOWN movement so rapidly stepping through the range does not spam the radio or freeze
the throttle at every slot crossed. While the peek is in flight the `Nnn:` line runs an animated dot field
(`.`→`..`→`...`→`....`), so a slow or unreachable receiver reads as "working" rather than a frozen screen.
`cst-sync.c`/`.h` (`syncPushSharedCnf()`/`syncPullSharedCnf()`/
`syncQuerySharedLocoAddress()`) is the throttle-side client, following the existing modular `cst-*.c`
convention.

**Storage (mrbw-cabbus side)**: `cnf-store.c`/`.h` own a dedicated EEPROM table (`cabbus-eeprom.h`) — 20
entries × 196 bytes (4-byte header + 192-byte payload reservation). Each entry: a key byte (occupied/
unused), a generation counter (reserved, unenforced), a stored CRC16, then the opaque payload — mrbw-cabbus
never parses the payload itself. The payload reservation (192 bytes) deliberately exceeds the current
128-byte slot format, so an ordinary future field addition can reach this store without the table needing
to be resized/reflashed.

**CNF format version guard**: a table-wide pin (ahead of every entry, never moved) records which
throttle-side `EEPROM_LAYOUT_VERSION` every currently-stored entry payload was written under, and its
real length. A push `BEGIN` carries the `EEPROM_LAYOUT_VERSION` of the pushing throttle; compared against
the pin: unset or strictly higher → accepted, and on COMMIT wipes every *other* entry and advances the pin
(there is no per-entry version tracking, so this is the only safe choice once a newer format is accepted);
equal → ordinary accept; strictly lower → refused immediately, before any `DATA` traffic. This means an
ordinary future field addition needs only an `EEPROM_LAYOUT_VERSION` bump on the throttle side — the next
push from updated firmware auto-advances the pin, no receiver reflash required. A receiver reflash is only
forced by a change to the wire protocol framing itself, or the payload outgrowing 192 bytes. A separate,
structural `CNF_TABLE_LAYOUT_VERSION` (the table own byte shape) is distinct from this payload-version
pin; a mismatch there means the firmware cannot trust the table shape at all, so it wipes everything
before self-healing.

On the throttle, `SAVE CNF` to a network entry first peeks the pinned version (`syncPeekSharedVersion()`);
that peek is a hard gate for the destructive path. If it fails (timeout, busy, protocol error) the SAVE is
refused with a `CHECK` / `RETRY` screen — the firmware will not push blind, since it cannot then tell
whether the push would advance the pin. If the peek succeeds and the pin is unset or older than the
`EEPROM_LAYOUT_VERSION` of this throttle, a two-stage "UPGRADE" / "WIPE N01-20?" confirmation runs before the real push, so
the operator is warned before every other stored network slot gets wiped. A dedicated `SUBTYPE_RESET`
command (`reset-cabbus` in `cst_cfgnetwork.py`) does an immediate whole-table wipe + pin clear, independent
of any version comparison, for a deliberate human-triggered reset.

**Only real throttle firmware may advance the pin.** `cst_cfgnetwork.py import` refuses outright — before
touching any entry — whenever the peeked pin is unset or strictly lower than its own
`SUPPORTED_LAYOUT_VERSION`, rather than offering to push and advance it itself. The reason: the PC tool
`SUPPORTED_LAYOUT_VERSION` tracks the same source tree the firmware does, not what is actually flashed to
any physical throttle in the field — it can drift ahead of every deployed throttle from nothing more than a
source update, no reflash required. If the tool could advance the pin on that basis, a routine tool update
run before any throttle had been upgraded could push the whole network store to a version nothing deployed
understands, disabling `N01`-`N20` network-wide until every throttle was individually reflashed. This is a
policy enforced by the tool, not something the wire protocol itself can verify — the low-level
`push_entry()` in `cnf_radio_io.py` still accepts an arbitrary version number, which remains useful for testing the
on-device upgrade flow without a special firmware build.

**Untested**: the two-throttle case (a second unit pulling a CNF the first pushed) and the failure-path
case (receiver unreachable/powered off mid-sync) — both blocked on multi-unit hardware availability.

## Removed: ACCEPT DOWNLOAD / wireless EEPROM write

Stock ISE firmware answers an inbound MRBee `'W'` (EEPROM Extended Write) packet by writing the requested
bytes straight into EEPROM, gated only by an `enableEepromWrite` flag that the on-device `COMM CFG` →
`ACCEPT DOWNLOAD` subscreen sets while it is on screen (ISE added that interlock in 2021; before it, `'W'`
writes were unconditional). The menu item is in the ISE menu-map but undocumented in the user manual, and no
ISE or fork tool drives it — `cst_cfgtransfer.py` (ISP) and this shared network CNF store (`'C'`/`'D'`)
together cover offline and wireless config transfer with real integrity checking.

This fork removes the `ACCEPT DOWNLOAD` subscreen and the entire `'W'` packet handler. A `'W'` packet is
now unmatched in `PktHandler()` and silently dropped — no `'w'` reply, no write. Wireless EEPROM **reads**
(`'R'`, ungated, used by generic MRBus tooling) and every other stock wire behaviour are unchanged. The
`'C'`/`'D'` CNF path is independent of `'W'` in every respect: different packet type, its own direct
`eeprom_write_block()` after a whole-payload CRC16 check, never routed through `PktHandler()` on the
throttle.

## PC tooling

Two Python 3, stdlib-only tools manipulate stored loco configurations from a PC rather than the on-device
menu. Both share `cst_eeprom_layout.py` (offset/enum constants) and `slot_codec.py` (pure decode/encode/
validate, no hardware dependency) — a **hand-maintained mirror** of `src/cst-eeprom.h` and the decode
logic in `readConfig()` inside `mrbw-cst.c`, not generated from them, since the C headers only give byte
offsets, not the bitfield/enum/multi-byte-array semantics that live in the firmware control flow. The
`-h`/`--help` output of both tools — including per-subcommand help, e.g. `cst_cfgtransfer.py import -h` —
documents every flag in more detail than covered below; check there for the exact current option set. A
third tool, `cst_fastclock.py`, also lives in this section but is unrelated to loco configuration — it
broadcasts a fast-clock time signal instead, with no EEPROM or config-slot involvement at all (see below).

### `cst_cfgtransfer.py` — ISP-based export/import

`src/cst-cfgtransfer/cst_cfgtransfer.py` exports the throttle stored loco configurations to
hand-editable JSON files and imports them back, entirely over the ISP programmer (`avrdude`/`iseavrprog`) —
offline EEPROM manipulation with the chip in the programmer, not a wireless or runtime interface.

```bash
python3 cst_cfgtransfer.py export --out-dir ~/protothrottle-backups/          # all 20 slots + active + device
python3 cst_cfgtransfer.py import --slot 5 --dry-run edited-slot05.json       # preview, no hardware write
python3 cst_cfgtransfer.py import --slot 5 edited-slot05.json                 # write it
python3 cst_cfgtransfer.py import --dir ~/protothrottle-backups/throttle-42/ --yes  # restore a whole folder
```

`export --out-dir <dir>` creates/reuses `<dir>/throttle-<mrbus-addr>/` (the MRBus device address is the
throttle identifier, since the ATmega1284P has no factory-unique ID readable over ISP); the working
profile exports as `slot00_active_addr*.json` (named `slot00` to group with the numbered slots, still
targeted with `--active`). Import always reads the full current EEPROM, splices in only the
explicitly-targeted byte range(s), and writes the full image back, so anything not targeted round-trips
byte-for-byte unchanged. `avrdude_io.py` is the only module that shells out to `avrdude`;
`cst_cfgtransfer.py` is the argparse CLI tying the pieces together. The JSON mirrors the on-device
menus — **one object per config menu**, objects and keys in menu order: a slot is `loco_address` /
`force_functions` (`{on, off}` — the FORCE FUNC menu, distinct from the CONFIG FUNC `functions`) /
`functions` / `notch_speedstep` / `speed` / `airbrake` (the `AIRBRAKE CFG` menu — `DISPLAY`/`COMP_MODE`
are string enums, everything else a plain number) / `options` (the OPTIONS menu — brake config plus
`reverser_swap`/`horn_type`; its meta-field is `unset`); `device.json` is `system` (ADV-FUNC battery
thresholds) / `comm` / `prefs` (`config_bits` nested here) / `calibration`. The `speed` object is
decoder-family-shaped: the five agnostic fields (`TYPE`/`MAXSPEED`/`UNIT`/`ACCEL`/`DECEL`) plus
only the model fields the `TYPE` uses (`speed_fields_for_type()` in `cst_eeprom_layout.py` — a `V4`
slot has 13 keys, a `V5` slot 21); `ACCELADJ`/`DECELADJ` are keyed right after `ACCEL`/`DECEL`
(matching the on-device splice), then `BRK1` leads the rest of the model list for every family. The
encoder writes inert values to the slots a `V4` drops so the image byte-matches the firmware.
`ACCEL`/`DECEL` and `ACCELADJ`/`DECELADJ` are read raw by the firmware (a stored `0xFF` is a real value — 255, or −127
sign-magnitude), so the codec decodes `0xFF` to that rather than `"UNSET"`, accepts `ACCEL`/`DECEL`
`0-255` and `ACCELADJ`/`DECELADJ` `-127..127`, and maps a bare `"UNSET"` for one of these to its
default value; `SPEED_FULL_RANGE_FIELDS` / `SPEED_SIGNED_FIELDS` in `cst_eeprom_layout.py` name them.
`SLOT_SCHEMA_VERSION` is 6 (`functions` gained `MENU_BUTTON` / `SEL_BUTTON`, `prefs.config_bits`
gained `ops_mode` — see "OPS MODE screen"). `encode_slot` / `encode_global` also accept the older
pre-schema shapes on import (flat device fields, `force_function_on`/`off`, `brake` / `options_unset`,
and — via `--import-old` — a pre-4 flat 19-field `speed` object whose `TYPE` is `V4`, a pre-5 `speed`
object missing `ACCELADJ`/`DECELADJ`, or a pre-6 backup missing the `MENU_BUTTON` / `SEL_BUTTON`
function keys). `MenuOrderTests` in `test_slot_codec.py` locks the ordering so a `slot_codec.py`
change is deliberate.

**CNF format version guard**: since the codec is a hand-maintained mirror, a stale copy of this tool run
against a newer/older device could silently misdecode. `EEPROM_LAYOUT_VERSION` on the chip is compared
against the tool supported version (`cst_eeprom_layout.SUPPORTED_LAYOUT_VERSION`, itself parsed from
`cst-eeprom.h` at import so it always tracks the firmware source tree) before either `export` or `import`
decodes/encodes anything, hard-refusing on any mismatch. A device running firmware from before this field
existed reads it as `0xFF` until it boots once with newer firmware (`readConfig()` self-heals it on every
boot).

**`--import-old`**: restores a backup exported under an older schema (e.g. taken just before an
`EEPROM_LAYOUT_VERSION` bump that added a field) by defaulting any field absent from the file instead of
rejecting it outright — functions default to `RAW:0xFF` (the same byte a never-written function already
decodes to), speed fields default to `"UNSET"` (letting `readByteOrDefault()` self-heal on next boot). A
field that is *present* but invalid is still always a hard error regardless of this flag, and a whole
missing top-level category (no `"functions"` object at all) is still always a hard error too.

**`dump` / `wipe`**: `dump` is a raw EEPROM readback (hex summary, or `--out FILE` for the full 4096-byte
image), with no decode and — unlike `export`/`import` — no version gate, so it works against a chip whose
format this tool would otherwise refuse. `wipe` factory-blanks the EEPROM (every byte `0xFF`) and is also
un-gated (a wipe is *how* you recover from a version mismatch). It deliberately avoids the `eeprom:w` path
— a full all-`0xFF` image write hits the exact reliability quirk below — and instead clears the HFUSE
`EESAVE` bit (`0xD1`→`0xD9`), does a chip erase (which then wipes EEPROM as a side effect), and restores
`EESAVE`. The restore runs on every exit path and is verified; `blank_eeprom_via_fuse_toggle()` raises
`EepromWipeError` with the manual-recovery command if it ever fails to put `EESAVE` back, since a cleared
`EESAVE` would make an ordinary `make flash` wipe the throttle config too. `wipe` erases flash as well —
the throttle needs re-flashing afterward.

**EEPROM write reliability**: `import` retries the whole write up to 3 times on failure — see the module
docstring of `avrdude_io.py` for the full story, including a confirmed driver-level mechanism (the EEPROM
write path of `iseavrprog`/`usbtiny` has a much narrower timing margin than flash, with no retry on
a dropped USB transfer) and a real-hardware finding that a marginal ISP USB cable was a major contributor
too — a cable swap took one machine from 3 retries in 4 writes down to 0 in 8. The mitigations here stay in
place regardless, since this tool has no way to know the cable/port/programmer quality of another user in
advance. A printed `attempt N/3 failed, retrying...` during a write is expected, not a sign of broken
hardware; only a failure across all 3 attempts is worth investigating. If every attempt fails, `import`
reads the chip back and reports exactly which targeted item(s), if any, ended up inconsistent, rather than
leaving the state ambiguous. A run of failed writes can also make the *next* `avrdude` call hang
indefinitely (confirmed on real hardware); every `avrdude` call therefore has a 90-second hard timeout, so
this always surfaces as a clean, immediate error instead of an unbounded hang. This is not a lasting
hardware fault — no physical power-cycle is needed to recover, just retry the operation (confirmed on real
hardware: a flash write and a plain EEPROM read both succeeded immediately right after a timeout, with
nothing unplugged in between).

### `cst_cfgnetwork.py` — wireless PC access to the shared network CNF store

`src/cst-cfgnetwork/cst_cfgnetwork.py` lets a PC export/import loco configurations to/from the shared
network CNF store of `mrbw-cabbus` (`N01`-`N20`, see "Shared network CNF store" above) directly over a
USB-attached XBee radio — no physical throttle and no ISP access to any device needed. It imports
`slot_codec.py`/`cst_eeprom_layout.py` directly from `cst-cfgtransfer/` rather than duplicating the codec,
since the 128-byte network-entry payload is byte-identical to a local slot.

`src/cst-cfgnetwork/cnf_radio_io.py` is a from-scratch, Python-3-only implementation of XBee API-frame
escaping/framing, MRBus CRC16, and the full `'C'`/`'D'` `BEGIN`/`DATA`/`COMMIT`/`DONE` client state machine
(`pull_entry()`/`push_entry()`/`peek_loco_address()`), plus two helpers that are not part of the `'C'`/`'D'`
protocol: `discover_nodes()` (an MRBus presence-ping sweep) behind the `discover` subcommand, and
`sniff()` + `decode_cst_status()` (a passive, read-only radio listener that decodes the throttle status
`'S'` packet — loco/direction/speed step/function mask/status flags/battery, and with `--cnf` the
shared-CNF `'C'`/`'D'` transfer packets too, via `format_cnf_packet()`) behind the `sniff` subcommand.
Requires a spare XBee3 module joined to the same PAN as the throttle/receiver radios, such as the
`ckt-xbee` USB-to-XBee adapter from ISE itself.

```bash
python3 cst_cfgnetwork.py discover --port /dev/cu.usbserial-XXXX --my-addr 0x3F
python3 cst_cfgnetwork.py sniff --my-addr 0x3F --changes
python3 cst_cfgnetwork.py list --cabbus-addr 0xD0
python3 cst_cfgnetwork.py export --entry 1 --out-dir ~/protothrottle-backups/
python3 cst_cfgnetwork.py import --entry 20 --dry-run edited.json
python3 cst_cfgnetwork.py reset-cabbus --out-dir ~/protothrottle-backups/pre-reset/
```

`discover` merges two mechanisms: a broadcast ping (finds throttles, which honor broadcast addressing) plus
a unicast sweep of the whole base-station address range `0xD0`-`0xEF` (finds receivers, which — like real
MRBus/MRBee nodes generally — silently drop broadcast packets; only 32 addresses, so a full sweep is cheap).
The sweep never stops at the first hit, since a layout may have more than one receiver. `sniff` opens no
transfer at all — it just prints CRC-valid packets as they arrive, decoding the throttle `'S'` status
packet; `--changes` collapses the ~1 Hz status stream to one line per actual state change (the view for
catching a transient glitch such as a light-knob flicker), `--status-only` drops non-`'S'` traffic,
`--cnf` decodes the `'C'`/`'D'` push/pull packets (`BEGIN`/`DATA`/`COMMIT`/`DONE`, entry, offset,
status) instead of the raw-hex fallback — the view for watching a `SAVE`/`LOAD CNF` transfer. `list`
gives a fast all-20-entries loco-address overview via `peek_loco_address()` with no full pulls. The
default integrity check for `import` is the same COMMIT CRC gate the push itself already uses on the
receiver side; `--verify` adds an opt-in pull-and-diff. `export --dir`-style batch imports auto-skip
`NONE`/never-configured stub entries (an un-decodable-back all-`0xFF` payload) rather than aborting the
whole batch on the first one, but a single explicitly-named file pointed straight at a stub still
errors, since silently doing nothing for the one thing explicitly requested would be worse than the
error.

If a queried receiver is reachable but running firmware without the shared CNF store (stock ISE firmware,
or any build predating `cnf-store.c`), every real CNF command times out on its first request — from the
radio side this looks identical to a dead link or wrong address. `probe_address()` (the same unicast-ping
primitive the `discover` sweep uses) is fired once, supplementarily, whenever the failing step of a timeout
is `BEGIN` or `reset`, to distinguish "answers ping but not `BEGIN`" (reachable, incompatible firmware) from
"answers nothing at all" (not reachable — wrong address, dead radio link, or powered off), reported as two
distinct clear messages rather than one generic timeout.

**CNF format version guard**: `export`/`list`/`import` each peek the shared network table version pin
before touching any entry — `export` aborts early on a mismatch, `list` tags each row individually, and
`import` refuses outright if the pin is unset or behind the tool own `SUPPORTED_LAYOUT_VERSION`. Unlike
`cst_cfgtransfer.py`, this tool is never allowed to *advance* the pin itself — only real throttle firmware
may (see "Shared network CNF store" above for the full policy and why).

**`--import-old`**: same flag and `slot_codec.py` defaulting behavior as the `--import-old` flag of
`cst_cfgtransfer.py` above — restores a backup exported under an older schema by defaulting any field
absent from the file rather than rejecting it.

**`reset-cabbus`**: wipes the whole shared network CNF table and clears the version pin via the wire
protocol `SUBTYPE_RESET` (see "Shared network CNF store" above) — a deliberate, human-triggered reset,
not something any version mismatch triggers automatically. Backs up all 20 entries by default (`--out-dir`
required unless `--skip-backup`), reusing the same output-folder convention `export` already uses.

### `cst_fastclock.py` — XBee-broadcast software fast clock

`src/cst-fastclock/cst_fastclock.py` broadcasts an MRBus/MRBee fast-clock `'T'` packet from a PC over a
USB-attached XBee radio, standing in for the mrb-fcm fast-clock-master hardware from ISE during bench
testing. Unlike the two tools above, it has no EEPROM/config-slot dependency at all — it only reuses the
radio transport (`cnf_radio_io.py`, serial port setup, XBee API framing, MRBus CRC16) from
`cst-cfgnetwork/`, via a small public `send_raw()` wrapper added around the existing internal
`_send_mrbus()` primitive of that module.

```bash
python3 cst_fastclock.py --port /dev/cu.usbserial-XXXX --my-addr 0x50 --start 08:00 --ratio 4
```

The wire format is copied unchanged from the mrb-fcm.c of ISE (an 18-byte packet: 6-byte MRBus header plus
12 payload bytes — real hours/minutes/seconds, a flags byte, fast hours/minutes/seconds, a big-endian
`timeScaleFactor` at ratio × 10, and three date bytes this throttle never reads) and is consumed unchanged
by the existing `processTimePacket()` in `src/cst-time.c` — no firmware change was needed to support this
tool. `--start`/`--ratio` set the fast time as of the moment the tool starts and the fast:real speed
ratio; the tool re-broadcasts every `--interval` seconds (default 2), comfortably inside the
dead-reckoning timeout the throttle already implements between packets, so no ack/retry logic is needed,
matching the fire-and-forget behavior of mrb-fcm itself. `--ampm` sets the `DISP_FAST_AMPM` flag bit for a
12-hour on-screen display (24-hour is the default; the on-screen indicator is a small custom pictographic
glyph, not literal text — see `displayTime()`/`ClockAM`/`ClockPM` in `cst-time.c`); `--hold` sets
`DISP_FAST_HOLD` alongside `DISP_FAST` to broadcast the clock paused, exercising the `" HOLD "` display
state of `printTime()`.

Requires the on-device `COMM CFG` → `TIME ADR` set to `0xFF` (accept a `'T'` packet from any MRBus source
address — the factory default of `0x00` instead restricts acceptance to whatever `BASE ADR` the throttle
is itself configured to) and `PREFS` → `DISPLAY` set to the clock option, not `SPEED` (see the SPEED
section above). Hardware-confirmed on a real throttle, including with `OPS MODE` active — the fast clock
is drawn by the same `renderBaseScreen()` path shared by `MAIN_SCREEN` and `OPS_MODE_SCREEN`, so it is
unaffected by the OPS MODE button rebinding.

### Maintenance checklist — follow whenever the EEPROM layout changes

New field, moved offset, or repurposed byte in `cst-eeprom.h`:

1. Add/change the field in `src/cst-eeprom.h` and wire up `readConfig()`/save-path code in `mrbw-cst.c`.
   If old EEPROMs need forward-migrating, add the migration block to `applyEepromMigrations()` in
   `src/cst-eeprom.c` (not `readConfig()`), plus a `sc_from_layoutN()` scenario and any invariant in
   `src/cst-eeprom-test/test_eeprom.c`. If it is a per-profile SPEED/AIRBRAKE/STACK "model" field, add
   its offset to `eepromResetProfileModel()` in `src/cst-eeprom.c` **and** to `resetModel_check[]` in
   `test_eeprom.c` (`make eepromtest` fails if the two disagree, or if a freed hole was reused without
   updating the range partition).
2. Mirror the same offset/type/decode logic in `cst_eeprom_layout.py` and the
   `decode_slot()`/`decode_global()`/`encode_slot()`/`encode_global()` functions of `slot_codec.py`.
3. Bump `EEPROM_LAYOUT_VERSION` in `cst-eeprom.h` — the Python tooling parses that `#define` at import
   (`cst_eeprom_layout._read_firmware_layout_version()`), so there is no second copy to keep in step.
4. Add/update the corresponding fixture in `src/cst-cfgtransfer/tests/test_slot_codec.py` (run via
   `python3 -m unittest discover tests` from `src/cst-cfgtransfer/`, no hardware needed).
5. Update the field reference in `src/cst-cfgtransfer/README.md` if the field introduces new JSON
   vocabulary, and bump `SLOT_SCHEMA_VERSION` in `slot_codec.py` if the exported JSON shape changes
   (this is the JSON schema version, independent of `EEPROM_LAYOUT_VERSION` — a change can move one
   without the other: the `V5MULT`/`V4` split bumped the schema with no layout change; the SPEED
   payload relocation bumped the layout with no schema change; the `ACCELADJ`/`DECELADJ` +
   genuine-0-255 `ACCEL`/`DECEL` work and the OPS MODE `MENU BTN` / `SEL BTN` addition each bumped
   both).
6. If the change touches `cst-speed.c`, `cst-pressure.c` or `cst-eeprom.c`, regenerate the reference
   traces with `make speedtest-accept` / `make pressuretest-accept` / `make eepromtest-accept` and
   review the `git diff` — that diff is the human-readable statement of how the model output (or the
   migration result) moved.

One local git hook (`.githooks/pre-commit`, wired up by `make setup`) guards against this checklist being
followed incompletely: `check_layout_change_bumps_version.py` catches a layout change that never bumped
`EEPROM_LAYOUT_VERSION` at all, by diffing the **layout-defining** `#define`s of `cst-eeprom.h` — the
`EE_*` byte offsets, the `CONFIG_*` addressing macro/constants, and `MAX_CONFIGS` / `WORKING_CONFIG` —
against their previous committed state. Other `#define`s in the header (the `STACK_COMBO_*` storage
encoding, `*_DEFAULT` values, helper constants) do not trigger it. It also cannot catch a change that
reinterprets what an existing, unmoved byte value *means* (a new enum numbering, repurposed bits) without
changing its offset, nor a layout-relevant constant added under a name matching none of those patterns —
those are only caught by careful review, or by the targeted per-field regression tests in
`test_slot_codec.py`.

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

## Compatibility: mixing stock ISE firmware with this fork on the same layout

A real deployment will often have a mix of throttles/receivers on different firmware vintages during a
rollout. This matters most for the shared network CNF store (`'C'`/`'D'` packets), since that is the one
feature added to the wire protocol of *both* repos. Everything else (DCC status relay, fast clock, EEPROM
read, ping, version query) is unchanged from stock and was never a compatibility question; the one stock
behaviour this fork *drops* is the `'W'` wireless-EEPROM-write handler (see "Removed: ACCEPT DOWNLOAD"
above) — a fork throttle silently ignores a `'W'` packet instead of writing, never a hazard to anything.

**Stock throttle + the receiver of this fork: fully compatible, no caveats.** `PktHandler()` in
`mrbw-cabbus.c` calls `cnfStoreHandlePacket()` first on every packet, but the first line of that
function is `if(CNF_PKT_TYPE_PUSH != type && CNF_PKT_TYPE_PULL != type) return 0;` — i.e. `'C'`/`'D'`
only. A stock throttle never transmits `'C'`/`'D'`, so this is a no-op on every packet it sends;
dispatch falls through to the unmodified chain exactly as before the CNF store existed.

**The throttle of this fork + stock receiver: fully compatible for everything except network slots, which fail
cleanly.** All normal operation (DCC relay, status, clock, local slots 1-20) is untouched. Attempting `SAVE
CNF`/`LOAD CNF` against an `N01`-`N20` entry sends a `'C'`/`'D'` `BEGIN` that a stock receiver silently
drops (no reply, no NACK); `cst-sync.c` retries 3× at 300ms and returns a timeout after ~900ms, shown
on-device as a clear, bounded `TIMEOUT` / `NO REPLY` screen — not a hang, not corruption, not a silent
no-op.

**`cst_cfgtransfer.py` + stock receiver: not applicable — no interaction at all.** This tool is ISP-only;
it never talks to a receiver over the radio, so receiver firmware version is irrelevant to it.

**`cst_cfgnetwork.py` + stock receiver: reachable, but every CNF command fails — diagnosed clearly rather
than as a generic timeout.** See "PC tooling" above.

**Net takeaway**: the shared network CNF store was built to degrade gracefully in both directions — a
version mismatch or a firmware-generation mismatch is always a clean, bounded failure on whichever side
lacks the feature, never a corruption, hang, or silent misinterpretation of an unrelated packet. Nothing
about normal DCC operation is ever put at risk by a partial firmware rollout.

## Design decisions not carried into the codebase

**Per-loco speed broadcast (reverted)**: an earlier design had `mrbw-cabbus` broadcast each tracked loco
commanded speed onto MRBus as a new packet type, received here and displayed on the main screen. Built and
confirmed working on real hardware, then reverted the same day — not due to a defect. This throttle already
has zero-latency access to its own commanded speed (the same values used to build its own outgoing status
packet), which is the same input any speed/momentum model needs, for the one use case that mattered:
showing the speed of the loco it is currently driving. Computing it locally (see the `SPEED` section above)
is strictly better than a wireless round trip through the gateway — lower latency, no dependency on that
gateway being present or in range.

**mrbw-wifi as an alternate shared-network-CNF-store host (evaluated, not implemented)**: `mrbw-wifi` (an
ESP32-S2 MRBus↔WiFi bridge from ISE) was evaluated from source as a second possible host for the shared network CNF
store, alongside `mrbw-cabbus`. Architecturally sound and arguably a better long-term host (no throttle-side
change needed at all, and ample flash/RAM headroom), but a genuine from-scratch port rather than a
code-reuse job, with the storage-placement question (a dedicated flash partition vs. the existing FAT
partition already used for `config.txt`, which is also exposed raw over USB and so has a real dual-writer
hazard) left open. Not implemented — blocked on hardware availability to test against.
