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

## Two-stage horn ("Horn2")

A second, independently-configurable DCC function (`HORN2_FN`, menu name "HORN2") tied to its own calibrated
lever position (`hornThreshold2`), with a `HORNTYPE` option (`OPTION_SCREEN`) selecting how it relates to the
primary horn: Additive, default (Horn2 fires on top of Horn1 once the lever passes Horn2's threshold) or
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
`<H1` cue whenever the captured `hornThreshold2` sits at or below `hornThreshold` (Horn1's point) — a
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

## SPEED — scale-speed simulation

Replaces the fast clock on the main screen with a locally-computed **scale miles-per-hour (or km/h)**
readout that models ESU LokSound/LokPilot (both V4 and V5, selected via the `TYPE` field) momentum and
three-brake deceleration behavior, so the displayed speed stays in visual sync with what the decoder is
actually doing rather than just the instantaneous commanded speed step. Local computation was chosen over
an earlier wireless-received-speed design that was evaluated and not adopted.

Commanded speed is `notchSpeedStep[activeThrottleSetting-1]` — the same 0-126 DCC speed step already
transmitted, no new input needed. Scale speed is `speedStep/126 × speedMaxMph`, with `speedMaxMph` a
per-profile configurable value (`MAXSPEED`), converted to km/h at display time if that unit is selected.
`src/cst-speed.c`/`.h` (mirroring the shape of `cst-pressure.c`: EEPROM-backed per-profile config, 10Hz update,
own display printer) holds `simSpeedStepQ8`, an 8.8 fixed-point simulated speed step — chosen over a plain
integer to avoid per-tick truncation stalling at slow rates, the same class of problem
`updatePressure10Hz()` works around with its own floor. `updateSpeed10Hz()` runs once per 100ms tick, but
from the **main loop**, not the timer ISR: `TIMER0_COMPA_vect` only sets a `speed10HzTick` flag and the
main loop consumes it. The standing-start ramp does `int64` multiply/divide that must not run inside an ISR
— it would stall the radio-UART and quadrature-encoder interrupts for its duration (a latency concern, not
a CPU-load one — the math is a ~0.25% duty cycle). `updateTime10Hz()` and `updatePressure10Hz()` stay in
the ISR: cheap, and they need precise timing. Brake-active flags and the other inputs are passed to
`updateSpeed10Hz()` explicitly rather than read via `extern`, so the module has no dependency on
the internal bit layout of `mrbw-cst.c`.

**ESU LokSound/LokPilot formulas modeled** (from ESU community documentation — covers both V4 and V5, see
`TYPE` below):
- Accel/decel: time to cross the full speed range = `CV × multiplier` seconds (CV3 for accel, CV4 for
  decel). The multiplier is decoder-family-dependent — see `TYPE` below.
- Brake override: `stopSeconds = (255-CVbrakeSum)/255 × (CV4 × multiplier)`.
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

`SPEED_CONFIG_SCREEN` (landing page `SPEED CFG`), a 19-item cycle grouped: momentum CVs (`ACCEL`/`DECEL`/
`BRK1`/`BRK2`/`BRK3`/`DELAY`) → display calibration (`MAXSPEED`/`UNIT`) → watched-function triggers
(`HOLDFN`/`STOPFN`) → load simulation (`OPLOAD`/`OPLOADFN`/`PRLOAD`/`PRLOADFN`) → decoder family (`TYPE`) →
correction tunables (`ACCPCT`/`ACCTGT`/`DECPCT`/`DECTHR`). The last four are fine-grained calibration
values hidden from the item cycle unless `ADV FUNC` (`SYSTEM` screen) is enabled, to avoid accidental
edits; they are still always read via `readByteOrDefault()` with real defaults regardless of visibility, so
hiding them never risks an unset field. `UNIT`/`TYPE` are strict two-way toggles; `HOLDFN`/`STOPFN`/
`OPLOADFN`/`PRLOADFN` show `OFF` or `F##`.

`printSpeed()` converts the configured `MAXSPEED` into km/h once before computing the displayed value
(rather than converting an already-rounded mph figure) to avoid compounding rounding error, and decides its
field width (2-digit-padded vs. 3-digit-unpadded) from that *configured* max rather than the live
instantaneous value, so the layout never jumps mid-session.

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
asserted, the loco-address line of the main screen is also replaced with a literal `"HOLD"` on-screen cue — the
same priority slot the throttle existing reverser-mismatch indicator already uses, checked in order:
alerter-timeout backlight blink → reverser mismatch → `"HOLD"` → normal loco address/speed display.
`STOPFN` has no on-screen text of its own — its effect is only visible through the speed readout itself
snapping to 0mph.

`THROTTLE_STATUS_EMERGENCY` is a live combinational OR of three sources, recomputed every main-loop pass:
brake lever at max (gated by `OPTIONBITS_ESTOP_ON_BRAKE`), a control configured to `FN_EMRG`, and alerter
timeout specifically when `ALERTER_FN = FN_EMRG` — an ordinary alerter timeout always forces the throttle
to zero and applies the brake as a fail-safe regardless of how `ALERTER_FN` is configured, but only feeds
`THROTTLE_STATUS_EMERGENCY` (and so the display snap-to-zero) in the `FN_EMRG` case. That forced brake
application asserts the `BRAKE_FN` bit directly in `functionMask` (the alerter-timeout block in
`mrbw-cst.c`), so the `functionMask`-based Brake1 detection the speed sim already does picks it up
through the same path as every other brake source.

### How the simulation math works

**Decoder family (`TYPE`)**: selects which momentum-CV time convention `ACCEL`/`DECEL`/`BRK1-3` use —
`V5DCC` (0.896s per CV unit, ESU LokSound 5 DCC-only, the NMRA S9.2.2-standard multiplier) or `V4V5MULT`
(0.25s per CV unit, the general case: LokPilot/LokSound V4, and V5 MultiProtocol).

**Load simulation (`OPLOAD`/`PRLOAD`/`OPLOADFN`/`PRLOADFN`)**: mirrors decoder CV103 (Optional Load)/CV104
(Primary Load) — each a 0-255 value (128 = neutral) that scales the base `ACCEL`/`DECEL` CV while its
watched DCC function is active, `time = CV × loadValue/128`. Primary Load wins if both are active
simultaneously, per the ESU manual.

**Brake-function detection**: the simulation reads whether Brake1/2/3 are active from the actual outgoing
`functionMask`, not from the lever own state-machine bits — so any control mapped to the same DCC
function number is detected regardless of source. Step brake mode is deliberately excluded (forced
inactive) — its pulses are a one-directional decoder-side ratchet the sum-based brake model cannot
represent; Step remains the one brake mode where the simulated behavior can diverge from a real
Step-braking locomotive.

**Deceleration lag correction (`DECTHR`/`DECPCT`)**: real decoders decelerate faster than the plain `DECEL`
(or summed brake) model predicts. `lag(speed) = slope × max(0, speedStep − DECTHR)`, slope proportional to
the relevant reference time. Calibrated from hardware measurement: `DECTHR≈11` (~4.2mph, roughly constant
across `DECEL`), validated for `DECEL 0-230` (values above showed non-monotonic real-decoder behavior and
are outside the validated scope — the underlying `ticksToCross()` math itself is strictly linear, so this
is a decoder characteristic, not a firmware bug). Applies to any deceleration via one unconditional path —
coast or brake, steady-state or interrupting an active climb.

**Standing-start acceleration (`ACCPCT`/`ACCTGT`)**: real locomotives reach any speed below 15mph faster
than the plain `ACCEL` model predicts, by a roughly constant amount of time (not proportional to distance).
`ACCPCT` (0-255 = 0-100% of the full-range crossing time of `ACCEL` itself) is that lead-time budget, spent as a
smooth cubic-Hermite ramp from a genuine stop up to the 15mph-equivalent speed, calibrated from a 30-point
standing-start hardware sweep. `ACCTGT` (0.1s/unit) is a target time-to-1mph, achieved by generalizing the
the zero-rate starting boundary condition of the ramp to a configurable nonzero initial slope chosen so the ramp
hits the target tick exactly — both ramp endpoints are algebraically unaffected by this, so
the calibrated total time of `ACCPCT` to 15mph never changes regardless of `ACCTGT`. `ACCTGT` can only *shorten*
the onset from its own natural baseline, never push it later. Very aggressive `ACCTGT` values can make the
ramp briefly non-monotonic later in its climb (confirmed tiny, below display resolution, at the one extreme
tested).

**Requires a linearized decoder speed table**: the simulation assumes real locomotive speed scales linearly
with the commanded DCC speed step — this only holds if the decoder own speed-table CVs are themselves
calibrated linear across the full 0-126 range. A non-linear/uncalibrated decoder speed table will make the
display diverge from the real locomotive regardless of `SPEED` tuning, since that is a decoder-side
characteristic entirely outside this codebase.

### Reference: `SPEED` field definitions and tested values

`ACCEL`/`DECEL`/`BRK1-3`/`DELAY` are named after their CV role, but the CV convention itself is inverted
from what the name suggests — a *bigger* number means *slower*/*weaker* (a time constant, not a rate); that
is an NMRA convention, not a naming choice made here.

**Momentum CVs (direct decoder mirrors)**

| Item | Default | Tested | What it does | Increase | Decrease |
|---|---|---|---|---|---|
| `ACCEL` (CV3) | 60 | 60 | Time to cross the full speed range while speeding up | Slower acceleration | Faster acceleration |
| `DECEL` (CV4) | 230 | — | Same as `ACCEL` but for coasting down (no brake held). **Values above 230 showed non-linear behavior on real decoder hardware during calibration — not recommended; `DECTHR`/`DECPCT` were only validated up to `DECEL=230`.** | Slower coast-down (up to 230) | Faster coast-down |
| `BRK1` (CV179) | 130 | — | How strongly Brake1 shortens the stop when active — sums with `BRK2`/`BRK3` (capped at 255) | Faster/harder stop | Weaker braking |
| `BRK2` (CV180) | 70 | — | Same as `BRK1`, second stackable brake | Faster/harder stop | Weaker braking |
| `BRK3` (CV181) | 100 | — | Same as `BRK1`, third stackable brake | Faster/harder stop | Weaker braking |
| `DELAY` (CV167) | 13 | — | Mirrors the decoder own programmed prime-mover spool-up time. 0.25s/unit | Longer pause before movement | Shorter pause (0 = none) |

**Display calibration**

| Item | Default | Tested | What it does |
|---|---|---|---|
| `MAXSPEED` | 50 | 50 | Real-world scale speed (mph) at speed step 126 — the calibration anchor |
| `UNIT` | MPH | — | MPH or KMH display |

**Watched-function triggers**

| Item | Default | What it does |
|---|---|---|
| `HOLDFN` | F09 | DCC function watched for ESU Drive Hold — display freezes while asserted |
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
| `TYPE` | V5DCC | `V5DCC` (0.896s/unit) or `V4V5MULT` (0.25s/unit) |

**Correction tunables** (hidden behind `ADV FUNC`)

| Item | Default | Tested | What it does |
|---|---|---|---|
| `ACCPCT` | 8 | 8 | Standing-start head start (0-255 = 0-100% of the `ACCEL` full-range time) |
| `ACCTGT` | 5 | 5 | Target time (0.1s/unit) for the display to first show 1mph |
| `DECPCT` | 22 | 22 | Strength (0-255 = 0-100%) of the steady-state deceleration-lag correction |
| `DECTHR` | 11 | 11 | Speed (raw step) below which the `DECPCT` correction does not apply |

Confirmed working values for the calibration locomotive: `ACCEL=60`, `MAXSPEED=50`, `DECTHR=11`,
`DECPCT=22`, `ACCPCT=8`, `ACCTGT=5` (the shipped defaults already match). Every other field above is still
the shipped compile-time default, not independently re-validated against that locomotive.

## Shared network CNF store

Lets one loco CNF (configuration profile: DCC function assignments, brake/STACK settings, notch table,
speed/momentum CVs — the existing 128-byte slot format) be shared wirelessly across multiple ProtoThrottle
units, instead of requiring physical ISP access to hand-copy JSON between them. **`mrbw-cabbus`** (the
sibling NCE Cab Bus gateway repo) is the authoritative store, since it is already layout infrastructure
since it is already layout infrastructure that is powered whenever the layout is. Holds 20 independent network slots (`N01`-`N20`).

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
real length. A push `BEGIN` carries the `EEPROM_LAYOUT_VERSION` of the pushing throttle; compared against the
pin: unset or strictly higher → accepted, and on COMMIT wipes every *other* entry and advances the pin
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
refused with a `CHECK` / `RETRY` screen — the firmware will not push blind, since it cannot then tell whether
the push would advance the pin. If the peek succeeds and the pin is unset or older than the `EEPROM_LAYOUT_VERSION` of this throttle,
`EEPROM_LAYOUT_VERSION`, a two-stage "UPGRADE" / "WIPE N01-20?" confirmation runs before the real push, so
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
low-level `push_entry()` still accepts an arbitrary version number, which remains useful for testing the
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
the decode logic in `readConfig()` inside `mrbw-cst.c`, not generated from them, since the C headers only give byte offsets,
not the bitfield/enum/multi-byte-array semantics that live in the firmware control flow. The
`-h`/`--help` output — including per-subcommand help, e.g. `cst_cfgtransfer.py import -h` — documents every
flag in more detail than covered below; check there for the exact current option set.

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
`functions` / `notch_speedstep` / `speed` / `options` (the OPTIONS menu — brake config plus
`reverser_swap`/`horn_type`; its meta-field is `unset`); `device.json` is `system` (ADV-FUNC battery
thresholds) / `comm` / `prefs` (`config_bits` nested here) / `calibration`. `encode_slot` /
`encode_global` also accept the older pre-schema-2 shapes on import (flat device fields,
`force_function_on`/`off`, `brake` / `options_unset`). `MenuOrderTests` in `test_slot_codec.py` locks
the ordering so a `slot_codec.py` change is deliberate.

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

**EEPROM write reliability**: `import` retries the whole write up to 3 times on failure — see
the module docstring of `avrdude_io.py` for the full story, including a confirmed driver-level mechanism
(the EEPROM write path of `iseavrprog`/`usbtiny` has a much narrower timing margin than flash, with no retry on
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

`src/cst-cfgnetwork/cst_cfgnetwork.py` lets a PC export/import loco configurations to/from the
shared network CNF store (`N01`-`N20`, see "Shared network CNF store" above) directly over a USB-attached
XBee radio — no physical throttle and no ISP access to any device needed. It imports `slot_codec.py`/
`cst_eeprom_layout.py` directly from `cst-cfgtransfer/` rather than duplicating the codec, since the
128-byte network-entry payload is byte-identical to a local slot.

`src/cst-cfgnetwork/cnf_radio_io.py` is a from-scratch, Python-3-only implementation of XBee API-frame
escaping/framing, MRBus CRC16, and the full `'C'`/`'D'` `BEGIN`/`DATA`/`COMMIT`/`DONE` client state machine
(`pull_entry()`/`push_entry()`/`peek_loco_address()`), plus two helpers that are not part of the `'C'`/`'D'`
protocol: `discover_nodes()` (an MRBus presence-ping sweep) behind the `discover` subcommand, and
`sniff()` + `decode_cst_status()` (a passive, read-only radio listener that decodes the throttle status
`'S'` packet — loco/direction/speed step/function mask/status flags/battery, and with `--cnf` the
shared-CNF `'C'`/`'D'` transfer packets too, via `format_cnf_packet()`) behind the `sniff` subcommand.
Requires a spare XBee3 module joined to the same PAN as the throttle/receiver radios, such as the
`ckt-xbee` USB-to-XBee adapter.

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
gives a fast all-20-entries loco-address overview via `peek_loco_address()` with no full pulls. The default
integrity check is the same COMMIT CRC gate the push itself already uses on the receiver side; `--verify` adds an opt-in
pull-and-diff. `export --dir`-style batch imports auto-skip `NONE`/never-configured stub entries (an
un-decodable-back all-`0xFF` payload) rather than aborting the whole batch on the first one, but a single
explicitly-named file pointed straight at a stub still errors, since silently doing nothing for the one thing
explicitly requested would be worse than the error.

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
`--import-old` above — restores a backup exported under an older schema by defaulting any field absent
from the file rather than rejecting it.

**`reset-cabbus`**: wipes the whole shared network CNF table and clears the version pin via the wire
protocol `SUBTYPE_RESET` (see "Shared network CNF store" above) — a deliberate, human-triggered reset,
not something any version mismatch triggers automatically. Backs up all 20 entries by default (`--out-dir`
required unless `--skip-backup`), reusing the same output-folder convention `export` already uses.

### Maintenance checklist — follow whenever the EEPROM layout changes

New field, moved offset, or repurposed byte in `cst-eeprom.h`:

1. Add/change the field in `src/cst-eeprom.h` and wire up `readConfig()`/save-path code in `mrbw-cst.c`.
2. Mirror the same offset/type/decode logic in `cst_eeprom_layout.py` and the
   `decode_slot()`/`decode_global()`/`encode_slot()`/`encode_global()`.
3. Bump `EEPROM_LAYOUT_VERSION` in `cst-eeprom.h` — the Python tooling parses that `#define` at import
   (`cst_eeprom_layout._read_firmware_layout_version()`), so there is no second copy to keep in step.
4. Add/update the corresponding fixture in `src/cst-cfgtransfer/tests/test_slot_codec.py` (run via
   `python3 -m unittest discover tests` from `src/cst-cfgtransfer/`, no hardware needed).
5. Update the field reference in `src/cst-cfgtransfer/README.md` if the field introduces new JSON
   vocabulary.

One local git hook (`.githooks/pre-commit`, wired up by `make setup`) guards against this checklist being
followed incompletely: `check_layout_change_bumps_version.py` catches a layout change that never bumped
`EEPROM_LAYOUT_VERSION` at all, by diffing the `#define` set of `cst-eeprom.h` against its previous committed
state. It cannot catch a change that reinterprets what an existing, unmoved byte value *means* (a new
enum numbering, repurposed bits) without changing its offset — that class of drift is only caught by
careful review, or by the targeted per-field regression tests in `test_slot_codec.py`.

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
`mrbw-cabbus.c` calls `cnfStoreHandlePacket()` first on every packet, but the first line of that function is
`if(CNF_PKT_TYPE_PUSH != type && CNF_PKT_TYPE_PULL != type) return 0;` — i.e. `'C'`/`'D'` only. A stock
throttle never transmits `'C'`/`'D'`, so this is a no-op on every packet it sends; dispatch falls through
to the unmodified chain exactly as before the CNF store existed.

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
ESP32-S2 MRBus↔WiFi bridge) was evaluated from source as a second possible host for the shared network CNF
store, alongside `mrbw-cabbus`. Architecturally sound and arguably a better long-term host (no throttle-side
change needed at all, and ample flash/RAM headroom), but a genuine from-scratch port rather than a
code-reuse job, with the storage-placement question (a dedicated flash partition vs. the existing FAT
partition already used for `config.txt`, which is also exposed raw over USB and so has a real dual-writer
hazard) left open. Not implemented — blocked on hardware availability to test against.
