# Air-brake model reference-trace test

A host-compiled regression net for the AIRBRAKE air-brake simulation in
`../cst-pressure.c` (see the `AIRBRAKE` section of the root `CLAUDE.md`). Same
technique and layout as `../cst-speed-test/`.

## What it does

`test_pressure.c` compiles `../cst-pressure.c` natively (plain `cc`, not
`avr-gcc`) and drives the real `updateBrake10Hz()` model through a fixed set of
scenarios - the idle compressor cycle, the initial full-service reduction and
its release, the minimum-reduction floor, graduated service, lapping, emergency
(with and without the `BRK ESTP` option), the `COMPMODE = CONSIST` deep/routine
compressor split, and the two defensive config guards. Every 10 Hz tick it
records the model state (`bpMilliPsi`, `mrMilliPsi`, the consist-sync credit) and
all six output accessors (`airBrakeReleased` / `airBrakeSetPulse` /
`airCompressorOn` / `airCompressorPendingRelease` / `airCompressorReleaseRun` /
`airEmergencyActive`) plus the two rounded whole-PSI readouts
(`airBrakePipePsi` / `airMainResPsi` - the AIRBRAKE screen "render") into a
plain-text **trace**, one file per scenario, compared byte-for-byte against the
checked-in copies under `reference/`.

`main()` also asserts three invariants it prints as `PASS`/`FAIL` lines and exits
non-zero if any fails:

1. the idle governor stays bounded and actually cycles both ways;
2. an inverted `MR LOW >= MR HIGH` config does not make the compressor stutter;
3. the brake pipe recharges to *exactly* `BP CHARGE` after an emergency dump.

A difference means the model output moved - either an intended change (a tuned
constant, a model fix) and the reference is re-blessed, or an unintended
regression and the code is fixed. Snapshot / characterization / golden-master
testing; "reference trace" is the name used in this tree.

## Running it

From `../` (the `src/` directory):

```
make pressuretest          # build + run + diff against reference/  (PASS / FAIL)
make pressuretest-accept    # regenerate reference/ from the current cst-pressure.c
```

`make pressuretest` also runs automatically from `.githooks/pre-commit`, but only
when the commit touches `cst-pressure.c`, `cst-pressure.h`, or this directory.

After `make pressuretest-accept`, always inspect `git diff` on `reference/`
before committing - that diff is the human-readable statement of exactly how the
model behaviour changed, and every trace should still read as physically sensible
(pipe vents then recharges, reservoir sawtooths, `S` pulses fire on reduction
edges, `E` only ever sets with `BRK ESTP` on).

## Layout

| Path | Purpose |
|---|---|
| `test_pressure.c` | the harness - scenario definitions, `cfgReset()` / `modelReset()`, the invariants, `main()` |
| `reference/*.txt` | the blessed trace for each scenario |
| `out/` | scratch output of the last run (git-ignored) |
| `pressuretest` | the compiled harness (git-ignored) |

No `stubs/` directory: `cst-pressure.h` pulls only `<stdint.h>` and
`cst-pressure.c` adds only `<stdlib.h>`, so the harness `#include`s the model
directly with no include-path shims (unlike the scale-speed harness, which needs
`stubs/avr/pgmspace.h` for `src/lcd.h`).

## Scope and fidelity

`updateBrake10Hz()` and every accessor use `uint32_t` / `UL` literals throughout
with no bare `int` intermediates, so the model is **width-identical between AVR
16-bit `int` and host 32-bit `int` by construction** - a stronger guarantee than
the scale-speed model (which needs its "keep CVs in non-zero ranges" caveat). The
one host/AVR divergence point is the `(uint8_t)rand()` gauge jitter
`initAirBrake()` applies to the two pressures; `modelReset()` pins them to exact
values right after, so it never reaches a trace.

The harness drives `independentBrakeAtRest` (the second `updateBrake10Hz()`
argument) directly. In the firmware that value is derived per `BRK TYPE` in
`mrbw-cst.c` (`~5389`); deriving it is out of scope here - the honest unit
boundary is the parameter, which `cst-pressure.c` already takes for exactly this
reason.

The `P` (`airCompressorPendingRelease`) and `X` (`airCompressorReleaseRun`)
columns are computed by the model *every tick regardless of `COMPMODE`* -
`COMPMODE` only decides whether they split `COMPRESSOR_FN` / `COMPRESSOR2_FN` in
`mrbw-cst.c` - so they appear in `NORMAL`-mode traces too.

`AIRBRAKE_DISPLAY` (DUAL / SINGLE) is not covered: it selects an `AIRBRAKE_SCREEN`
rendering in `mrbw-cst.c` and has no effect on the `cst-pressure.c` model.

Adding a scenario: write a `sc_*()` function, call it from `main()`, run
`make pressuretest-accept`, review the new `reference/` file, commit.
