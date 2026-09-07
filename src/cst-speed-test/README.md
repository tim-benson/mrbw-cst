# Scale-speed model reference-trace test

A host-compiled regression net for the scale-speed simulation in `../cst-speed.c`
(the `SPEED` feature - see the `SPEED` section of the root `CLAUDE.md`).

## What it does

`test_speed.c` compiles `../cst-speed.c` natively (plain `cc`, not `avr-gcc`) and
drives the real `updateSpeed10Hz()` model through a fixed set of scenarios -
standing starts, coast-downs, the three stacked brakes, e-stop, Drive Hold, Start
Delay, the load CVs, CV sweeps, and the two decoder types. Every 10 Hz tick it
records the internal `simSpeedStepQ8` value and the `printSpeed()` render into a
plain-text **trace**, one file per scenario. Those traces are compared
byte-for-byte against the checked-in copies under `reference/`.

A difference means the model output moved. Either it is an intended change (a new
correction term, a tuned default, a genuine behaviour fix) and the reference is
re-blessed, or it is an unintended regression and the code is fixed.

This technique is known more widely as snapshot, characterization, or
golden-master testing. "Reference trace" is the name used in this tree.

## Running it

From `../` (the `src/` directory):

```
make speedtest          # build + run + diff against reference/  (PASS / FAIL)
make speedtest-accept    # regenerate reference/ from the current cst-speed.c
```

`make speedtest` also runs automatically from `.githooks/pre-commit`, but only when
the commit touches `cst-speed.c`, `cst-speed.h`, or this directory.

After `make speedtest-accept`, always inspect `git diff` on `reference/` before
committing - that diff is the human-readable statement of exactly how the model
behaviour changed.

## Layout

| Path | Purpose |
|---|---|
| `test_speed.c` | the harness - scenario definitions, LCD capture shims, `main()` |
| `reference/*.txt` | the blessed trace for each scenario |
| `stubs/avr/pgmspace.h` | no-op host shim, needed only because `src/lcd.h` includes `<avr/pgmspace.h>` |
| `out/` | scratch output of the last run (git-ignored) |
| `speedtest` | the compiled harness (git-ignored) |

## Scope and fidelity

The harness keeps every scenario in realistic, non-zero momentum-CV ranges, where
`cst-speed.c` is provably identical between AVR 16-bit `int` and host 32-bit `int`
(the file uses explicit-width types with casts throughout; the only divergence
point is a zero CV, which the scenarios avoid). Bit-for-bit agreement with the
flashed firmware is separately covered by the on-hardware drive check that goes
with each `SPEED` change. The header comment in `test_speed.c` spells this out.

Adding a scenario: write a `sc_*()` function, call it from `main()`, run
`make speedtest-accept`, review the new `reference/` file, commit.
