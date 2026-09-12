# mrbw-cst (ProtoThrottle X)

Hardware + firmware source for the **MRBW-CST** ("Control Stand Throttle") from
[Iowa Scaled Engineering](https://www.iascaled.com/) — the wireless throttle sold as the **ProtoThrottle**.
This fork is **ProtoThrottle X** — the `X` is for *Extensions*.

This is a fork adding:

- **Headlights debounce** — fixes a brief dark-flicker glitch when twisting the front or rear
  Headlights knob between detents.
- **STACK brake mode** — a fourth brake mode driving up to three stackable DCC brake functions (e.g.
  ESU LokSound/LokPilot V5 Brake1/2/3) from lever position, in 3-step or 5-step variants.
- **Two-stage horn ("Horn2")** — a second, independently-calibrated horn function, additive or exclusive
  with the primary horn.
- **Scale-speed simulation ("SPEED")** — replaces the main screen's fast clock with a locally-computed
  scale mph / km/h readout modeling ESU LokSound/LokPilot V4/V5 momentum and brake deceleration, so the
  display stays in visual sync with what the decoder is actually doing. Covers the momentum CVs (CV3/CV4
  and the CV23/CV24 adjust), the brake and load CVs, and start delay.
- **Offline loco-config export/import (cfgtransfer)** — `cst_cfgtransfer.py`, a Python 3 stdlib-only
  tool to back up and hand-edit stored loco profiles as JSON over the ISP programmer, with the firmware
  not running.
- **Wireless shared loco-config store (cabbus + cfgnetwork)** — up to 20 loco profiles (`N01`-`N20`)
  stored on the sibling [`mrbw-cabbus`](https://github.com/tim-benson/mrbw-cabbus) gateway and shared
  across throttles over the radio, plus `cst_cfgnetwork.py` for wireless access to the store from a
  computer.
- **Menu-button refinements** — long-press Menu to cancel an in-progress config edit, a brief backlight
  hold so the light does not strobe between menu laps, and the SELECT backlight toggle moved to button
  release.
- **OPS MODE** — an opt-in base-screen variant (long-press Menu to enter and leave) that frees the Menu
  and Select buttons to drive two more assignable DCC functions (`MENU BTN` / `SEL BTN`, same options
  as `UP BTN` / `DOWN BTN`) while running.
- **LOAD button function** — a fifth option for `UP BTN` / `DOWN BTN` / `MENU BTN` / `SEL BTN` that
  cycles OFF -> Optional Load -> Primary Load -> OFF on each press, driving SPEED's `OPLOADFN`/`PRLOADFN`
  without wiring a separate control to the same DCC function.
- **CLOCK Peek button function** — a sixth option for `UP BTN` / `DOWN BTN` / `MENU BTN` / `SEL BTN`
  that, while held, temporarily swaps the SPEED readout for the fast clock, reverting the instant it is
  released; offered only while the SPEED display is enabled.
- **AIRBRAKE train-brake simulation** — models a locomotive automatic (train) brake — brake pipe, main
  reservoir, compressor governor — driving the ESU air sound functions from the simulated pressures,
  with a read-only gauge screen (two-pressure or analogue-dial style). Opt-in via a PREFS toggle;
  replaces the stock transient "Brake Test" gauge.

Everything else — DCC status relay, fast clock, EEPROM read, ping, version query — is unchanged from
stock ISE firmware. The stock features this fork drops: the `ACCEPT DOWNLOAD` menu item and its
wireless EEPROM-write (`'W'`) packet (superseded by the config tooling above), and the "Special
Functions" menu whose transient Brake Test gauge AIRBRAKE replaces. A mixed fleet of stock and
fork-firmware throttles/receivers on the same layout is fully supported; see `CLAUDE.md`,
"Compatibility," for exactly how each combination behaves.

The stored-config layout carries an `EEPROM_LAYOUT_VERSION` (currently 5) and the firmware migrates an
older EEPROM forward on first boot. Migrations carry existing per-profile values across in place, with one
exception: a throttle already running an early ProtoThrottle X build loses its SPEED and AIRBRAKE
per-profile config to defaults when it crosses the AIRBRAKE layout change. Coming from stock ISE firmware
nothing is lost. Either way, back up a configured throttle with `cst_cfgtransfer.py` before a firmware
upgrade and re-import afterward if anything looks reset.

## How this fork was developed

This fork was implemented with substantial help from an AI coding assistant (Claude) — hence the
`Co-Authored-By` trailers throughout the history. The assistant was used to turn designs into C, build
the reference-trace test harnesses, and draft documentation. The design and engineering are my own: the
feature set, the simulation models and their calibration against real decoders, the EEPROM layout and its
forward-migration strategy, and the wire protocol for the shared configuration store. Every change that
affects firmware behaviour was flashed to a physical ProtoThrottle and tested against real locomotives
and decoders before being committed; the commit messages record the specifics.

## Repo layout

`src/` is the only directory with code in the normal sense — the AVR firmware, plus the two Python PC
tools. Everything else (`sch/`, `pcb/`, `fp/`, `sym/`, `mech/`, `doc/`, `pg/`) is gEDA schematic/PCB CAD,
3D-printable parts, and datasheets/manuals for the physical hardware.

## Build / flash quick start

```bash
cd src
make setup   # one-time: fetches the src/mrbus git submodule
make hex     # compile -> mrbw-cst.hex
make program # flash a connected board via an ISP programmer (fuse + flash in one step)
```

Target: ATmega1284P @ 11.0592 MHz / 3.3V, `avr-gcc`. See `CLAUDE.md`, "Build / flash," for full detail
including the macOS/Homebrew setup and programmer options.

The SPEED and AIRBRAKE simulation models and the EEPROM layout migrations each have a host-compiled
golden-master test — `make speedtest`, `make pressuretest`, `make eepromtest` (run automatically by a
pre-commit hook) — that locks their behaviour against a set of reference traces.

## Documentation

`CLAUDE.md` at the repo root is the full technical reference — architecture, every feature's design
rationale, the shared configuration store's wire protocol, and the PC tooling. Start there for anything
beyond a quick build.

## License

GPLv3 — see `LICENSE`.
