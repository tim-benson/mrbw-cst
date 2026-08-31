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
  scale mph/km-h readout modeling ESU LokSound/LokPilot V4/V5 momentum and brake deceleration, so the
  display stays in visual sync with what the decoder is actually doing.
- **Offline loco-config export/import (cfgtransfer)** — `cst_cfgtransfer.py`, a Python 3 stdlib-only
  tool to back up and hand-edit stored loco profiles as JSON over the ISP programmer, with the firmware
  not running.
- **Wireless shared loco-config store (cabbus + cfgnetwork)** — up to 20 loco profiles (`N01`-`N20`)
  stored on the sibling [`mrbw-cabbus`](https://github.com/tim-benson/mrbw-cabbus) gateway and shared
  across throttles over the radio, plus `cst_cfgnetwork.py` for wireless access to the store from a
  computer.

Everything else — DCC status relay, fast clock, EEPROM read, ping, version query — is unchanged from
stock ISE firmware; the one stock feature this fork removes is the `ACCEPT DOWNLOAD` menu item and the
wireless EEPROM-write (`'W'`) packet it gated, superseded by the config tooling above. A mixed fleet of
stock and fork-firmware throttles/receivers on the same layout is fully supported; see `CLAUDE.md`,
"Compatibility," for exactly how each combination behaves.

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

## Documentation

`CLAUDE.md` at the repo root is the full technical reference — architecture, every feature's design
rationale, the shared configuration store's wire protocol, and the PC tooling. Start there for anything
beyond a quick build.

## License

GPLv3 — see `LICENSE`.
