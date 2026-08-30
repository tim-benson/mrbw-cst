# mrbw-cst (ProtoThrottle X)

Hardware + firmware source for the **MRBW-CST** ("Control Stand Throttle") from
[Iowa Scaled Engineering](https://www.iascaled.com/) — the wireless throttle sold as the **ProtoThrottle**.
This fork is **ProtoThrottle X** — the `X` is for *Extensions*.

This is a fork adding:

- **Headlights debounce** — fixes a brief dark-flicker glitch when twisting the front or rear
  Headlights knob between detents.
- **STACK brake mode** — a fourth brake mode driving up to three stackable DCC brake functions (e.g.
  ESU LokSound/LokPilot V5 Brake1/2/3) from lever position, in 3-step or 5-step variants.

## Repo layout

`src/` is the only directory with code in the normal sense — the AVR firmware. Everything else (`sch/`,
`pcb/`, `fp/`, `sym/`, `mech/`, `doc/`, `pg/`) is gEDA schematic/PCB CAD, 3D-printable parts, and
datasheets/manuals for the physical hardware.

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

`CLAUDE.md` at the repo root is the technical reference — architecture and build/flash detail today,
growing to cover this fork's added features as they land.

## License

GPLv3 — see `LICENSE`.
