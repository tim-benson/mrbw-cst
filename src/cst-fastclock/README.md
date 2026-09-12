# cst_fastclock.py

Broadcasts an MRBus/MRBee fast-clock `'T'` packet over a USB-attached XBee radio, so a ProtoThrottle
throttle running this fork firmware can display a running fast clock during bench testing - no physical
[ISE mrb-fcm](https://github.com/IowaScaledEngineering/mrb-fcm) fast-clock-master hardware needed.

This throttle `src/cst-time.c` already fully implements the receive side of the ISE fast-clock wire
protocol unchanged (`processTimePacket()`), so this tool only needs to *transmit* - periodically, no
ack/retry logic, matching the fire-and-forget behavior of mrb-fcm itself. It shares the same USB-XBee
hardware and radio transport (`cnf_radio_io.py`) as [`cst-cfgnetwork`](../cst-cfgnetwork/), reused via the
same cross-directory `sys.path` import that tool itself uses to reach `cst-cfgtransfer/`. It has no
EEPROM/config-slot dependency at all - just this one script.

## Requirements

Same USB-XBee hardware and radio setup as `cst-cfgnetwork` - see
[`../cst-cfgnetwork/README.md`](../cst-cfgnetwork/README.md) for the full XBee firmware/profile and
platform (macOS/Linux/Windows) notes; they apply unchanged here. In short: Python 3, `pyserial`
(`pip install pyserial`), and a USB-attached XBee3 module configured with the same firmware/profile the
cst and cabbus boards use.

## One-time throttle setup

On the throttle on-device menu, before testing:

- **`COMM CFG` -> `TIME ADR`**: set to `0xFF` so the throttle accepts a `'T'` packet from any MRBus
  source address, regardless of what `--my-addr` this tool is run with. (Leaving it at the factory default
  of `0x00` instead restricts acceptance to whatever `BASE ADR` the throttle is itself configured to - only
  do this if deliberately running the tool as that same address instead of touching `TIME ADR`.)
- **`PREFS` -> `DISPLAY`**: set to the clock option, not `SPEED` - that preference decides whether the
  main screen shows the fast clock or the SPEED scale-speed readout at all (see this repo `CLAUDE.md`,
  "SPEED" section). The fast clock still runs internally either way; it is just not visible on-screen
  under `DISPLAY = SPEED`.

## Quick start

```bash
python3 cst_fastclock.py --port /dev/cu.usbserial-D30DUGOT --my-addr 0x50 --start 08:00 --ratio 4
```

Starts the fast clock at 08:00, running at 4x real time, broadcasting an update every 2 seconds until
`Ctrl+C`. `--my-addr` just needs to be an MRBus address not otherwise in use on the layout - there is no
fixed convention for a fast-clock-master-like accessory node, unlike the throttle (`0x30`-`0x49`) and
base-station (`0xD0`-`0xEF`) ranges `cst-cfgnetwork` uses.

Other flags (`--help` for the full list): `--interval` (seconds between broadcasts, default 2.0),
`--ampm` (12-hour display instead of 24-hour - note that the AM/PM indicator is a small custom glyph, not
literal text, drawn in the last column of the time field), `--hold` (broadcast the clock paused at
`--start`, to exercise the `" HOLD "` display state of the throttle).

## Wire format

MRBus/MRBee packet type `'T'`, broadcast (`dest = 0xFF`), 18 bytes total (6-byte MRBus header + 12
payload bytes) - copied unchanged from
[the mrb-fcm.c of ISE](https://github.com/IowaScaledEngineering/mrb-fcm/blob/master/src/mrb-fcm.c) and
consumed unchanged by `src/cst-time.c:processTimePacket()` on this throttle:

| Payload offset | Field | Notes |
|---|---|---|
| 0 | real hours | 0-23; parsed but not displayed while the fast clock is active |
| 1 | real minutes | |
| 2 | real seconds | |
| 3 | flags | bit0 `DISP_FAST`, bit1 `DISP_FAST_HOLD`, bit2 `DISP_REAL_AMPM`, bit3 `DISP_FAST_AMPM` |
| 4 | fast hours | 0-23 |
| 5 | fast minutes | |
| 6 | fast seconds | |
| 7-8 | `timeScaleFactor`, big-endian `uint16` | ratio x 10 (e.g. 40 = 4.0:1) |
| 9-11 | year/month/day | sent as zero - never read by this throttle firmware |

This tool always sets `DISP_FAST` (bit0) so the throttle shows the fast clock rather than real time; the
throttle dead-reckons the fast time forward between broadcasts at the given ratio, and falls back to
`--:--` if no packet arrives within the configured dead-reckoning timeout of the throttle (a few seconds
by default) - so the default `--interval 2` comfortably keeps it live.

## Tests

```bash
python3 -m unittest discover tests
```

Unit tests for the pure time-computation and packet-payload-building functions - no hardware or serial
port involved, matching the pattern in `cst-cfgnetwork/tests/test_cnf_radio_io.py`.
