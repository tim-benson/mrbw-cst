# Flashing ProtoThrottle X firmware

This guide is for owners of a physical ProtoThrottle who want to install ready-built firmware — no
programming knowledge, and no compiler, required. If you are a developer who wants to build the firmware
from source instead, see `README.md` and `CLAUDE.md`.

Firmware files are published on the repo's
[Releases page](https://github.com/tim-benson/mrbw-cst/releases). Each release has a `.hex` file attached
— that is the file you flash. There are two kinds of release:

- **Milestone releases** (named like `X1.0`) — stable, maintainer-tested versions. Pick one of these if
  you are not sure which to use.
- **Pre-releases** (named like `X1.0.1`, marked "Pre-release" on GitHub) — the newest firmware, built
  automatically after every change. Use one of these if you have been asked to try a specific fix.

## What you need — hardware

1. **The ProtoThrottle itself**, with its case open enough to reach the small programming header (called
   the "ISP header") on the circuit board inside.
2. **An AVR ISP programmer.** This is a small separate device that plugs in between your computer and the
   throttle's circuit board — it is not the same as a USB cable to the throttle. Any of the following
   work:
   - The **ISE AVR Programmer** — this works with no setup changes, since it is the default this project
     is configured for.
   - A **USBtinyISP**-compatible programmer.
   - An **Atmel/Microchip AVRISP mkII**.

   If you are using a USBtinyISP or an AVRISP mkII (not the ISE AVR Programmer), you need to make one
   small edit before flashing — see step 4 below.
3. **A USB cable** from your computer to the programmer, and the small ribbon/pin cable from the
   programmer to the throttle's ISP header.
4. **A computer** — macOS or Linux is the most straightforward; Windows works too, but non-ISE programmers
   may need an extra driver step (search for "Zadig" and your programmer's name if Windows does not
   recognize it).

## What you need — software

- **`avrdude`** — the small tool that actually talks to the programmer and writes the firmware onto the
  chip. This is the only tool you need to install; you do **not** need a compiler, `avr-gcc`, or anything
  else described in `README.md`/`CLAUDE.md` for building from source. You are installing a file someone
  else already built, not building one yourself.
- **A copy of this repository**, even though you are not compiling anything from it. The reason: this
  repo's `src/Makefile` already knows the correct chip type, timing settings, and programmer settings, and
  hands them to `avrdude` for you — so you never have to type a long, error-prone `avrdude` command by
  hand. Get it either with:
  ```bash
  git clone https://github.com/tim-benson/mrbw-cst.git
  ```
  or by using GitHub's green "Code" button and "Download ZIP" if you do not have `git` installed.
- **Python 3** — only needed for the optional backup step below. Most Macs and Linux machines already
  have it; Windows users can get it from the Microsoft Store or python.org. No extra packages need
  installing.

### Installing avrdude

On macOS, with [Homebrew](https://brew.sh/) already installed:
```bash
brew install avrdude
```
(avrdude 8.2 and newer already knows about the ISE AVR Programmer out of the box — no extra configuration
file editing needed.)

On Linux, use your distribution's package manager, e.g. `sudo apt install avrdude` on Debian/Ubuntu.

On Windows, download an `avrdude` build from its project page, or install it via a package manager such as
`winget install avrdude` if you have `winget`.

## Back up your loco profiles first (recommended)

If this throttle already has loco profiles saved on it (function assignments, brake settings, speed
calibration, and so on), back them up before flashing anything new. Firmware updates are designed not to
disturb your saved settings, but an update can occasionally reset them back to factory defaults as part of
a bigger change, so a backup costs a couple of minutes and removes all the risk.

1. **Connect the programmer** to the throttle's ISP header and to your computer with USB — the same
   connection you will use for flashing in the next section.
2. **Run the backup tool**, from the `src/cst-cfgtransfer/` folder inside the repo you downloaded:
   ```bash
   python3 cst_cfgtransfer.py export --out-dir ~/protothrottle-backup/
   ```
3. **Check the output** — you should see a new folder appear under `~/protothrottle-backup/` named after
   your throttle, containing several `.json` files.
4. **Keep that folder somewhere safe** until you have confirmed the new firmware is working the way you
   expect.

If anything looks reset after updating, you can restore from this backup:
```bash
python3 cst_cfgtransfer.py import --dir ~/protothrottle-backup/throttle-<your-throttle's-address>/ --yes
```

## Step-by-step flashing

1. **Download the firmware.** On the [Releases page](https://github.com/tim-benson/mrbw-cst/releases),
   pick a release and download the `.hex` file attached to it (for example `mrbw-cst-X1.0.1.hex`). Save
   it somewhere you can find it again, such as your Desktop or Downloads folder.
2. **Get the repository**, if you have not already — see "What you need — software" above.
3. **Connect the hardware** — programmer to computer over USB, and programmer to the throttle's ISP header
   with the ribbon/pin cable. The cable is keyed so it only fits one way; do not force it.
4. **If you are using a USBtinyISP or AVRISP mkII** (not the ISE AVR Programmer), open `src/Makefile` in a
   plain text editor and change these three lines near the top from:
   ```makefile
   #PROGRAMMER_TYPE=avrispmkii
   #PROGRAMMER_TYPE=usbtiny
   PROGRAMMER_TYPE=iseavrprog
   ```
   to (for a USBtinyISP, for example):
   ```makefile
   #PROGRAMMER_TYPE=avrispmkii
   PROGRAMMER_TYPE=usbtiny
   #PROGRAMMER_TYPE=iseavrprog
   ```
   (Move the `#` so that only your programmer's line is uncommented.) If you are using the ISE AVR
   Programmer, skip this step — nothing to change.
5. **The first time you program a chip, set its fuses.** Fuses are a couple of low-level chip settings —
   not the firmware itself — that only need to be written once per chip, or after a full chip erase. From
   the `src/` folder of the repository:
   ```bash
   make fuse
   ```

   **Warning:** `src/Makefile` has a line just above the fuse settings that is commented out:
   `#FUSE_H  = 0xD9  # Erase EEPROM after programming`. Leave this line commented out. Uncommenting it
   makes every future flash wipe all saved loco profiles from the throttle. Only use it if that is
   specifically what you want to do.
6. **Flash the firmware you downloaded**, pointing directly at the file — no renaming needed:
   ```bash
   make firmware HEX=/path/to/mrbw-cst-X1.0.1.hex
   ```
   (Replace the path with wherever you saved the file in step 1.)
7. **Check it worked.** `avrdude` should finish with lines like `avrdude: verifying ...` and
   `... bytes of flash verified` and no error messages. If instead you see a timeout or "not responding"
   error, it almost always means a connection problem (a loose cable, the wrong USB port, or the throttle
   not powered) rather than anything wrong with the firmware file itself — recheck the cabling and try
   again.

## After flashing

Power the throttle on and confirm it starts up normally. If you made a backup earlier, check that your
loco profiles are still there; if they are not, restore them with the `cst_cfgtransfer.py import` command
shown above.

## Getting help

For the fuller technical picture of how this fork works — the versioning scheme, the EEPROM layout, every
feature's design — see `README.md` and `CLAUDE.md` in this repository. If something goes wrong that this
guide does not cover, open an issue on the
[GitHub Issues page](https://github.com/tim-benson/mrbw-cst/issues).
