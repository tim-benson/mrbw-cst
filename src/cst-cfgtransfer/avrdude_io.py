# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""Thin subprocess wrapper around avrdude for reading/writing the throttle's EEPROM over the ISP
programmer.

EEPROM writes over this programmer (`iseavrprog`, a rebrand of avrdude's `usbtiny` driver - every SPI
operation is a `usb_control_msg()` call to a small AVR bit-banging both SPI and USB in firmware, not a
fast bulk-transfer protocol) can fail intermittently. Root-caused by reading avrdude 8.2's actual source
and installed avrdude.conf, not guessed:

- The ATmega1284P's *flash* memory is configured `paged = yes` in avrdude.conf; its *eeprom* memory never
  is. So EEPROM writes take usbtiny.c's non-paged write path, which bakes the real per-byte EEPROM
  write-cycle delay (max_write_delay = 9000us for this part) into the timeout window of every subsequent
  USB chunk transfer - at -B1 that's 32 separate 128-byte chunks for a full image, each blocking on-device
  for a real ~1.15s inside a ~1.65s host-side timeout (a narrow ~500ms margin, 32 times per write). Flash
  writes instead go through avr_write_page() with delay=0 per chunk (its own write-cycle wait is folded
  into a single per-256-byte-page poll instead), so no individual flash transfer comes anywhere near
  timing out - flash's margin is generous everywhere, EEPROM's is tight on every single chunk.
- The critical asymmetry, confirmed directly in usbtiny.c: usb_in() (used for every read, and for flash's
  page-completion polling) auto-retries up to 10x on a failed USB transfer. usb_out() (used for every
  EEPROM write chunk) has zero retry logic - one dropped/timed-out transfer aborts the whole avrdude
  invocation immediately. Corroborated by a real, previously-filed avrdude issue with the identical
  usb_out()/128-byte-chunk failure signature on another usbtiny-based programmer:
  https://github.com/avrdudes/avrdude/issues/1210 (closed unresolved - a known but unfixed driver
  characteristic, not something a newer avrdude version addresses).
- Also confirmed by hardware testing, each ruling out an alternative explanation: running as root vs. a
  normal user gives identical failures (not a USB permissions issue); two different firmware builds
  flashed on the target chip give the identical failure/success pattern (not firmware-dependent - expected,
  since ISP programming holds the chip in reset throughout); a settle delay before writing (1s, 3s, and
  10s all tested) showed no meaningful effect.

**Connection quality does matter here, just not the way it first looked.** An earlier round of testing
initially ruled out "flaky USB connection" as the explanation, reasoning that a generically bad cable/port
should hurt large, long-running flash writes too, not spare them completely - and flash stayed 100%
reliable throughout. That reasoning was correct as far as it went, but incomplete: a marginal ISP USB cable
was later confirmed as a major real contributor after all, discovered when swapping it eliminated retries
entirely (0 of 8 real writes needed a retry afterward, vs. 3 of 4 immediately before). The resolution isn't
a contradiction - a marginal cable only ever has a chance to matter on the ~500ms-margin EEPROM path
described above; flash's generous per-chunk margin absorbs the same cable without issue. So the mechanism
above is the reason EEPROM is the *only* place a marginal connection can show up at all, not evidence that
connection quality is irrelevant. Given that, some of this driver's write failures may always come down to
cable/port/programmer quality on a given machine, which this tool has no way to detect or control for -
hence the mitigations below staying in place regardless of any one setup's actual cable quality.

Two mitigations, both real and independent of whatever mix of driver-level fragility and cable quality is
in play on any given machine: (1) a bounded retry around the whole avrdude write invocation, since
avrdude's own usb_in()-based operations already recover this way and a failed write's own internal verify
means a failure is always detected, never silently accepted - retrying the whole operation is safe. (2) a
hard subprocess-level timeout (AVRDUDE_TIMEOUT_SECONDS) on every avrdude call, read or write - this exists
because of a further confirmed finding: a run of failed writes can leave the *next* avrdude invocation -
even a plain EEPROM read, which otherwise has no reliability issue of its own (it uses usb_in() throughout,
confirmed from source, so already gets avrdude's own internal 10x retry) - hanging indefinitely, well below
avrdude's own timeout/retry logic, at the OS/USB level. Reproduced on real hardware multiple times,
including with no Python code involved at all (a raw shell loop of back-to-back `avrdude` write invocations
reproduced the same hang). Without _run_avrdude()'s subprocess timeout, that hang is completely unbounded -
the timeout converts it into a clean, immediate AvrdudeError instead. Importantly, this is *not* a lasting
hardware wedge: killing the hung process (whether via the timeout here, Ctrl+C, or a plain SIGTERM) is
sufficient to recover - a flash write and a plain EEPROM read both succeeded immediately right afterward in
testing, with nothing physically unplugged in between. A caller can just retry.

read_full_eeprom() does not need the retry treatment - only the subprocess timeout, since a read's own
usb_in()-based path has no observed reliability issue of its own, only the "hangs if the previous
operation was a failed write" risk that the timeout already covers for every call.

blank_eeprom_via_fuse_toggle() deliberately does NOT go through the eeprom:w path at all - a full
all-0xFF image write over this programmer hits exactly the fragility above. Instead it clears the HFUSE
EESAVE bit, does a chip erase (which then wipes EEPROM as a side effect, via avrdude's ordinary reliable
erase, not eeprom:w), and restores EESAVE. It also erases FLASH - the caller is expected to re-flash
afterward. Fuse writes and the chip erase are single, fast operations, well within AVRDUDE_TIMEOUT_SECONDS;
this module still never issues a flash *write* (that stays the Makefile's own $(AVRDUDE) path).

Mirrors src/Makefile's $(AVRDUDE) invocation (avrdude -P usb -c iseavrprog -p atmega1284p -B1) - if that
Makefile constant ever changes (different programmer, different part), update AVRDUDE_ARGS below to match.
AVRDUDE_SLOW_ARGS mirrors the Makefile's $(AVRDUDE_SLOW) (-B32) used for its own fuse target.
"""

import os
import subprocess
import tempfile
import time

import cst_eeprom_layout as layout

_AVRDUDE_BASE = ["avrdude", "-P", "usb", "-c", "iseavrprog", "-p", "atmega1284p"]
AVRDUDE_ARGS = _AVRDUDE_BASE + ["-B1"]
# Slower SPI bit clock for fuse reads/writes and the chip erase - mirrors src/Makefile's $(AVRDUDE_SLOW).
AVRDUDE_SLOW_ARGS = _AVRDUDE_BASE + ["-B32"]

# HFUSE values for the ATmega1284P, from src/Makefile: 0xD1 is the committed value (EESAVE programmed, so
# EEPROM survives a chip erase); 0xD9 is the Makefile's own commented-out "Erase EEPROM after programming"
# value (EESAVE unprogrammed). blank_eeprom_via_fuse_toggle() flips to 0xD9, erases, flips back.
HFUSE_EESAVE_ON = 0xD1
HFUSE_EESAVE_OFF = 0xD9

# Hard ceiling on any single avrdude invocation - see _run_avrdude_cmd()'s try/except, and the module
# docstring, for why this exists at all (a wedged USB connection can hang below avrdude's own
# timeout/retry logic, with no ceiling of its own). This module never invokes a flash write (that's a
# separate path, the Makefile's own $(AVRDUDE)); it does now issue fuse reads/writes and a chip erase
# (all sub-second), but the longest legitimate operation here is still write_full_eeprom()'s full 32-chunk
# image write, observed at ~37-40s when successful. 90s leaves a comfortable margin above that even if
# avrdude's own internal per-chunk SPI retries add real delay before eventually succeeding, while staying
# far below an actual hang (confirmed on real hardware to exceed 120s with zero progress).
AVRDUDE_TIMEOUT_SECONDS = 90

# How many whole-write attempts to make before giving up, and the delay between them (only between
# attempts, never before the first - a pre-write settle delay showed no meaningful effect in testing).
# Raised from 3 to 5 after real-hardware testing hit a flaky connection needing more than 3 attempts
# within a single run (requiring a manual re-invocation) - cheap to raise, since a failing attempt fails
# within a couple of seconds rather than running anywhere near AVRDUDE_TIMEOUT_SECONDS.
WRITE_MAX_ATTEMPTS = 5
WRITE_RETRY_DELAY_SECONDS = 1.0


class AvrdudeError(RuntimeError):
    pass


class AvrdudeWriteFailedError(AvrdudeError):
    """Every write_full_eeprom() attempt failed. `.attempts` holds each attempt's AvrdudeError in order,
    in case a caller wants more than just the last one (str(self) already reports on the last attempt)."""

    def __init__(self, attempts):
        self.attempts = attempts
        super().__init__(
            "EEPROM write failed after %d attempt(s); last error:\n%s" % (len(attempts), attempts[-1])
        )


class EepromWipeError(AvrdudeError):
    """blank_eeprom_via_fuse_toggle() failed. Message says whether EESAVE was left cleared (urgent - the
    next `make flash` would then wipe the throttle's config) or whether the chip erase / verify failed
    after EESAVE was safely restored."""


def _run_avrdude_cmd(extra_args, base=None):
    cmd = list(base or AVRDUDE_ARGS) + list(extra_args)
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=AVRDUDE_TIMEOUT_SECONDS)
    except FileNotFoundError:
        raise AvrdudeError(
            "%s not found on PATH - install it (see src/cst-cfgtransfer/README.md) or put it on PATH."
            % AVRDUDE_ARGS[0]
        )
    except subprocess.TimeoutExpired:
        # Confirmed on real hardware: a run of failed writes (each already showing avrdude's own
        # "Operation timed out"/"Input/output error" - the driver-level failure documented above) can
        # leave the *next* avrdude invocation - even a plain read - hanging indefinitely below avrdude's
        # own timeout/retry logic, at the OS/USB level. Without a hard ceiling here, that hang is
        # unbounded. Also confirmed: this is not a lasting hardware wedge needing a physical power-cycle
        # - a plain flash write and a plain EEPROM read both succeeded immediately right after killing a
        # hung process this way, with no cable unplugged in between. Simply retrying is enough; see
        # AVRDUDE_TIMEOUT_SECONDS above for why 90s is the chosen ceiling.
        raise AvrdudeError(
            "avrdude did not respond within %ds and was killed.\ncommand: %s\nThis is usually recoverable "
            "without unplugging anything - retry the operation." % (AVRDUDE_TIMEOUT_SECONDS, " ".join(cmd))
        )
    if result.returncode != 0:
        raise AvrdudeError(
            "avrdude failed (exit %d)\ncommand: %s\nstdout:\n%s\nstderr:\n%s"
            % (result.returncode, " ".join(cmd), result.stdout, result.stderr)
        )
    return result


def _run_avrdude(memop):
    """One `-U <memop>` operation at the normal bit clock - the shape every pre-existing caller uses."""
    return _run_avrdude_cmd(["-U", memop])


def _run_avrdude_with_retries(memop, on_retry=None):
    attempts = []
    for attempt in range(1, WRITE_MAX_ATTEMPTS + 1):
        try:
            return _run_avrdude(memop)
        except AvrdudeError as e:
            attempts.append(e)
            if attempt < WRITE_MAX_ATTEMPTS:
                if on_retry is not None:
                    on_retry(attempt, WRITE_MAX_ATTEMPTS, e)
                time.sleep(WRITE_RETRY_DELAY_SECONDS)
    raise AvrdudeWriteFailedError(attempts)


def read_full_eeprom():
    """Reads the whole EEPROM region from the connected chip over ISP. Returns EEPROM_SIZE bytes."""
    fd, tmp_path = tempfile.mkstemp(suffix=".bin")
    os.close(fd)
    try:
        _run_avrdude("eeprom:r:%s:r" % tmp_path)
        with open(tmp_path, "rb") as f:
            data = f.read()
    finally:
        os.unlink(tmp_path)
    if len(data) != layout.EEPROM_SIZE:
        raise AvrdudeError("expected a %d-byte EEPROM read, got %d bytes"
                            % (layout.EEPROM_SIZE, len(data)))
    return data


def write_full_eeprom(data, on_retry=None):
    """Writes the entire `data` (must be exactly EEPROM_SIZE bytes) to the connected chip's EEPROM over
    ISP - see the module docstring for why this always writes the whole image rather than just a changed
    sub-range.

    `on_retry(attempt, max_attempts, exc)`, if given, is called right before each retry (not after the
    final failed attempt) - a real write attempt can take tens of seconds, so a silent multi-attempt retry
    would look like a hang to an interactive caller.

    Raises AvrdudeWriteFailedError if every attempt fails.
    """
    if len(data) != layout.EEPROM_SIZE:
        raise ValueError("expected exactly %d bytes, got %d" % (layout.EEPROM_SIZE, len(data)))
    fd, tmp_path = tempfile.mkstemp(suffix=".bin")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(data)
        _run_avrdude_with_retries("eeprom:w:%s:r" % tmp_path, on_retry=on_retry)
    finally:
        os.unlink(tmp_path)


def read_fuses():
    """Returns (lfuse, hfuse, efuse) as ints. Fuse reads are cheap and not subject to the eeprom:w
    fragility, but use the slower bit clock anyway, matching the Makefile's own fuse handling."""
    result = _run_avrdude_cmd(
        ["-U", "lfuse:r:-:h", "-U", "hfuse:r:-:h", "-U", "efuse:r:-:h"], base=AVRDUDE_SLOW_ARGS)
    values = [int(line.strip(), 16) for line in result.stdout.splitlines()
              if line.strip().startswith("0x")]
    if len(values) != 3:
        raise AvrdudeError("expected 3 fuse values from avrdude, got %r\n%s" % (values, result.stdout))
    return tuple(values)


def write_hfuse(value):
    """Writes HFUSE (avrdude verifies the write itself; a non-zero exit -> AvrdudeError)."""
    _run_avrdude_cmd(["-U", "hfuse:w:0x%02X:m" % value], base=AVRDUDE_SLOW_ARGS)


def chip_erase():
    """`avrdude -e` - erases flash AND (only while HFUSE EESAVE is unprogrammed) EEPROM."""
    _run_avrdude_cmd(["-e"], base=AVRDUDE_SLOW_ARGS)


def blank_eeprom_via_fuse_toggle(on_progress=None):
    """Factory-blank the connected chip's EEPROM - every byte to 0xFF - without ever touching the flaky
    eeprom:w path (see the module docstring). Clears the HFUSE EESAVE bit, does a chip erase (which then
    wipes EEPROM as a side effect), and restores HFUSE to whatever it was.

    This also erases FLASH: the throttle has no firmware afterward until it is re-flashed.

    `on_progress(str)`, if given, is called with a short line before each step.

    Raises EepromWipeError - and the caller MUST surface it - if the HFUSE restore fails (EESAVE is then
    left cleared, so an ordinary `make flash` would wipe the throttle's config), or if the chip erase
    fails (EESAVE is restored first).

    The two verification steps at the end are best-effort: EESAVE-off + chip erase blanks EEPROM as a
    hardware guarantee, and both HFUSE writes are self-verified by avrdude, so a *readback* that fails
    only because the ISP link is marginal is reported as a warning via on_progress(), not a failure. A
    readback that succeeds and finds a non-0xFF byte (or the wrong HFUSE) is still a hard EepromWipeError.
    """
    def progress(msg):
        if on_progress is not None:
            on_progress(msg)

    original_hfuse = read_fuses()[1]

    progress("clearing HFUSE EESAVE bit (0x%02X -> 0x%02X)" % (original_hfuse, HFUSE_EESAVE_OFF))
    write_hfuse(HFUSE_EESAVE_OFF)   # if this raises, EESAVE was not changed - nothing to undo

    erase_error = None
    try:
        progress("chip erase")
        chip_erase()
    except AvrdudeError as exc:
        erase_error = exc

    progress("restoring HFUSE (0x%02X -> 0x%02X)" % (HFUSE_EESAVE_OFF, original_hfuse))
    try:
        write_hfuse(original_hfuse)
    except AvrdudeError as exc:
        raise EepromWipeError(
            "HFUSE EESAVE bit is LEFT CLEARED - the restore write failed. Until you fix this, the next "
            "`make flash` will wipe the throttle's config. Restore it manually:\n"
            "    avrdude -P usb -c iseavrprog -p atmega1284p -B32 -U hfuse:w:0x%02X:m\n\n%s"
            % (original_hfuse, exc))

    if erase_error is not None:
        raise EepromWipeError("chip erase failed (HFUSE has been restored, no harm done):\n\n%s"
                               % erase_error)

    progress("verifying")
    try:
        hfuse_now = read_fuses()[1]
        data = read_full_eeprom()
    except AvrdudeError as exc:
        progress("could not read back to verify (ISP link) - the erase and HFUSE restore both "
                  "completed and were self-verified, so this is almost certainly fine: %s" % exc)
        return
    if hfuse_now != original_hfuse:
        raise EepromWipeError("HFUSE did not read back as 0x%02X after the restore (got 0x%02X)"
                               % (original_hfuse, hfuse_now))
    first_nonblank = next((i for i, b in enumerate(data) if b != 0xFF), None)
    if first_nonblank is not None:
        raise EepromWipeError("EEPROM is not blank after the erase - byte 0x%04X reads 0x%02X"
                               % (first_nonblank, data[first_nonblank]))
