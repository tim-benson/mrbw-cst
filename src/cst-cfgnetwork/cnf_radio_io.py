# Copyright (C) 2026 Tim Benson <blw@east-slope.com>
# License: GNU General Public License v3 (see LICENSE)

"""MRBus/MRBee radio transport for mrbw-cabbus's shared CNF store, speaking the 'C' (push)/'D' (pull)
protocol a real throttle uses for its on-device SAVE CNF > SH:NET / LOAD CNF > SH:NET menu actions.

This is the radio equivalent of cst-cfgtransfer's avrdude_io.py - where that module shells out to
avrdude over ISP, this one frames MRBus packets inside Digi XBee API frames and exchanges them with a
cabbus over a USB-attached XBee radio (pyserial).

Protocol reference (both already fully read and cross-checked against each other while designing this):
  - src/cst-sync.c / src/cst-sync.h (this repo) - the throttle-side client this module mirrors.
  - cnf-store.c / cnf-store.h / cabbus-eeprom.h (mrbw-cabbus repo) - the server side.
The exact byte layouts and timeout/retry constants below come from reading both files in full; see
mrbw-cst's CLAUDE.md ("Shared network CNF store") for the human-readable design writeup.

`pyserial` is only imported lazily, inside CnfRadioLink.__init__'s real-port branch - the framing/CRC/
protocol-state-machine code below has no hardware dependency at all and is fully unit-testable (see
tests/test_cnf_radio_io.py) against a fake serial object, without pyserial installed.
"""

import time

# --- XBee API frame constants ---

XBEE_START = 0x7E
XBEE_ESCAPE = 0x7D
XBEE_ESCAPE_XOR = 0x20
XBEE_ESCAPED_BYTES = frozenset((0x7E, 0x7D, 0x11, 0x13))

XBEE_API_TX_16BIT = 0x01   # "Transmit Request, 16-bit address" - what we send
XBEE_API_RX_16BIT = 0x81   # "RX Packet, 16-bit Address" - what mrbeeTransmit()'s broadcast TX arrives as
XBEE_API_RX_64BIT = 0x80   # "RX Packet, 64-bit Address" - some XBee firmware/config combos use this instead

# --- MRBus packet layout (mrbus-constants.h) ---

MRBUS_HEADER_LEN = 6  # dest, src, len, crcL, crcH, type - payload (incl. this protocol's subtype byte)
                        # starts right after this

# --- MRBus CRC16 (mrbus-crc.c, ported faithfully from the already hardware-validated Python port in
# src/eep-test/mrbus.py's mrbusCRC16Update/mrbusCRC16Calculate - same table values, same bit-twiddling
# order, just cleaned up for Python 3 / this module's naming) ---

_CRC16_HIGH_TABLE = (0x00, 0xA0, 0xE0, 0x40, 0x60, 0xC0, 0x80, 0x20,
                      0xC0, 0x60, 0x20, 0x80, 0xA0, 0x00, 0x40, 0xE0)
_CRC16_LOW_TABLE = (0x00, 0x01, 0x03, 0x02, 0x07, 0x06, 0x04, 0x05,
                     0x0E, 0x0F, 0x0D, 0x0C, 0x09, 0x08, 0x0A, 0x0B)


def mrbus_crc16_update(crc, byte):
    crc_h = (crc >> 8) & 0xFF
    crc_l = crc & 0xFF
    for i in range(2):
        if i != 0:
            w = ((crc_h << 4) & 0xF0) | ((crc_h >> 4) & 0x0F)
            t = (w ^ byte) & 0x0F
        else:
            t = (crc_h ^ byte) & 0xF0
            t = ((t << 4) & 0xF0) | ((t >> 4) & 0x0F)
        crc_h = (crc_h << 4) & 0xFF
        crc_h = crc_h | (crc_l >> 4)
        crc_l = (crc_l << 4) & 0xFF
        crc_h = crc_h ^ _CRC16_HIGH_TABLE[t]
        crc_l = crc_l ^ _CRC16_LOW_TABLE[t]
    return (crc_h << 8) | crc_l


def mrbus_packet_crc16(packet_bytes):
    """CRC16 over a full MRBus packet (dest..end, per packet_bytes[2]'s own length byte), skipping the
    CRC field itself (indices 3,4) - matches mrbusCRC16Calculate()/the firmware's packet-level CRC."""
    length = packet_bytes[2]
    crc = 0
    for i in range(length):
        if i in (3, 4):
            continue
        crc = mrbus_crc16_update(crc, packet_bytes[i])
    return crc


def payload_crc16(data):
    """Plain CRC16 fold over raw bytes, no skipped indices - matches cst-sync.c's cnfCrc16()/
    cnf-store.c's cnfCrc16(), used for the 128-byte shared-CNF payload checksum (COMMIT / pull verify)."""
    crc = 0
    for b in data:
        crc = mrbus_crc16_update(crc, b)
    return crc


# --- XBee framing ---

def _escape(raw):
    """Byte-stuff everything except the leading start delimiter, per XBee API escaping."""
    out = bytearray((raw[0],))
    for b in raw[1:]:
        if b in XBEE_ESCAPED_BYTES:
            out.append(XBEE_ESCAPE)
            out.append(b ^ XBEE_ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def build_xbee_frame(mrbus_packet_bytes):
    """Wraps a full MRBus packet (dest..payload) in a "Transmit Request, 16-bit address" XBee API frame,
    broadcast to 0xFFFF (matching mrbeeTransmit()'s own behavior - MRBus-level addressing is handled by
    the dest/src bytes inside the packet, not the XBee network layer), and escapes it. Returns the exact
    bytes to write to the serial port."""
    frame_data = bytearray()
    frame_data.append(XBEE_API_TX_16BIT)
    frame_data.append(0x00)   # frame ID
    frame_data.append(0xFF)   # dest addr MSB - broadcast
    frame_data.append(0xFF)   # dest addr LSB - broadcast
    frame_data.append(0x00)   # transmit options
    frame_data.extend(mrbus_packet_bytes)

    length = len(frame_data)
    checksum = (0xFF - (sum(frame_data) & 0xFF)) & 0xFF

    raw = bytearray()
    raw.append(XBEE_START)
    raw.append((length >> 8) & 0xFF)
    raw.append(length & 0xFF)
    raw.extend(frame_data)
    raw.append(checksum)
    return _escape(raw)


class _XBeeFrameReceiver:
    """Incremental, byte-at-a-time XBee API frame unescaper/assembler - mirrors the firmware's own RX
    ISR (mrbee-avr.c) and eep-test/mrbus.py's mrbeeSimple.getpkt() state machine. A corrupt frame (bad
    XBee-level checksum) is silently dropped and the receiver resyncs on the next start delimiter, same
    as both of those reference implementations - this is a shared radio channel, and a garbled frame is
    expected to happen occasionally, not something to raise an exception over."""

    def __init__(self):
        self._reset()

    def _reset(self):
        self._buf = bytearray()
        self._escape_next = False
        self._active = False
        self._expected_len = None

    def feed_byte(self, byte):
        """Returns the inner frame (API-ID byte onward, unescaped, XBee checksum stripped and verified)
        once a complete frame is assembled, else None."""
        if byte == XBEE_START:
            self._buf = bytearray((byte,))
            self._escape_next = False
            self._active = True
            self._expected_len = None
            return None
        if byte == XBEE_ESCAPE:
            self._escape_next = True
            return None
        if not self._active:
            return None
        if self._escape_next:
            byte = byte ^ XBEE_ESCAPE_XOR
            self._escape_next = False

        self._buf.append(byte)
        if len(self._buf) == 3:
            self._expected_len = self._buf[1] * 256 + self._buf[2] + 4
        if self._expected_len is not None and len(self._buf) == self._expected_len:
            frame = bytes(self._buf)
            self._active = False
            if (sum(frame[3:]) & 0xFF) != 0xFF:
                return None  # bad checksum, drop
            return frame[3:-1]
        return None


def _decode_inner_frame(frame):
    """frame: API-ID byte onward (see _XBeeFrameReceiver). Returns (dest, src, type_byte, payload) for a
    recognized+CRC-valid RX frame carrying an MRBus packet, else None. Unlike eep-test/mrbus.py's
    getpkt(), this verifies the MRBus-level CRC16 too, not just the XBee framing checksum - a reply
    should be rejected as corrupt rather than trusted blind."""
    api_id = frame[0]
    if api_id == XBEE_API_RX_16BIT:
        rf_data = frame[5:]     # 1 (API ID) + 2 (16-bit src) + 1 (RSSI) + 1 (options)
    elif api_id == XBEE_API_RX_64BIT:
        rf_data = frame[11:]    # 1 (API ID) + 8 (64-bit src) + 1 (RSSI) + 1 (options)
    else:
        return None             # some other API frame (AT response, etc.) - not our concern here

    if len(rf_data) < MRBUS_HEADER_LEN or rf_data[2] != len(rf_data):
        return None
    crc_rx = rf_data[3] | (rf_data[4] << 8)
    if mrbus_packet_crc16(rf_data) != crc_rx:
        return None
    return rf_data[0], rf_data[1], rf_data[5], bytes(rf_data[6:])


# --- Shared-CNF protocol constants (cst-sync.c / cnf-store.c) ---

PKT_TYPE_PUSH_REQ = ord('C')
PKT_TYPE_PUSH_ACK = ord('c')
PKT_TYPE_PULL_REQ = ord('D')
PKT_TYPE_PULL_ACK = ord('d')

SUBTYPE_BEGIN = 0x00
SUBTYPE_DATA = 0x01
SUBTYPE_COMMIT = 0x02   # push only
SUBTYPE_DONE = 0x03     # pull only
SUBTYPE_RESET = 0x04    # push type only - whole-table wipe, no entry index

STATUS_OK = 0x00
STATUS_BUSY = 0x01
STATUS_BAD_ENTRY = 0x02
STATUS_EMPTY = 0x03
STATUS_CHECKSUM_FAIL = 0x04
STATUS_NOT_IN_PROGRESS = 0x05
STATUS_VERSION_MISMATCH = 0x06  # push BEGIN: layout_version is strictly lower than the table's pin

CNF_VERSION_UNSET = 0xFF  # table-wide pin's "never pinned yet" sentinel (cabbus-eeprom.h's CNF_VERSION_UNSET)

CNF_CHUNK_MAX_BYTES = 10
# Per-entry storage reservation on cabbus (cabbus-eeprom.h's CNF_PAYLOAD_SIZE) - a sanity ceiling on any
# single payload, not the actual transfer length, which is now derived from the pushed/pulled data itself
# (see push_entry()/pull_entry() below) so a future mrbw-cst format growth needs no change here.
CNF_PAYLOAD_MAX_SIZE = 192
CNF_ENTRY_COUNT = 20

CNF_CHUNK_TIMEOUT_S = 0.3
CNF_CHUNK_MAX_RETRIES = 3
CNF_COMMIT_TIMEOUT_S = 0.8
CNF_OVERALL_TIMEOUT_S = 15.0
CNF_POLL_INTERVAL_S = 0.02

# Generic MRBus presence ping, not part of the 'C'/'D' protocol - used only by discover_nodes().
PING_TYPE_REQ = ord('A')
PING_TYPE_ACK = ord('a')
PING_REPEAT_INTERVAL_S = 0.3

# --- CST throttle status packet ('S') - not part of the 'C'/'D' protocol; decoded purely as a passive,
# read-only diagnostic by cst_cfgnetwork.py's `sniff` subcommand. Payload layout mirrors mrbw-cst.c's own
# status-packet builder: [loco hi][loco lo][speed|dir][function mask F0-F28, big-endian x4][throttle
# status][battery decivolts]. If that firmware-side layout ever changes, update decode_cst_status(). ---
CST_STATUS_TYPE = ord('S')

THROTTLE_STATUS_EMERGENCY = 0x01
THROTTLE_STATUS_ALL_STOP = 0x02
THROTTLE_STATUS_ALERTER = 0x40
THROTTLE_STATUS_SLEEP = 0x80


def decode_cst_status(payload):
    """Decode a CST 'S' status packet's payload (the bytes after the MRBus type byte) into a dict, or
    return None if it is not at least the 9 bytes mrbw-cst.c sends. `loco_raw` is the 16-bit
    loco-address word verbatim (short-address bit included) - the caller decodes that, so this module
    stays free of any EEPROM-layout dependency. `speed_step` is 0 idle / 1 e-stop / else the raw DCC
    speed step; bit 7 of that same byte is direction."""
    if len(payload) < 9:
        return None
    speed_byte = payload[2]
    return {
        "loco_raw": (payload[0] << 8) | payload[1],
        "forward": bool(speed_byte & 0x80),
        "speed_step": speed_byte & 0x7F,
        "function_mask": (payload[3] << 24) | (payload[4] << 16) | (payload[5] << 8) | payload[6],
        "throttle_status": payload[7],
        "battery_decivolts": payload[8],
    }


# --- shared-CNF 'C'/'c'/'D'/'d' packet decode - a passive, read-only diagnostic for the `sniff --cnf`
# subcommand, entirely separate from the transfer client below. Byte layouts mirror push_entry() /
# pull_entry() (the hardware-validated spec). Best-effort: a short payload or an unknown subtype falls
# back to a hex dump, never raises. ---
CNF_PACKET_TYPES = frozenset((PKT_TYPE_PUSH_REQ, PKT_TYPE_PUSH_ACK, PKT_TYPE_PULL_REQ, PKT_TYPE_PULL_ACK))

_CNF_SUBTYPE_NAMES = {
    SUBTYPE_BEGIN: "BEGIN", SUBTYPE_DATA: "DATA", SUBTYPE_COMMIT: "COMMIT",
    SUBTYPE_DONE: "DONE", SUBTYPE_RESET: "RESET",
}
_CNF_STATUS_NAMES = {
    STATUS_OK: "OK", STATUS_BUSY: "BUSY", STATUS_BAD_ENTRY: "BAD_ENTRY", STATUS_EMPTY: "EMPTY",
    STATUS_CHECKSUM_FAIL: "CRC_FAIL", STATUS_NOT_IN_PROGRESS: "NOT_IN_PROGRESS",
    STATUS_VERSION_MISMATCH: "VERSION_MISMATCH",
}


def _cnf_status_name(b):
    return _CNF_STATUS_NAMES.get(b, "status?0x%02X" % b)


def format_cnf_packet(type_byte, payload):
    """One-line human decode of a 'C'/'c'/'D'/'d' shared-CNF packet for `sniff --cnf`
    (`payload` = the bytes after the MRBus type byte, i.e. payload[0] is the subtype). Returns a
    string; falls back to a hex dump on a short/empty payload or an unrecognized subtype."""
    t = chr(type_byte)
    hexdump = " ".join("%02X" % b for b in payload)
    if not payload:
        return "'%s'  (empty)" % t
    is_ack = type_byte in (PKT_TYPE_PUSH_ACK, PKT_TYPE_PULL_ACK)
    is_push = type_byte in (PKT_TYPE_PUSH_REQ, PKT_TYPE_PUSH_ACK)
    fam = "PUSH" if is_push else "PULL"
    sub = payload[0]
    subname = _CNF_SUBTYPE_NAMES.get(sub)
    if subname is None:
        return "%s '%s'  subtype?0x%02X  %s" % (fam, t, sub, hexdump)
    tag = "%s %s%s" % (fam, subname, "-ACK" if is_ack else "")

    def entry(i):
        return "N%02d" % (payload[i] + 1) if i < len(payload) else "N??"

    try:
        if sub == SUBTYPE_RESET:
            return "%s  %s" % (tag, _cnf_status_name(payload[2])) if is_ack else "%s  (whole-table wipe)" % tag
        if sub == SUBTYPE_BEGIN:
            if not is_ack:
                if is_push:                                   # [00][entry][total_len][layout_ver]
                    return "%s  %s  len=%d ver=%d" % (tag, entry(1), payload[2], payload[3])
                return "%s  %s" % (tag, entry(1))             # pull req [00][entry]
            if is_push:                                        # [00][entry][status]
                return "%s  %s  %s" % (tag, entry(1), _cnf_status_name(payload[2]))
            crc = payload[4] | (payload[5] << 8)               # pull ack [00][entry][status][len][crcL][crcH][ver]
            return "%s  %s  %s  len=%d crc=0x%04X ver=%d" % (
                tag, entry(1), _cnf_status_name(payload[2]), payload[3], crc, payload[6])
        if sub == SUBTYPE_DATA:
            if not is_ack:                                     # push [01][entry][off][len][chunk..]  pull req [01][entry][off][len]
                base = "%s  %s  off=%d len=%d" % (tag, entry(1), payload[2], payload[3])
                chunk = payload[4:]
                return base + (("  " + " ".join("%02X" % b for b in chunk)) if chunk else "  (request)")
            base = "%s  %s  off=%d  %s" % (tag, entry(1), payload[2], _cnf_status_name(payload[3]))
            chunk = payload[4:]                                # pull ack carries the chunk; push ack does not
            return base + (("  " + " ".join("%02X" % b for b in chunk)) if chunk else "")
        if sub == SUBTYPE_COMMIT:
            if not is_ack:                                     # [02][entry][crcL][crcH]
                return "%s  %s  crc=0x%04X" % (tag, entry(1), payload[2] | (payload[3] << 8))
            return "%s  %s  %s" % (tag, entry(1), _cnf_status_name(payload[2]))
        if sub == SUBTYPE_DONE:                                # pull only [03][entry], no reply
            return "%s  %s" % (tag, entry(1))
    except IndexError:
        pass
    return "%s  %s" % (tag, hexdump)

# Base-station (cabbus-class) address range, mirroring mrbw-cst.c's own MRBUS_BASE_ADDR_MIN/MAX convention
# - mrbw-cabbus's own device address is a 5-bit DIP field added to this base (0xD0 + (sw2 & 0x1F)), so this
# range is small and fully enumerable, which is what makes the unicast sweep below practical.
MRBUS_BASE_ADDR_MIN = 0xD0
MRBUS_BASE_ADDR_MAX = 0xEF

# Per-address timing for the unicast sweep phase of discover_nodes() - see that method's docstring for why
# a sweep is needed at all (mrbw-cabbus deliberately drops broadcast packets, so the plain broadcast ping
# above can never find one).
SWEEP_PROBE_TIMEOUT_S = 0.2
SWEEP_PROBE_RETRIES = 1


class CnfRadioError(RuntimeError):
    pass


class CnfBusyError(CnfRadioError):
    pass


class CnfEmptyError(CnfRadioError):
    pass


class CnfBadEntryError(CnfRadioError):
    pass


class CnfChecksumError(CnfRadioError):
    pass


class CnfVersionMismatchError(CnfRadioError):
    """Raised only when cabbus's push BEGIN ack reports STATUS_VERSION_MISMATCH - this link's
    layout_version is strictly lower than the table's currently pinned version, so this push was refused
    before any DATA traffic. No pull equivalent: cabbus never refuses a pull based on version, it just
    reports the pinned value (see pull_entry()'s returned layout_version) - deciding whether to trust it
    is the caller's job (cst_cfgnetwork.py)."""
    pass


class CnfTimeoutError(CnfRadioError):
    def __init__(self, step):
        self.step = step
        super().__init__("timed out waiting for a %s reply" % step)


def _raise_for_status(status):
    if status == STATUS_OK:
        return
    if status == STATUS_BUSY:
        raise CnfBusyError("the ProtoThrottle Receiver is busy with a different in-progress transfer")
    if status == STATUS_EMPTY:
        raise CnfEmptyError("this shared network entry has never been saved to")
    if status == STATUS_CHECKSUM_FAIL:
        raise CnfChecksumError("the ProtoThrottle Receiver rejected COMMIT - the CRC16 it computed over "
                                "the received payload did not match ours")
    if status == STATUS_BAD_ENTRY:
        raise CnfBadEntryError("the ProtoThrottle Receiver reported a malformed request (bad entry "
                                "index?)")
    if status == STATUS_NOT_IN_PROGRESS:
        raise CnfBadEntryError("the ProtoThrottle Receiver reports no transfer in progress (protocol got "
                                "out of sync)")
    if status == STATUS_VERSION_MISMATCH:
        # push_entry() special-cases this before calling _raise_for_status() (its message names the
        # actual layout_version involved) - this fallback only fires if STATUS_VERSION_MISMATCH is ever
        # seen from a call site that doesn't already handle it.
        raise CnfVersionMismatchError("the ProtoThrottle Receiver refused: this CNF format version is "
                                       "older than the version pinned on the table")
    raise CnfRadioError("unrecognized status byte 0x%02X" % status)


def _validate_entry(entry):
    if not (isinstance(entry, int) and not isinstance(entry, bool) and 0 <= entry < CNF_ENTRY_COUNT):
        raise CnfBadEntryError("entry must be 0-%d, got %r" % (CNF_ENTRY_COUNT - 1, entry))


class CnfRadioLink:
    """One MRBus/MRBee link to a specific cabbus, over a USB-attached XBee radio.

    Pass `serial_obj` (anything providing .read(n) and .write(data), non-blocking/short-timeout reads)
    to inject a fake/test transport - this is how tests/test_cnf_radio_io.py exercises the protocol
    state machine without any real hardware or pyserial installed. Real usage leaves it as None, which
    opens `port` via pyserial at the framing the firmware expects (115200 baud, 2 stop bits).
    """

    def __init__(self, port=None, my_addr=None, cabbus_addr=None, baud=115200, serial_obj=None):
        """cabbus_addr may be left as None if this link will only ever be used for discover_nodes() (the
        one operation that doesn't target a specific cabbus) - every 'C'/'D' protocol method below
        raises a clear ValueError up front if called without one, rather than silently timing out."""
        if my_addr is None:
            raise ValueError("my_addr is required")
        if serial_obj is not None:
            self._ser = serial_obj
        else:
            import serial  # lazy: only needed for a real port, see module docstring
            self._ser = serial.Serial(port, baud, timeout=0, rtscts=True, stopbits=serial.STOPBITS_TWO)
        self.my_addr = my_addr
        self.cabbus_addr = cabbus_addr
        self._rx = _XBeeFrameReceiver()

    def close(self):
        self._ser.close()

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # --- low level ---

    def send_raw(self, dest, type_byte, payload):
        """Send an arbitrary MRBus/MRBee packet - the generic low-level primitive other tools (e.g.
        cst-fastclock) can reuse without any dependency on the shared-CNF protocol below."""
        self._send_mrbus(dest, type_byte, payload)

    def _send_mrbus(self, dest, type_byte, payload):
        packet = bytearray()
        packet.append(dest & 0xFF)
        packet.append(self.my_addr & 0xFF)
        packet.append(MRBUS_HEADER_LEN + len(payload))
        packet.append(0)
        packet.append(0)
        packet.append(type_byte & 0xFF)
        packet.extend(payload)
        crc = mrbus_packet_crc16(packet)
        packet[3] = crc & 0xFF
        packet[4] = (crc >> 8) & 0xFF
        self._ser.write(build_xbee_frame(bytes(packet)))

    def _poll_once(self):
        packets = []
        chunk = self._ser.read(256)
        for byte in chunk:
            frame = self._rx.feed_byte(byte)
            if frame is None:
                continue
            pkt = _decode_inner_frame(frame)
            if pkt is not None:
                packets.append(pkt)
        return packets

    def _wait_for_matching(self, expect_type, match_fn, timeout_s):
        if timeout_s <= 0:
            return None
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            for dest, src, type_byte, payload in self._poll_once():
                if (src == self.cabbus_addr and dest in (self.my_addr, 0xFF)
                        and type_byte == expect_type and match_fn(payload)):
                    return payload
            time.sleep(CNF_POLL_INTERVAL_S)
        return None

    def _request_reply(self, req_type, req_payload, ack_type, match_fn, timeout_s, retries, deadline,
                        step_name):
        attempt = 0
        while True:
            self._send_mrbus(self.cabbus_addr, req_type, req_payload)
            remaining = deadline - time.monotonic()
            reply = self._wait_for_matching(ack_type, match_fn, min(timeout_s, remaining))
            if reply is not None:
                return reply
            attempt += 1
            if attempt > retries or time.monotonic() >= deadline:
                raise CnfTimeoutError(step_name)

    # --- shared-CNF protocol ---

    def _require_cabbus_addr(self):
        if self.cabbus_addr is None:
            raise ValueError("this CnfRadioLink has no cabbus_addr set - it can only be used for "
                              "discover_nodes()")

    def _pull_begin_raw(self, entry, deadline):
        """No-raise core of a pull BEGIN - returns (status, total_length, expected_crc, layout_version)
        regardless of status, since layout_version/total_length are table-wide pinned values reported
        even against a BUSY/BAD_ENTRY/EMPTY reply (see cnf-store.c's pull BEGIN handler). Callers that
        need the usual raise-on-error behavior should use _pull_begin() below instead."""
        payload = self._request_reply(
            PKT_TYPE_PULL_REQ, bytes((SUBTYPE_BEGIN, entry)),
            PKT_TYPE_PULL_ACK,
            lambda p: len(p) >= 7 and p[0] == SUBTYPE_BEGIN and p[1] == entry,
            CNF_CHUNK_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "pull BEGIN")
        status = payload[2]
        total_length = payload[3]
        expected_crc = payload[4] | (payload[5] << 8)
        layout_version = payload[6]
        return status, total_length, expected_crc, layout_version

    def _pull_begin(self, entry, deadline):
        """Returns (total_length, expected_crc, layout_version); raises the usual Cnf*Error on any
        non-OK status (including EMPTY - see peek_pinned_version() for the one caller that needs EMPTY
        to NOT raise)."""
        status, total_length, expected_crc, layout_version = self._pull_begin_raw(entry, deadline)
        _raise_for_status(status)
        return total_length, expected_crc, layout_version

    def _pull_data_chunk(self, entry, offset, length, deadline):
        payload = self._request_reply(
            PKT_TYPE_PULL_REQ, bytes((SUBTYPE_DATA, entry, offset, length)),
            PKT_TYPE_PULL_ACK,
            lambda p: len(p) >= 4 and p[0] == SUBTYPE_DATA and p[1] == entry and p[2] == offset,
            CNF_CHUNK_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "pull DATA@%d" % offset)
        _raise_for_status(payload[3])
        return bytes(payload[4:4 + length])

    def _pull_done(self, entry):
        # Fire-and-forget - cabbus sends no reply, this just lets it release its lock/snapshot promptly.
        self._send_mrbus(self.cabbus_addr, PKT_TYPE_PULL_REQ, bytes((SUBTYPE_DONE, entry)))

    def pull_entry(self, entry):
        """Pulls one shared-CNF entry's full payload (length is whatever's currently pinned - see
        _pull_begin()). Returns (payload_bytes, layout_version). Raises CnfEmptyError/CnfBusyError/
        CnfTimeoutError/CnfChecksumError as appropriate."""
        self._require_cabbus_addr()
        _validate_entry(entry)
        deadline = time.monotonic() + CNF_OVERALL_TIMEOUT_S
        total_length, expected_crc, layout_version = self._pull_begin(entry, deadline)
        if total_length > CNF_PAYLOAD_MAX_SIZE:
            raise CnfBadEntryError("the ProtoThrottle Receiver reported entry length %d, exceeding the "
                                    "%d-byte sanity ceiling" % (total_length, CNF_PAYLOAD_MAX_SIZE))

        data = bytearray(total_length)
        offset = 0
        while offset < total_length:
            length = min(CNF_CHUNK_MAX_BYTES, total_length - offset)
            chunk = self._pull_data_chunk(entry, offset, length, deadline)
            data[offset:offset + length] = chunk
            offset += length

        self._pull_done(entry)

        actual_crc = payload_crc16(bytes(data))
        if actual_crc != expected_crc:
            raise CnfChecksumError("pulled payload failed local CRC16 verification (got 0x%04X, the "
                                    "ProtoThrottle Receiver said 0x%04X)" % (actual_crc, expected_crc))
        return bytes(data), layout_version

    def peek_loco_address(self, entry):
        """Lightweight 2-byte peek at just an entry's loco address (BEGIN + DATA(0,2) + DONE), mirroring
        cst-sync.c's syncQuerySharedLocoAddress() - no whole-payload CRC involved, matching that
        function's own scope. Returns (address_bytes, layout_version). Raises the same exceptions as
        pull_entry() (except CnfChecksumError, which this read-only peek never raises)."""
        self._require_cabbus_addr()
        _validate_entry(entry)
        deadline = time.monotonic() + CNF_OVERALL_TIMEOUT_S
        _total_length, _expected_crc, layout_version = self._pull_begin(entry, deadline)
        chunk = self._pull_data_chunk(entry, 0, 2, deadline)
        self._pull_done(entry)
        return chunk, layout_version

    def peek_pinned_version(self, entry):
        """Lightweight peek at the table-wide pinned layoutVersion only (BEGIN + DONE, no DATA/CRC) -
        works even against a never-written entry, since the pin is table-wide, not per-entry (mirrors
        cst-sync.c's syncPeekSharedVersion()). Returns CNF_VERSION_UNSET (0xFF) if the table has never
        been pinned. Unlike pull_entry()/peek_loco_address(), an EMPTY entry does NOT raise here - the
        pin is still meaningful and returned regardless of this specific entry's own occupancy. Raises
        CnfBusyError/CnfBadEntryError/CnfTimeoutError."""
        self._require_cabbus_addr()
        _validate_entry(entry)
        deadline = time.monotonic() + CNF_OVERALL_TIMEOUT_S
        status, _total_length, _expected_crc, layout_version = self._pull_begin_raw(entry, deadline)
        self._pull_done(entry)
        if status not in (STATUS_OK, STATUS_EMPTY):
            _raise_for_status(status)
        return layout_version

    def push_entry(self, entry, payload, layout_version):
        """Pushes `payload` (any length up to CNF_PAYLOAD_MAX_SIZE - the actual length is derived from
        the payload itself, not a fixed constant, so a future mrbw-cst format growth needs no change
        here) to one shared-CNF entry, tagged with `layout_version` so cabbus's version guard can enforce
        the unset/equal/higher/lower rule (see mrbw-cabbus's cnf-store.c). Raises
        CnfVersionMismatchError if layout_version is strictly lower than the table's current pin (before
        any DATA traffic is sent), CnfChecksumError if cabbus's own CRC over what it received doesn't
        match (nothing is written on its end in that case), CnfBusyError/CnfTimeoutError as appropriate.
        """
        self._require_cabbus_addr()
        _validate_entry(entry)
        total_length = len(payload)
        if total_length > CNF_PAYLOAD_MAX_SIZE:
            raise ValueError("payload must be at most %d bytes, got %d"
                              % (CNF_PAYLOAD_MAX_SIZE, total_length))
        deadline = time.monotonic() + CNF_OVERALL_TIMEOUT_S

        begin_reply = self._request_reply(
            PKT_TYPE_PUSH_REQ, bytes((SUBTYPE_BEGIN, entry, total_length, layout_version)),
            PKT_TYPE_PUSH_ACK,
            lambda p: len(p) >= 3 and p[0] == SUBTYPE_BEGIN and p[1] == entry,
            CNF_CHUNK_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "push BEGIN")
        if begin_reply[2] == STATUS_VERSION_MISMATCH:
            raise CnfVersionMismatchError(
                "CNF format version %d is older than the version currently pinned on the table - check "
                "out (or flash the target device with) the mrbw-cst revision matching the firmware that "
                "pinned it before pushing here" % layout_version)
        _raise_for_status(begin_reply[2])

        offset = 0
        while offset < total_length:
            length = min(CNF_CHUNK_MAX_BYTES, total_length - offset)
            chunk = payload[offset:offset + length]
            data_reply = self._request_reply(
                PKT_TYPE_PUSH_REQ, bytes((SUBTYPE_DATA, entry, offset, length)) + chunk,
                PKT_TYPE_PUSH_ACK,
                lambda p, off=offset: len(p) >= 4 and p[0] == SUBTYPE_DATA and p[1] == entry and p[2] == off,
                CNF_CHUNK_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "push DATA@%d" % offset)
            _raise_for_status(data_reply[3])
            offset += length

        crc = payload_crc16(payload)
        commit_reply = self._request_reply(
            PKT_TYPE_PUSH_REQ, bytes((SUBTYPE_COMMIT, entry, crc & 0xFF, (crc >> 8) & 0xFF)),
            PKT_TYPE_PUSH_ACK,
            lambda p: len(p) >= 3 and p[0] == SUBTYPE_COMMIT and p[1] == entry,
            CNF_COMMIT_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "push COMMIT")
        _raise_for_status(commit_reply[2])

    def reset_table(self):
        """Sends SUBTYPE_RESET (a 'C' packet with no entry index) - wipes every shared entry and clears
        the table-wide version pin, independent of any version comparison. This is the deliberate,
        human-triggered reset path (cst_cfgnetwork.py's `reset-cabbus` subcommand), not part of the
        BEGIN/DATA/COMMIT transfer shape. Raises CnfBusyError if a transfer is currently in progress,
        CnfTimeoutError if cabbus never replies."""
        self._require_cabbus_addr()
        deadline = time.monotonic() + CNF_OVERALL_TIMEOUT_S
        reply = self._request_reply(
            PKT_TYPE_PUSH_REQ, bytes((SUBTYPE_RESET,)),
            PKT_TYPE_PUSH_ACK,
            lambda p: len(p) >= 3 and p[0] == SUBTYPE_RESET,
            CNF_CHUNK_TIMEOUT_S, CNF_CHUNK_MAX_RETRIES, deadline, "reset")
        _raise_for_status(reply[2])

    # --- discovery (not part of the 'C'/'D' protocol - a generic MRBus presence probe) ---

    def _broadcast_ping_sweep(self, wait_s):
        """Broadcasts a generic MRBus 'A' ping (dest=0xFF) and collects distinct source addresses that
        reply 'a' within wait_s. Only finds devices that actually honor a broadcast destination - real
        throttles do (mrbw-cst.c's PktHandler() explicitly accepts dest==0xFF), but mrbw-cabbus does NOT
        (see discover_nodes()'s docstring) - this alone can never find a cabbus."""
        found = set()
        deadline = time.monotonic() + wait_s
        last_ping = None  # time.monotonic()'s epoch is unspecified (can start near 0 for a short-lived
                           # process) - a numeric sentinel like 0.0 would risk "now - last_ping" staying
                           # under PING_REPEAT_INTERVAL_S and never firing the first ping at all
        while time.monotonic() < deadline:
            now = time.monotonic()
            if last_ping is None or now - last_ping >= PING_REPEAT_INTERVAL_S:
                self._send_mrbus(0xFF, PING_TYPE_REQ, b"")
                last_ping = now
            for dest, src, type_byte, _payload in self._poll_once():
                if type_byte == PING_TYPE_ACK and src != self.my_addr:
                    found.add(src)
            time.sleep(CNF_POLL_INTERVAL_S)
        return found

    def probe_address(self, addr, timeout_s, retries):
        """Unicasts an 'A' ping directly to one address (not broadcast) and returns True if that exact
        address replies 'a' within timeout_s, retrying up to `retries` additional times against RF packet
        loss. Used by the sweep phase below, one address at a time - a directed reply is proof the device
        is genuinely at that address, not just a heuristic based on which range the address falls in.
        Public (not `_`-prefixed) since it's also used by cst_cfgnetwork.py's `_explain_error()` to
        distinguish "device reachable but doesn't speak the shared-CNF protocol" from "no reply at all"
        after a BEGIN timeout."""
        for _attempt in range(retries + 1):
            self._send_mrbus(addr, PING_TYPE_REQ, b"")
            deadline = time.monotonic() + timeout_s
            while time.monotonic() < deadline:
                for dest, src, type_byte, _payload in self._poll_once():
                    if type_byte == PING_TYPE_ACK and src == addr:
                        return True
                time.sleep(CNF_POLL_INTERVAL_S)
        return False

    def discover_nodes(self, wait_s=2.0, sweep_base_addrs=True,
                        sweep_timeout_s=SWEEP_PROBE_TIMEOUT_S, sweep_retries=SWEEP_PROBE_RETRIES):
        """Finds MRBus nodes on the network. One-time setup helper only - does not feed into any other
        method here; the caller decides what to do with what's found.

        Two mechanisms, because one alone misses real devices:
        - A broadcast 'A' ping (dest=0xFF), which catches anything that honors broadcast - throttles do.
        - A unicast sweep of the whole base-station address range (0xD0-0xEF, 32 addresses - small enough
          to fully enumerate, since mrbw-cabbus's own address is just a 5-bit DIP field added to 0xD0),
          pinging each address individually. This exists because **mrbw-cabbus deliberately drops every
          broadcast packet** (`mrbw-cabbus.c`'s `PktHandler()`: "Also ignore broadcast packets, which is
          a little unconventional for MRBus nodes... helps avoid unrelated MRBus packets from causing
          havoc here") - the broadcast ping alone can *never* find a cabbus, confirmed on real hardware.
          Deliberately does not stop at the first address that answers: a layout can have more than one
          cabbus, and this sweeps the entire range regardless of earlier hits.

        Returns a list of (address, method) tuples sorted by address, where method is "broadcast" or
        "sweep" - a "sweep" hit is a directly-addressed reply (strong positive: this exact device is at
        this exact address), while "broadcast" just means something answered a general ping. If an address
        answers both ways, it's reported once as "sweep" (the stronger signal). Pass sweep_base_addrs=False
        to skip the sweep phase and only do the (much faster) broadcast ping.
        """
        broadcast_found = self._broadcast_ping_sweep(wait_s)

        sweep_found = set()
        if sweep_base_addrs:
            for addr in range(MRBUS_BASE_ADDR_MIN, MRBUS_BASE_ADDR_MAX + 1):
                if addr == self.my_addr:
                    continue
                if self.probe_address(addr, sweep_timeout_s, sweep_retries):
                    sweep_found.add(addr)

        results = []
        for addr in sorted(broadcast_found | sweep_found):
            results.append((addr, "sweep" if addr in sweep_found else "broadcast"))
        return results

    # --- passive sniff (read-only; not part of any protocol) ---

    def sniff(self, packet_types=None, stop_after_s=None):
        """Yield every CRC-valid MRBus packet seen on the radio as (dest, src, type_byte, payload) -
        the same shape _poll_once() produces. Purely passive: never transmits. Iterates until the
        caller stops (break / KeyboardInterrupt), or, if stop_after_s is given, until that many
        seconds have elapsed. `packet_types`, if given, is a container of MRBus type bytes to yield;
        every other type is skipped. Decoding a CST 'S' status packet's payload is decode_cst_status()."""
        deadline = None if stop_after_s is None else time.monotonic() + stop_after_s
        while deadline is None or time.monotonic() < deadline:
            for pkt in self._poll_once():
                if packet_types is None or pkt[2] in packet_types:
                    yield pkt
            time.sleep(CNF_POLL_INTERVAL_S)
