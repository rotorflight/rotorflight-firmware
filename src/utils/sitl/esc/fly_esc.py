#!/usr/bin/env python3
"""A FlyRotor ESC as bytes -- the fourth `rrfsm` protocol, and the only paged one.

    from fly_esc import FlyRotor
    esc = FlyRotor()

Like the Scorpion, a protocol the firmware supports that is rarely at hand on a test
bench. Protocol from
`sensors/esc_sensor.c`, names from rfsuite's `esc_parameters_flyrotor.lua`.

## The frame

`[0x73 sync][cmd][0][data length][data ...][CRC8][0x65 end]` -- **CRC-8/ATM**, polynomial
`0x07`, MSB-first, over everything before it (`flyCalculateCRC8()`). Three protocols in
this repository now use three different checksums; picking the wrong one is a frame the
firmware drops in silence.

| the flight controller sends | cmd | our answer | cmd |
|---|---|---|---|
| connect (`0xA0` as its one data byte) | `0x50` | connect acknowledged | `0xAF` |
| read information | `0x60` | page 0, 22 bytes | `0x9F` |
| read basic parameters | `0x61` | page 1, 12 bytes | `0x9E` |
| read advanced parameters | `0x62` | page 2, 10 bytes | `0x9D` |
| read other parameters | `0x64` | page 3, 10 bytes | `0x9B` |
| write basic / advanced / other | `0x90` / `0x91` / `0x92` | write acknowledged | `0x6F` / `0x6E` / `0x6D` |
| disconnect | `0x5F` | disconnect acknowledged | `0xA0` |

**The parameters are four pages, not one block**: `flyParamPages[] = {0x0016, 0x160C,
0x220A, 0x2C0A}` -- offset in the high byte, length in the low -- so the MSP buffer is
22 + 12 + 10 + 10 = **54 bytes**, 56 with the header. The flight controller publishes it
only when every page has been read, and after a write it re-reads that page alone.

**It connects on demand and disconnects when idle.** `flyBuildNextReq()` only opens a
connection while a read or a write is pending and closes it again when nothing is -- so a
FlyRotor dialogue is a short burst, not a steady conversation, and an emulator that only
answers reads will see the connection dropped and reopened around every write.
"""

import struct

from esc_wire import decode_fields, field_offset

SYNC = 0x73
END = 0x65
HEADER_LENGTH = 4
FOOTER_LENGTH = 2                       # CRC + end byte
BOOT_DELAY_S = 5.0
TEMP_OFFSET = 30

CMD_TELEMETRY = 0xFF
CMD_CONNECT, CMD_CONNECT_RESP = 0x50, 0xAF
CMD_DISCONNECT, CMD_DISCONNECT_RESP = 0x5F, 0xA0
CMD_READ_INFO, CMD_READ_INFO_RESP = 0x60, 0x9F
CMD_READ_BASIC, CMD_READ_BASIC_RESP = 0x61, 0x9E
CMD_READ_ADV, CMD_READ_ADV_RESP = 0x62, 0x9D
CMD_READ_OTHER, CMD_READ_OTHER_RESP = 0x64, 0x9B
CMD_WRITE_BASIC, CMD_WRITE_BASIC_RESP = 0x90, 0x6F
CMD_WRITE_ADV, CMD_WRITE_ADV_RESP = 0x91, 0x6E
CMD_WRITE_OTHER, CMD_WRITE_OTHER_RESP = 0x92, 0x6D

PARAM_SIG = 0x73                        # ESC_SIG_FLY, and rfsuite's `mspSignature`

#: `flyParamPages[]`: (offset, length) per page, in the firmware's order.
PAGES = ((0x00, 0x16), (0x16, 0x0C), (0x22, 0x0A), (0x2C, 0x0A))
PARAM_PAYLOAD_LENGTH = sum(length for _, length in PAGES)          # 54

READ_COMMANDS = {CMD_READ_INFO: 0, CMD_READ_BASIC: 1, CMD_READ_ADV: 2, CMD_READ_OTHER: 3}
READ_RESPONSES = {0: CMD_READ_INFO_RESP, 1: CMD_READ_BASIC_RESP,
                  2: CMD_READ_ADV_RESP, 3: CMD_READ_OTHER_RESP}
WRITE_COMMANDS = {CMD_WRITE_BASIC: 1, CMD_WRITE_ADV: 2, CMD_WRITE_OTHER: 3}
WRITE_RESPONSES = {1: CMD_WRITE_BASIC_RESP, 2: CMD_WRITE_ADV_RESP,
                   3: CMD_WRITE_OTHER_RESP}

#: rfsuite's map, as far as it names the 54 bytes. It is shorter than the payload, like
#: every other one measured; `decode_fields` leaves the rest under `_unmapped`.
FIELD_SPEC = (
    ("esc_id", "U32"), ("firmware_version", "U32"), ("hardware_version", "U32"),
    ("serial_number", "U64"), ("unknown_info", "U16"),
    ("gov_mode", "U8"), ("cell_count", "U8"), ("cutoff_voltage", "U8"),
    ("timing", "U8"), ("startup_power", "U8"), ("brake_type", "U8"),
    ("brake_force", "U8"), ("rotation", "U8"), ("active_freewheel", "U8"),
    ("gov_p", "U8"), ("gov_i", "U8"), ("motor_poles", "U8"),
)

#: Plausible defaults. The information page carries identity, the three others carry
#: settings; nothing upstream fixes these numbers, so they are only sane, not measured.
DEFAULT_PAYLOAD = (
    bytes([0x01, 0x00, 0x00, 0x00, 0x02, 0x01, 0x00, 0x00, 0x01, 0x00, 0x00, 0x00,
           0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x00, 0x00])     # page 0, 22
    + bytes([3, 12, 33, 15, 20, 1, 40, 0, 1, 5, 3, 10])                     # page 1, 12
    + bytes([2, 8, 30, 1, 0, 100, 10, 2, 1, 0])                             # page 2, 10
    + bytes([0, 1, 0, 0, 0, 0, 0, 0, 0, 0])                                 # page 3, 10
)
assert len(DEFAULT_PAYLOAD) == PARAM_PAYLOAD_LENGTH


def crc8_atm(data):
    """`flyCalculateCRC8()` -- polynomial 0x07, MSB-first, initial 0."""
    result = 0
    for byte in data:
        result ^= byte
        for _ in range(8):
            result = ((result << 1) ^ 0x07) & 0xFF if result & 0x80 else (result << 1) & 0xFF
    return result & 0xFF


class FlyRotor:
    """The ESC's four parameter pages, its connection state, and its telemetry."""

    name = "flyrotor"
    sig = PARAM_SIG
    spec = FIELD_SPEC
    boot_delay_s = BOOT_DELAY_S
    rate_hz = 20.0
    witness = "timing"
    msp_payload_length = PARAM_PAYLOAD_LENGTH

    def __init__(self, payload=None):
        self.payload = bytearray(payload or DEFAULT_PAYLOAD)
        self.connected = False
        self.streaming_enabled = True
        self.telemetry = dict(volts=25.2, amps=48.0, erpm=12_000.0, temp_c=44.0,
                              motor_temp_c=40.0, bec_volts=8.0, consumption_mah=350.0,
                              throttle_pct=70.0, pwm_pct=70.0, status=0)
        self.counts = {"connect": 0, "disconnect": 0, "read": 0, "write": 0,
                       "tele": 0, "rejected": 0}
        self.writes = []

    # ------------------------------------------------------------------ the payload

    def msp_payload(self):
        """What the flight controller would publish for this ESC."""
        return bytes(self.payload)

    def load_msp_payload(self, payload):
        """Restore a set saved earlier -- the round trip of `msp_payload()`."""
        if len(payload) == len(self.payload):
            self.payload[:] = payload

    def named(self):
        return decode_fields(self.payload, FIELD_SPEC)

    def named_from_msp(self, payload):
        return decode_fields(payload, FIELD_SPEC)

    def msp_with_change(self, payload, name, value):
        out = bytearray(payload)
        offset, size = field_offset(FIELD_SPEC, name)
        out[offset:offset + size] = int(value).to_bytes(size, "little")
        return bytes(out)

    # --------------------------------------------------------------------- framing

    @staticmethod
    def _frame(cmd, data=b""):
        body = bytes([SYNC, cmd, 0, len(data)]) + bytes(data)
        return body + bytes([crc8_atm(body), END])

    def telemetry_frame(self):
        """`0xFF`, with the fields at their big-endian offsets after the header."""
        state = self.telemetry
        data = bytearray(16)
        struct.pack_into(">H", data, 0, min(0xFFFF, int(round(state["volts"] * 100))))
        struct.pack_into(">H", data, 2, min(0xFFFF, int(round(state["amps"] * 100))))
        struct.pack_into(">H", data, 4, min(0xFFFF, int(round(state["consumption_mah"]))))
        struct.pack_into(">H", data, 6, min(0xFFFF, int(round(state["erpm"] / 10))))
        data[8] = min(255, int(round(state["pwm_pct"])))
        data[9] = min(255, int(round(state["temp_c"] + TEMP_OFFSET)))
        data[11] = min(255, int(round(state["motor_temp_c"] + TEMP_OFFSET)))
        data[12] = min(255, int(round(state["bec_volts"] * 10)))
        data[13] = state["status"] & 0xFF
        data[14] = min(255, int(round(state["throttle_pct"])))
        self.counts["tele"] += 1
        return self._frame(CMD_TELEMETRY, data)

    def frame_length(self, head):
        if len(head) < HEADER_LENGTH:
            return None
        if head[0] != SYNC:
            return 1                                       # resynchronise
        return HEADER_LENGTH + head[3] + FOOTER_LENGTH

    def handle(self, request):
        if (len(request) < HEADER_LENGTH + FOOTER_LENGTH or request[0] != SYNC
                or request[-1] != END or request[-2] != crc8_atm(request[:-FOOTER_LENGTH])):
            self.counts["rejected"] += 1          # bad framing or CRC: no state change
            return None
        cmd = request[1]
        data = request[HEADER_LENGTH:HEADER_LENGTH + request[3]]

        if cmd == CMD_CONNECT:
            self.connected = True
            self.counts["connect"] += 1
            return self._frame(CMD_CONNECT_RESP, b"\xa0")

        if cmd == CMD_DISCONNECT:
            self.connected = False
            self.counts["disconnect"] += 1
            return self._frame(CMD_DISCONNECT_RESP)

        if cmd in READ_COMMANDS:
            page = READ_COMMANDS[cmd]
            offset, length = PAGES[page]
            self.counts["read"] += 1
            return self._frame(READ_RESPONSES[page],
                               bytes(self.payload[offset:offset + length]))

        if cmd in WRITE_COMMANDS:
            page = WRITE_COMMANDS[cmd]
            offset, length = PAGES[page]
            if len(data) == length:
                self.payload[offset:offset + length] = data
                self.writes.append((page, bytes(data)))
            self.counts["write"] += 1
            return self._frame(WRITE_RESPONSES[page])

        self.counts["rejected"] += 1
        return None
