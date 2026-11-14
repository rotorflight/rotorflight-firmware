#!/usr/bin/env python3
"""A HobbyWing Platinum V5 as bytes -- the most stateful of the emulated ESCs.

    from pl5_esc import HobbyWingV5
    esc = HobbyWingV5()

Protocol from `sensors/esc_sensor.c`, field names from rfsuite's
`tasks/msp/api/esc_parameters_hw5.lua`. Nothing about a real Platinum's behaviour is
claimed: like every module here, this emulates what the firmware expects, which is what
a test of the firmware needs.

## Modbus-shaped, and driven entirely by the flight controller

Every frame ends with **CRC16 MODBUS** (`calculateCRC16_MODBUS()`), little-endian. A
response starts with `0xFD` or `0xFE`, carries a function byte at index 3 and a
little-endian type at 4-5, and the flight controller sizes it from those two alone
(`pl5Accept()`); the payload sits at index 7, except in a telemetry frame where it sits at
9.

| the flight controller sends | type | our answer | length | function |
|---|---|---|---|---|
| ping, every 480 ms when idle | `0x242C` | ping response | 11 | 3 |
| device info | `0x252C` | 48-byte payload at +7 | 73 | 3 |
| read parameters | `0x3538` | 96 bytes at +7, of which it keeps 48 | 105 | 3 |
| write parameters | `0x3538` | write acknowledgement | 58 | `0x17` |
| reset | `0x022B` | reset acknowledgement | 10 | 6 |

**The write is a three-step dance and the ESC has to hold its side of it.** On a successful
write the flight controller sets `pl5VerifyParamsAfterWrite`, re-reads the parameters and
**`memcmp`s them against what it sent** (`pl5DecodeGetParamsResp()`); a mismatch drops the
cache and starts over. Only when they match does it schedule the ESC reset, and only after
the reset response does it read everything again. So an emulator that acknowledges a write
without storing it does not merely fail the round trip -- it loops.

**What MSP publishes is two payloads glued together**: the 48 device-info bytes, then bytes
1..31 of the parameter read. That is 79, and 81 with the header -- which is exactly the
length of rfsuite's own fixture, the two sides agreeing for once.
"""

import struct

from esc_wire import decode_fields, field_offset

SIGNATURE = 0xFD                          # first byte of a response; also ESC_SIG_PL5
BOOT_DELAY_S = 5.0                        # PL5_BOOT_DELAY
TELE_TYPE, TELE_LENGTH = 0x5C30, 32
PING_TYPE, PING_LENGTH = 0x242C, 11
DEVINFO_TYPE, DEVINFO_LENGTH = 0x252C, 73
RESET_TYPE, RESET_LENGTH = 0x022B, 10
#: `PL5_RESP_PARAMS_TYPE`. The bytes on the wire are `35 38` and the firmware reads them
#: as a little-endian `uint16` at offset 4, so the constant is **0x3835** and not the 0x3538
#: the byte order suggests. Reading it the other way round rejects every parameter request
#: the flight controller makes -- measured 2026-08-21, 237 requests refused in 20 s while
#: the wire looked perfectly healthy.
PARAMS_TYPE = 0x3835
GETPARAMS_LENGTH = 105
WRITEPARAMS_LENGTH = 58
DEVINFO_PAYLOAD_LENGTH = 48
GETPARAMS_PAYLOAD_LENGTH = 31
SET_PARAMS_PAYLOAD_LENGTH = 48
WRITE_REQUEST_LENGTH = 63
WRITE_REQUEST_HEADER_LENGTH = 13
REQUEST_LENGTH = 10                       # ping, devinfo, getparams and reset are all 10

#: rfsuite's map. The first three fields are the device-info block; `com_version` and the
#: sixteen bytes after it are the writable half.
FIELD_SPEC = (
    ("firmware_version", "U128"), ("hardware_version", "U128"), ("esc_type", "U128"),
    ("com_version", "U120"),
    ("flight_mode", "U8"), ("lipo_cell_count", "U8"), ("volt_cutoff_type", "U8"),
    ("cutoff_voltage", "U8"), ("bec_voltage", "U8"), ("startup_time", "U8"),
    ("gov_p_gain", "U8"), ("gov_i_gain", "U8"), ("auto_restart", "U8"),
    ("restart_time", "U8"), ("brake_type", "U8"), ("brake_force", "U8"),
    ("timing", "U8"), ("rotation", "U8"), ("active_freewheel", "U8"),
    ("startup_power", "U8"),
)

#: rfsuite's `SIM_RESPONSE` payload: 48 device-info bytes then the 31 writable ones.
_FIXTURE = bytes([
    32, 32, 32, 80, 76, 45, 48, 52, 46, 49, 46, 48, 50, 32, 32, 32,      # "   PL-04.1.02   "
    72, 87, 49, 49, 48, 54, 95, 86, 50, 48, 48, 52, 53, 54, 78, 66,      # "HW1106_V200456NB"
    80, 108, 97, 116, 105, 110, 117, 109, 95, 86, 53, 32, 32, 32, 32, 32,  # "Platinum_V5"
    80, 108, 97, 116, 105, 110, 117, 109, 32, 86, 53, 32, 32, 32, 32,    # "Platinum V5"
    0, 0, 0, 3, 0, 11, 6, 5, 25, 1, 0, 0, 24, 0, 0, 2,
])
DEFAULT_DEVINFO = _FIXTURE[:DEVINFO_PAYLOAD_LENGTH]
DEFAULT_WRITABLE = _FIXTURE[DEVINFO_PAYLOAD_LENGTH:]
assert len(DEFAULT_WRITABLE) == GETPARAMS_PAYLOAD_LENGTH


def crc16_modbus(data):
    """`calculateCRC16_MODBUS()` -- reflected 0xA001, init 0xFFFF."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0xA001 if crc & 1 else crc >> 1
    return crc & 0xFFFF


class HobbyWingV5:
    """The ESC's state: device info, the 48-byte parameter block, and the write dance."""

    name = "hobbywingv5"
    sig = SIGNATURE
    spec = FIELD_SPEC
    boot_delay_s = BOOT_DELAY_S
    rate_hz = 10.0
    witness = "timing"
    msp_payload_length = DEVINFO_PAYLOAD_LENGTH + GETPARAMS_PAYLOAD_LENGTH   # 79

    def __init__(self, devinfo=None, writable=None):
        self.devinfo = bytearray(devinfo or DEFAULT_DEVINFO)
        #: The 48 bytes the flight controller reads and writes back. Byte 0 is kept by the
        #: firmware across a write (`pl5ExpectedParamsPayload[0]`), bytes 1..31 are what
        #: MSP exposes, and the remaining 16 are read but never written.
        self.block = bytearray(48)
        self.block[1:1 + GETPARAMS_PAYLOAD_LENGTH] = writable or DEFAULT_WRITABLE
        self.streaming_enabled = True
        self.telemetry = dict(volts=25.2, amps=48.0, erpm=12_000.0, temp_c=44.0,
                              bec_volts=8.2, bec_amps=1.5, bec_temp_c=38.0,
                              motor_temp_c=40.0, consumption_mah=350.0,
                              throttle_pct=70.0, pwm_pct=70.0, status=0)
        self.counts = {"ping": 0, "devinfo": 0, "getparams": 0, "write": 0,
                       "reset": 0, "tele": 0, "rejected": 0}
        self.writes = []

    # ------------------------------------------------------------------ the payload

    def msp_payload(self):
        """What the flight controller assembles: device info, then bytes 1..31."""
        return bytes(self.devinfo) + bytes(self.block[1:1 + GETPARAMS_PAYLOAD_LENGTH])

    def load_msp_payload(self, payload):
        """Restore a set saved earlier: device info, then the writable half."""
        if len(payload) != self.msp_payload_length:
            return
        self.devinfo[:] = payload[:DEVINFO_PAYLOAD_LENGTH]
        self.block[1:1 + GETPARAMS_PAYLOAD_LENGTH] = payload[DEVINFO_PAYLOAD_LENGTH:]

    def named(self):
        return decode_fields(self.msp_payload(), FIELD_SPEC)

    def named_from_msp(self, payload):
        return decode_fields(payload, FIELD_SPEC)

    def msp_with_change(self, payload, name, value):
        out = bytearray(payload)
        offset, size = field_offset(FIELD_SPEC, name)
        out[offset:offset + size] = int(value).to_bytes(size, "little")
        return bytes(out)

    # --------------------------------------------------------------------- framing

    @staticmethod
    def _frame(function, frame_type, length, payload=b"", payload_at=7):
        body = bytearray(length - 2)
        body[0] = SIGNATURE
        body[1] = 0x01
        body[3] = function
        struct.pack_into("<H", body, 4, frame_type)
        body[payload_at:payload_at + len(payload)] = payload
        return bytes(body) + struct.pack("<H", crc16_modbus(bytes(body)))

    def telemetry_frame(self):
        """The 32-byte telemetry frame, payload at offset 9."""
        state = self.telemetry
        payload = struct.pack(
            "<BBBBHHHBBBBB6s",
            min(100, int(round(state["throttle_pct"]))),
            0, 0,
            state["status"] & 0xFF,
            min(0xFFFF, int(round(state["erpm"] / 10))),
            min(0xFFFF, int(round(state["volts"] * 10))),
            min(0xFFFF, int(round(state["amps"] * 10))),
            max(0, min(255, round(state["temp_c"]))),
            max(0, min(255, round(state["bec_temp_c"]))),
            max(0, min(255, round(state["motor_temp_c"]))),
            min(255, int(round(state["bec_volts"] * 10))),
            min(255, int(round(state["bec_amps"] * 10))),
            b"\xff" * 6)
        self.counts["tele"] += 1
        return self._frame(3, TELE_TYPE, TELE_LENGTH, payload, payload_at=9)

    def frame_length(self, head):
        """Size the flight controller's request: everything is 10 bytes but a write."""
        if len(head) < 6:
            return None
        if head[0] != 0x01:
            return 1                                  # resynchronise
        frame_type = struct.unpack_from("<H", head, 4)[0]
        if frame_type == PARAMS_TYPE and head[3] == 0x17:
            return WRITE_REQUEST_LENGTH
        return REQUEST_LENGTH

    def handle(self, request):
        if (len(request) < 8 or request[0] != 0x01
                or struct.unpack_from("<H", request, len(request) - 2)[0]
                != crc16_modbus(request[:-2])):
            self.counts["rejected"] += 1          # bad framing or CRC: no state change
            return None
        function = request[3]
        frame_type = struct.unpack_from("<H", request, 4)[0]

        if frame_type == PING_TYPE:
            self.counts["ping"] += 1
            return self._frame(3, PING_TYPE, PING_LENGTH)

        if frame_type == DEVINFO_TYPE:
            self.counts["devinfo"] += 1
            return self._frame(3, DEVINFO_TYPE, DEVINFO_LENGTH, bytes(self.devinfo))

        if frame_type == RESET_TYPE:
            self.counts["reset"] += 1
            return self._frame(6, RESET_TYPE, RESET_LENGTH)

        if frame_type == PARAMS_TYPE and function == 0x17:
            # A write: the flight controller sends the whole 48-byte block, then re-reads
            # it and compares. Storing it is the only way through.
            payload = request[WRITE_REQUEST_HEADER_LENGTH:
                              WRITE_REQUEST_HEADER_LENGTH + SET_PARAMS_PAYLOAD_LENGTH]
            if len(payload) == SET_PARAMS_PAYLOAD_LENGTH:
                self.block[:] = payload
                self.writes.append(bytes(payload))
            self.counts["write"] += 1
            return self._frame(0x17, PARAMS_TYPE, WRITEPARAMS_LENGTH)

        if frame_type == PARAMS_TYPE:
            # A read: 96 register bytes, of which the flight controller keeps the first 48.
            self.counts["getparams"] += 1
            return self._frame(3, PARAMS_TYPE, GETPARAMS_LENGTH,
                               bytes(self.block) + bytes(48))

        self.counts["rejected"] += 1
        return None
