#!/usr/bin/env python3
"""An OpenYGE ESC as bytes -- telemetry and parameters.

    from oyge_esc import OpenYGE
    esc = OpenYGE()
    esc.telemetry.update(volts=25.2, amps=48.0, erpm=12_000)

The protocol is read out of Rotorflight's own decoder (`sensors/esc_sensor.c`) and the field names out of rfsuite
(`tasks/msp/api/esc_parameters_yge.lua`); nothing about a real YGE's behaviour is claimed.

## The frame

Six-byte header -- `sync 0xA5`, `version`, `frame_type`, `frame_length`, `seq`, `device` --
then the payload, then **CRC16-CCITT MSB-first with polynomial 0x1021**, little-endian at
the end. That CRC is *not* the Scorpion's: `esc_sensor.c` carries two functions of the same
name and this protocol uses `oygeCalculateCRC16_CCITT()` at `:3199`. Getting it wrong
produces frames the firmware drops without a word.

| frame type | direction | payload |
|---|---|---|
| `0x00` `TELE_AUTO` | ESC → FC | telemetry, unsolicited (the one-way mode) |
| `0x03` `TELE_REQ` | FC → ESC | `{index, param}`, both zero |
| `0x02` `TELE_RESP` | ESC → FC | telemetry, and **the `seq` must match the request** |
| `0x05` `WRITE_PARAM_REQ` | FC → ESC | `{index, value}` |
| `0x04` `WRITE_PARAM_RESP` | ESC → FC | telemetry again, `seq` matching |

**The parameters ride inside the telemetry frames.** There is no separate read: every
telemetry payload ends with `pidx`/`pdata`, and `oygeCacheParam()` files that pair into a
`uint16` array. `param[0]` is the **count**, and the flight controller publishes the buffer
over MSP only once every index below the count has been seen
(`~(~1ULL << (count - 1)) == oygeCachedParams`). So an ESC that wants to be readable has to
**cycle `pidx` through its whole set**, one per frame — which is exactly what this does.

**`version` must be 3 or higher.** Below that `oygeDecodeTelemetry()` switches the flight
controller into auto mode and sets `paramCommit = NULL` — telemetry keeps working and the
parameter half disappears, silently.

**Forward programming needs `esc_sensor_halfduplex = ON`**, like every rrfsm protocol:
`oygeSensorInit(bidirectional)` puts `oygeDecode` and `oygeParamCommit` behind that flag
and otherwise leaves the one-way `oygeDecodeAuto`.
"""

import struct

from esc_wire import crc16_ccitt_msb, decode_fields, field_offset

SYNC = 0xA5
VERSION = 3
HEADER_LENGTH = 6
CRC_LENGTH = 2
MIN_FRAME_LENGTH = 6                     # legacy header (4) + CRC (2)
TELEMETRY_PAYLOAD_LENGTH = 26
TEMP_OFFSET = 40                         # OPENYGE_TEMP_OFFSET
BOOT_DELAY_S = 5.0                       # OPENYGE_BOOT_DELAY

FTYPE_TELE_AUTO = 0x00
FTYPE_TELE_RESP = 0x02
FTYPE_TELE_REQ = 0x03
FTYPE_WRITE_PARAM_RESP = 0x04
FTYPE_WRITE_PARAM_REQ = 0x05
DEV_MASTER = 0x80

PARAM_SIG = 0xA5                         # ESC_SIG_OPENYGE, and rfsuite's `mspSignature`
PARAM_CACHE_SIZE_MAX = 64                # OPENYGE_PARAM_CACHE_SIZE_MAX

#: rfsuite's field map for the MSP buffer, minus its two header bytes. Shorter than the
#: payload on purpose -- see `decode_fields`.
FIELD_SPEC = (
    ("esc_model", "U8"), ("esc_version", "U8"),
    ("governor", "U16"), ("lv_bec_voltage", "U16"), ("timing", "U16"),
    ("acceleration", "U16"), ("gov_p", "U16"), ("gov_i", "U16"),
    ("throttle_response", "U16"), ("auto_restart_time", "U16"),
    ("cell_cutoff", "U16"), ("active_freewheel", "U16"), ("esc_type", "U16"),
    ("firmware_version", "U32"), ("serial_number", "U32"), ("unknown_1", "U16"),
    ("stick_zero_us", "U16"), ("stick_range_us", "U16"), ("unknown_2", "U16"),
    ("motor_poll_pairs", "U16"), ("pinion_teeth", "U16"), ("main_teeth", "U16"),
    ("min_start_power", "U16"), ("max_start_power", "U16"), ("unknown_3", "U16"),
    ("flags", "U8"), ("unknown_4", "U8"), ("current_limit", "U16"),
)

#: rfsuite's `SIM_RESPONSE` payload (50 bytes), padded to the count it declares.
#: **`param[0]` is the count and it says 32**, i.e. 64 bytes -- while rfsuite's own field
#: map covers 56 and its fixture carries 50. Both gaps are left as they are: the firmware
#: decides how many parameters exist, the radio names as many as it knows, and inventing
#: agreement between them would be the one thing this file must not do.
_FIXTURE = bytes([
    32, 0, 3, 0, 55, 0, 0, 0, 4, 0, 3, 0, 1, 0, 1, 0, 80, 3,
    131, 148, 1, 0, 30, 170, 0, 0, 3, 0, 86, 4, 22, 3, 163, 15,
    1, 0, 2, 0, 2, 0, 20, 0, 20, 0, 0, 0, 0, 0, 2, 19,
])
PARAM_COUNT = 32
DEFAULT_PARAMS = _FIXTURE + bytes(PARAM_COUNT * 2 - len(_FIXTURE))


class OpenYGE:
    """The ESC's state: what it reports, and the parameter array it publishes."""

    name = "openyge"
    sig = PARAM_SIG
    spec = FIELD_SPEC
    boot_delay_s = BOOT_DELAY_S
    rate_hz = 20.0                       # OPENYGE_FRAME_PERIOD is ~50 ms

    def __init__(self, params=None):
        self.params = bytearray(params or DEFAULT_PARAMS)
        self.streaming_enabled = True
        self.telemetry = dict(volts=25.2, amps=48.0, erpm=12_000.0, temp_c=44.0,
                              bec_volts=8.2, bec_amps=1.5, bec_temp_c=38.0,
                              consumption_mah=350.0, throttle_pct=70.0, pwm_pct=70.0,
                              status=0x0E)             # STATE_RUNNING_NORM
        self.counts = {"tele_req": 0, "write": 0, "auto": 0, "rejected": 0}
        self.writes = []
        self._pidx = 0
        self._seq = 0

    # ------------------------------------------------------------------ the payload

    @property
    def count(self):
        return struct.unpack_from("<H", self.params, 0)[0]

    def named(self):
        return decode_fields(self.params, FIELD_SPEC)

    def get(self, name):
        offset, size = field_offset(FIELD_SPEC, name)
        if offset is None:
            raise KeyError(name)
        return int.from_bytes(self.params[offset:offset + size], "little")

    def set(self, name, value):
        offset, size = field_offset(FIELD_SPEC, name)
        if offset is None:
            raise KeyError(name)
        self.params[offset:offset + size] = int(value).to_bytes(size, "little")

    # ------------------------------------------------- what a generic probe needs

    #: The field a round-trip test writes. `timing` is a plain U16 parameter with an
    #: index of its own, which is what the flight controller writes by.
    witness = "timing"

    @property
    def msp_payload_length(self):
        """What MSP 217 will hand out: `count` parameters of two bytes each."""
        return self.count * 2

    def msp_payload(self):
        """What the flight controller would publish for this ESC."""
        return bytes(self.params)

    def load_msp_payload(self, payload):
        """Restore a set saved earlier -- the round trip of `msp_payload()`."""
        if len(payload) == len(self.params):
            self.params[:] = payload

    def named_from_msp(self, payload):
        return decode_fields(payload, FIELD_SPEC)

    def msp_with_change(self, payload, name, value):
        """`payload` with one named field changed -- the buffer MSP 218 sends back."""
        out = bytearray(payload)
        offset, size = field_offset(FIELD_SPEC, name)
        out[offset:offset + size] = int(value).to_bytes(size, "little")
        return bytes(out)

    def param_index(self, name):
        """Which `pidx` carries this field -- the flight controller writes by index."""
        offset, size = field_offset(FIELD_SPEC, name)
        if offset is None or size != 2 or offset % 2:
            return None
        return offset // 2

    # --------------------------------------------------------------------- framing

    def _frame(self, frame_type, payload, seq=None, device=1):
        length = HEADER_LENGTH + len(payload) + CRC_LENGTH
        head = struct.pack("<BBBBBB", SYNC, VERSION, frame_type, length,
                           self._seq if seq is None else seq, device)
        body = head + bytes(payload)
        return body + struct.pack("<H", crc16_ccitt_msb(body))

    def _telemetry_payload(self, pidx=None):
        """One `OpenYGETelemetryFrame_t`, in the ESC's own units.

        The parameter pair is what makes the set readable at all, so it advances on every
        frame unless the caller names one.
        """
        state = self.telemetry
        if pidx is None:
            pidx = self._pidx
            self._pidx = (self._pidx + 1) % max(1, min(self.count, PARAM_CACHE_SIZE_MAX))
        pdata = struct.unpack_from("<H", self.params, 2 * pidx)[0] \
            if 2 * pidx + 2 <= len(self.params) else 0
        return struct.pack(
            "<BBHHHHbbHHBBBBBBHH",
            0,                                                        # reserved
            min(255, int(round(state["temp_c"] + TEMP_OFFSET))),
            min(0xFFFF, int(round(state["volts"] * 100))),
            min(0xFFFF, int(round(state["amps"] * 100))),
            min(0xFFFF, int(round(state["consumption_mah"]))),
            min(0xFFFF, int(round(state["erpm"] / 10))),
            max(-128, min(127, int(round(state["pwm_pct"])))),
            max(-128, min(127, int(round(state["throttle_pct"])))),
            min(0xFFFF, int(round(state["bec_volts"] * 1000))),
            min(0xFFFF, int(round(state["bec_amps"] * 1000))),
            min(255, int(round(state["bec_temp_c"] + TEMP_OFFSET))),
            state["status"] & 0xFF,
            min(255, int(round(state["temp_c"] + TEMP_OFFSET))),      # cap_temp
            min(255, int(round(state["temp_c"] + TEMP_OFFSET))),      # aux_temp
            0, 0,                                                     # status2, reserved1
            pidx, pdata)

    def telemetry_frame(self):
        """An unsolicited frame -- the `esc_sensor_halfduplex = OFF` mode."""
        self._seq = (self._seq + 1) & 0xFF
        self.counts["auto"] += 1
        return self._frame(FTYPE_TELE_AUTO, self._telemetry_payload())

    def frame_length(self, head):
        if len(head) < 4:
            return None
        if head[0] != SYNC:
            return 1                     # resynchronise a byte at a time
        return head[3]

    def handle(self, request):
        if len(request) < HEADER_LENGTH + CRC_LENGTH or request[0] != SYNC:
            self.counts["rejected"] += 1
            return None
        frame_type, seq = request[2], request[4]
        payload = request[HEADER_LENGTH:len(request) - CRC_LENGTH]

        if frame_type == FTYPE_TELE_REQ:
            self.counts["tele_req"] += 1
            return self._frame(FTYPE_TELE_RESP, self._telemetry_payload(), seq=seq)

        if frame_type == FTYPE_WRITE_PARAM_REQ and len(payload) >= 4:
            index, value = struct.unpack_from("<HH", payload, 0)
            if 2 * index + 2 <= len(self.params):
                struct.pack_into("<H", self.params, 2 * index, value)
            self.counts["write"] += 1
            self.writes.append((index, value))
            # The answer is a telemetry frame carrying the parameter just written, which
            # is what lets the flight controller re-cache it without a second exchange.
            return self._frame(FTYPE_WRITE_PARAM_RESP,
                               self._telemetry_payload(pidx=index), seq=seq)

        self.counts["rejected"] += 1
        return None
