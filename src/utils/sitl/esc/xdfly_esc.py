#!/usr/bin/env python3
"""XDFLY, OMPHOBBY and ZTW -- three protocol names, one implementation, as bytes.

    from xdfly_esc import XDFly
    esc = XDFly(flavour="omphobby")

Rotorflight serves all three from `xdflySensorInit(sig)` (`sensors/esc_sensor.c`), and
they differ in exactly two things: the sync byte that opens a parameter frame (`0xA5`
XDFLY, `0xA4` OMPHOBBY, `0xA3` ZTW) and the signature the flight controller reports over
MSP (`0xA6`, `0xD0`, `0xDD`). Everything else -- frames, parameters, the walk -- is shared,
so this file is one class with a flavour rather than three copies. Field names from
rfsuite's `esc_parameters_xdfly.lua` / `_omp.lua` / `_ztw.lua`, which agree on the layout.

**Since #478 they need `esc_sensor_halfduplex = ON` for the parameter dialogue**, like the
four `rrfsm` protocols: `xdflySensorInit(sig, bidirectional)` only installs the handshake
crank and `paramCommit` when it is set; OFF is telemetry only. The handshake reply has to
be decoded within 10 ms, which needs the ESC sensor task at 100 Hz or faster (the SITL
runs it at 250 Hz).

## Two frame shapes, and both are big-endian

**A parameter frame is eight bytes**: `[sync][0x08][cmd][index][value hi][value lo][crc
hi][crc lo]`, CRC16 MODBUS over the first six, **stored big-endian** -- the opposite of
HobbyWing's, which uses the same CRC little-endian.

| cmd | direction | meaning |
|---|---|---|
| `0x03` | FC → ESC | handshake; the ESC answers `0x00 0x55 0x55` and only then is it *connected* |
| `0x33` | FC → ESC | read parameter `index` |
| `0x34` | FC → ESC | write parameter `index`, value with bit 15 set |
| `0x03`/`0xCD` | ESC → FC | the answer to any of them, carrying the index and the value |

**The high bit of a value means *this parameter exists on this ESC***, not part of the
number: `xdfly_params[i] = raw & 0x7FFF` and `xdfly_params_active[i] = raw & 0x8000`. The
flight controller then **skips inactive parameters when writing**, so an emulator that
leaves the bit clear is one whose parameters can be read and never written.

**A telemetry frame is 32 bytes**: `[0xDD][0x01][0x20]` and then fixed big-endian offsets
(voltage 3-4, current 5-6, throttle 7, rpm 8-9, temperature 10, power 12, status 13-14,
consumption 15-16, BEC volts 19). The firmware checks the length and the version and **not
a CRC** (`xdfly_decode()`), which is worth knowing before hunting a checksum that is never
verified.

**The walk is driven by telemetry.** After the handshake the flight controller asks for
parameter 0 and steps forward on each answer -- but a stalled walk is restarted by a
*telemetry* frame (`xdfly_decode()`'s telemetry branch calls `xdfly_get_next_param()`),
and the **write** walk is started the same way: `xdfly_param_commit()` only sets a state,
and the first write request goes out when the next telemetry frame arrives. An ESC that
falls silent after the handshake therefore never gets programmed, however correctly it
answers.

The 21 parameters -- 19 real ones plus two validity words the firmware fills in itself --
are copied into the MSP buffer as **native little-endian `uint16`**, even though the wire
is big-endian.
"""

import struct

from esc_wire import decode_fields, field_offset
from pl5_esc import crc16_modbus

FLAVOURS = {
    #  name         sync   MSP signature
    "xdfly": (0xA5, 0xA6),
    "omphobby": (0xA4, 0xD0),
    "ztw": (0xA3, 0xDD),
}

TELEM_SYNC = 0xDD
TELEM_VERSION = 0x01
TELEM_FRAME_LENGTH = 0x20
FRAME_LENGTH = 8
BOOT_DELAY_S = 5.0

CMD_HANDSHAKE = 0x03
CMD_RESPONSE = 0xCD
CMD_SET_PARAM = 0x34
CMD_GET_PARAM = 0x33

PARAM_COUNT = 21                 # XDFLY_PARAM_COUNT, the last two are validity words
VALIDITY_FIELDS = 2
REAL_PARAMS = PARAM_COUNT - VALIDITY_FIELDS

#: The firmware's own enum order, and rfsuite's names for the same slots.
FIELD_SPEC = (
    ("model", "U16"), ("gov_mode", "U16"), ("cutoff", "U16"), ("timing", "U16"),
    ("lv_bec_voltage", "U16"), ("rotation", "U16"), ("gov_p", "U16"), ("gov_i", "U16"),
    ("acceleration", "U16"), ("auto_restart_time", "U16"), ("hv_bec_voltage", "U16"),
    ("startup_power", "U16"), ("brake_type", "U16"), ("brake_force", "U16"),
    ("sr_function", "U16"), ("capacity_correction", "U16"), ("motor_poles", "U16"),
    ("led_colour", "U16"), ("smart_fan", "U16"),
    ("valid_1", "U16"), ("valid_2", "U16"),
)

#: Plausible values for the nineteen real parameters, in the firmware's enum order.
DEFAULT_PARAMS = (1, 3, 30, 15, 6, 0, 5, 3, 2, 10, 8, 20, 1, 40, 0, 100, 10, 2, 1)


class XDFly:
    """One ESC of the XDFLY family: parameters that answer, and telemetry that drives."""

    boot_delay_s = BOOT_DELAY_S
    rate_hz = 20.0
    #: **This family streams throughout, and it is not a choice.** `xdfly_decode()`'s
    #: telemetry branch is what pumps both walks -- `xdfly_get_next_param()` when caching
    #: and `xdfly_write_next_param()` when writing -- and `xdfly_param_commit()` only sets
    #: a state. Measured 2026-08-21: with telemetry withheld during the dialogue the write
    #: walk stopped after its first parameter and the round trip never completed.
    telemetry_after_reply = True
    witness = "timing"
    msp_payload_length = PARAM_COUNT * 2                    # 42
    spec = FIELD_SPEC

    def __init__(self, flavour="xdfly", params=None):
        if flavour not in FLAVOURS:
            raise ValueError(f"unknown flavour {flavour!r}; one of {sorted(FLAVOURS)}")
        self.name = flavour
        self.sync, self.sig = FLAVOURS[flavour]
        self.params = list(params or DEFAULT_PARAMS)
        #: Which parameters this ESC declares it has. The flight controller writes only
        #: these, so an emulator with everything inactive is read-only by accident.
        self.active = [True] * REAL_PARAMS
        self.connected = False
        self.streaming_enabled = True
        self.telemetry = dict(volts=25.2, amps=48.0, erpm=12_000.0, temp_c=44.0,
                              bec_volts=8.0, consumption_mah=350.0, throttle_pct=70.0,
                              pwm_pct=70.0, status=0)
        self.counts = {"handshake": 0, "get": 0, "set": 0, "tele": 0, "rejected": 0}
        self.writes = []

    # ------------------------------------------------------------------ the payload

    def msp_payload(self):
        """What the flight controller assembles: 19 parameters then two validity words."""
        valid_1 = sum(1 << i for i in range(min(16, REAL_PARAMS)) if self.active[i])
        valid_2 = sum(1 << (i - 16) for i in range(16, REAL_PARAMS) if self.active[i])
        return struct.pack(f"<{PARAM_COUNT}H", *self.params, valid_1, valid_2)

    def load_msp_payload(self, payload):
        """Restore a set saved earlier. The two validity words are derived, not stored."""
        if len(payload) == self.msp_payload_length:
            self.params = list(struct.unpack(f"<{PARAM_COUNT}H", payload))[:REAL_PARAMS]

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

    def _param_frame(self, cmd, index, value):
        body = bytes([self.sync, FRAME_LENGTH, cmd, index,
                      (value >> 8) & 0xFF, value & 0xFF])
        crc = crc16_modbus(body)
        return body + bytes([crc >> 8, crc & 0xFF])

    def telemetry_frame(self):
        """The 32-byte telemetry frame -- big-endian, and the firmware checks no CRC."""
        state = self.telemetry
        frame = bytearray(TELEM_FRAME_LENGTH)
        frame[0], frame[1], frame[2] = TELEM_SYNC, TELEM_VERSION, TELEM_FRAME_LENGTH
        struct.pack_into(">H", frame, 3, min(0xFFFF, int(round(state["volts"] * 10))))
        struct.pack_into(">H", frame, 5, min(0xFFFF, int(round(state["amps"] * 10))))
        frame[7] = min(255, int(round(state["throttle_pct"])))
        struct.pack_into(">H", frame, 8, min(0xFFFF, int(round(state["erpm"] / 10))))
        frame[10] = min(255, int(round(state["temp_c"])))
        frame[12] = min(255, int(round(state["pwm_pct"])))
        struct.pack_into(">H", frame, 13, state["status"] & 0xFFFF)
        struct.pack_into(">H", frame, 15, min(0xFFFF, int(round(state["consumption_mah"]))))
        frame[19] = min(255, int(round(state["bec_volts"])))
        self.counts["tele"] += 1
        return bytes(frame)

    def frame_length(self, head):
        if not head:
            return None
        if head[0] != self.sync:
            return 1                                       # resynchronise
        return FRAME_LENGTH

    def handle(self, request):
        if len(request) < FRAME_LENGTH or request[0] != self.sync:
            self.counts["rejected"] += 1
            return None
        cmd, index = request[2], request[3]

        if cmd == CMD_HANDSHAKE:
            # `0x00 0x55 0x55` is the whole handshake, and until it arrives the flight
            # controller is not *connected* and asks for nothing.
            self.connected = True
            self.counts["handshake"] += 1
            return self._param_frame(CMD_RESPONSE, 0x00, 0x5555)

        if cmd == CMD_GET_PARAM and index < PARAM_COUNT:
            self.counts["get"] += 1
            if index < REAL_PARAMS:
                value = self.params[index] & 0x7FFF
                if self.active[index]:
                    value |= 0x8000                        # "this ESC has this parameter"
            else:
                value = 0
            return self._param_frame(CMD_RESPONSE, index, value)

        if cmd == CMD_SET_PARAM and index < REAL_PARAMS:
            value = ((request[4] << 8) | request[5]) & 0x7FFF
            self.params[index] = value
            self.writes.append((index, value))
            self.counts["set"] += 1
            return self._param_frame(CMD_RESPONSE, index, value | 0x8000)

        self.counts["rejected"] += 1
        return None
