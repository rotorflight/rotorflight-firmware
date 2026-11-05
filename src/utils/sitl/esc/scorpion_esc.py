#!/usr/bin/env python3
"""A Scorpion (Tribunus) ESC, as bytes -- telemetry AND the forward-programming half.

    from scorpion_esc import Tribunus
    esc = Tribunus()
    wire = esc.unc_frame(volts=25.2, amps=48.0, erpm=12_000, temp_c=44, bec_volts=8.2)
    reply = esc.handle(request)          # the flight controller's parameter dialogue

**Why this can be written without a Scorpion at hand.** The protocol is not
guessed anywhere here: Rotorflight's own decoder is the specification
(`sensors/esc_sensor.c` ff, 4.6.0), and every constant below is read off it. What the
firmware does NOT hold is what the parameter bytes MEAN, and that comes from the other
side of the same wire -- `rfsuite`'s `tasks/msp/api/esc_parameters_scorpion.lua`, which is
the radio-side reader of exactly this payload, plus its forward-programming page
`app/pages/setup/esc_motors/esc_tools/escmfg/scorp/page.lua`. The values below are
transcribed from them, not fetched at run time.

**What is therefore honest to claim, and what is not.** The wire format, the framing, the
CRC, the address ranges and the field semantics are all documented. What no document
supplies is a *real* Tribunus's behaviour -- its startup timing, the values it reports
under load, how its error byte moves in a fault. So this emulates the **protocol**
faithfully and the **device** only as far as plausible defaults go, and nothing here may be
cited as evidence about a physical ESC.

## The two halves

**Telemetry** is one unsolicited frame, `0x55`, at 10 Hz: 4 header bytes, a 16-byte
`Log_rec_t`, and CRC16-CCITT over the first 20. That half needs no dialogue at all.

**Forward programming** is a request/response dialogue the flight controller drives, and it
exists **only when `esc_sensor_halfduplex = ON`**: `escSensorInit()` passes that flag as
`tribSensorInit(bidirectional)`, and the parameter machinery -- `escSig`, `paramCommit`,
the invalid/dirty bitmaps, the UNC abort -- is entirely inside that branch
(`esc_sensor.c`). With it off the firmware decodes telemetry and nothing else,
so a probe that finds no parameters should check the configuration before the feeder.

The dialogue is eight address ranges, 82 bytes in all, which the flight controller
concatenates in this order into the buffer MSP 217 hands out (behind a 2-byte header the
firmware supplies itself):

| # | request | addr | len | what rfsuite calls it |
|---|---|---|---|---|
| 0 | `0x53` settings | `0x00` | 32 | `escinfo_1..32` -- the identity string |
| 1 | `0x53` settings | `0x10` | 8 | `esc_mode`, `bec_voltage`, `rotation`, `telemetry_protocol` |
| 2 | `0x53` settings | `0x23` | 14 | `protection_delay`, `min_voltage`, `max_temperature`, `max_current`, `cutoff_handling`, `max_used`, `motor_startup_sound` |
| 3 | `0x50` system | `0x02` | 4 | `padding_1`, `padding_2` |
| 4 | `0x50` system | `0x05` | 2 | `padding_3` |
| 5 | `0x53` settings | `0x14` | 6 | `soft_start_time`, `runup_time`, `bailout` |
| 6 | `0x53` settings | `0x18` | 8 | `gov_proportional`, `gov_integral` -- **IQ22 on the wire** |
| 7 | `0x53` settings | `0x34` | 8 | **unnamed** |

**That last row is a finding, not a gap in this file.** The firmware's payload is 82 bytes
(84 with the header); rfsuite's field map covers 76, so the whole `0x34` range has no
reader on the radio side. Measured 2026-08-21 by summing both sides. The emulator serves
it as opaque bytes, which is exactly what a real ESC would do to a reader that ignores it.

## The rules the flight controller checks, and which this file must therefore obey

- **The response header echoes the request**, all six bytes except the length at index 4
  (`tribValidateResponseHeader()`, `esc_sensor.c`). Byte 5 is the header CRC and
  the firmware sends 0, meaning *not used* -- so nothing here computes one.
- **A read answers header + `len` data bytes; a write answers the header alone.** The
  firmware decides which from bit `0x80` of the request byte (`tribAccept()`), so `0xD3`
  (write settings) and `0xD0` (reset) are header-only replies.
- **A length must be even and at most 32**, or the frame is rejected outright.
- **A write is followed by a re-read of the same range.** `tribDecodeWriteParamResp()`
  clears the dirty bit and sets the invalid bit again, so the flight controller reads back
  what it wrote -- which is what makes a *stored* value the only way to pass a round trip.
"""

import struct

import esc_wire

# --- the packet types, off `esc_sensor.c` ----------------------------------
READ_SYSTEM = 0x50
READ_STATUS = 0x51
READ_SETTINGS = 0x53
WRITE_SETTINGS = 0xD3
RESET = 0xD0
UNSOLICITED = 0x55

REQ_WRITE = 0x80          # TRIB_REQ_WRITE -- the bit that makes a reply header-only
REQ_SIG = 0x50            # TRIB_REQ_SIG -- every packet type carries these bits
HEADER_LENGTH = 6         # TRIB_HEADER_LENGTH
UNC_HEADER_LENGTH = 4     # TRIB_UNC_HEADER_LENGTH
UNC_FRAME_LENGTH = 22     # 4 header + 16 Log_rec_t + 2 CRC
UNC_RATE_HZ = 10          # "Frame rate running:10Hz idle:1Hz"
BOOT_DELAY_S = 4.0        # TRIB_REQ_BOOT_DELAY -- the FC is quiet this long after boot
IQ22_ADDR = 0x18          # TRIB_PARAM_IQ22_ADDR
Q22 = 1 << 22

#: (address, length, system) in the firmware's own order -- `tribParamAddrLen[]`,
#: `esc_sensor.c`. The order is the layout: the flight controller concatenates the
#: ranges into one buffer in exactly this sequence.
PARAM_RANGES = (
    (0x00, 32, False),
    (0x10, 8, False),
    (0x23, 14, False),
    (0x02, 4, True),
    (0x05, 2, True),
    (0x14, 6, False),
    (0x18, 8, False),
    (0x34, 8, False),
)

PARAM_PAYLOAD_LENGTH = sum(length for _, length, _ in PARAM_RANGES)      # 82
MSP_PARAM_LENGTH = PARAM_PAYLOAD_LENGTH + 2                             # + PARAM_HEADER
PARAM_SIG = 0x53                                                        # ESC_SIG_TRIB

#: The named U16 fields, in payload order, starting after the 32-byte identity block.
#: Transcribed from rfsuite's `FIELD_SPEC`.
U16_FIELDS = (
    "esc_mode", "bec_voltage", "rotation", "telemetry_protocol",
    "protection_delay", "min_voltage", "max_temperature", "max_current",
    "cutoff_handling", "max_used", "motor_startup_sound",
    "padding_1", "padding_2", "padding_3",
    "soft_start_time", "runup_time", "bailout",
)
U16_BASE = 32
IQ22_OFFSET = U16_BASE + 2 * len(U16_FIELDS)                            # 66
UNNAMED_OFFSET = IQ22_OFFSET + 8                                        # 74

#: rfsuite's own `SIM_RESPONSE`, minus the two header bytes the firmware supplies. It is a
#: simulator payload rather than a capture -- see the module docstring -- and the eight
#: bytes of range `0x34` are appended as zeros because nothing upstream names them.
DEFAULT_PAYLOAD = bytes([
    84, 114, 105, 98, 117, 110, 117, 115, 32, 69, 83, 67, 45, 54, 83, 45,   # "Tribunus ESC-"
    56, 48, 65, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 4, 0,                      # "6S-80A", then 4,0
    3, 0, 3, 0, 1, 0, 3, 0,                                                 # 0x10
    136, 19, 22, 3, 16, 39, 64, 31, 136, 19, 0, 0, 1, 0,                    # 0x23
    7, 2, 0, 6,                                                             # sys 0x02
    63, 0,                                                                  # sys 0x05
    160, 15, 64, 31, 208, 7,                                                # 0x14
    0, 0, 64, 0, 197, 32, 0, 0,                                             # 0x18, IQ22
    0, 0, 0, 0, 0, 0, 0, 0,                                                 # 0x34, unnamed
])
assert len(DEFAULT_PAYLOAD) == PARAM_PAYLOAD_LENGTH


#: The firmware's own `calculateCRC16_CCITT()` -- reflected, polynomial 0x8408. Not the
#: same function as OpenYGE's, which shares the name and is MSB-first.
crc16_ccitt = esc_wire.crc16_ccitt_reflected


def iq22_encode(proportional, integral):
    """The two governor gains, as the wire carries them.

    The inverse of `tribDecodeReadParamResp()`: the flight controller reads
    `round(raw / 2^22 * 100)` and `round(raw / 2^22 * 100000)`, so a store that wants to
    hand back 100 and 200 has to carry these.
    """
    return struct.pack("<II",
                       int(round(proportional / 100 * Q22)),
                       int(round(integral / 100000 * Q22)))


def iq22_decode(raw):
    first, second = struct.unpack("<II", raw)
    return (round(first / Q22 * 100), round(second / Q22 * 100000))


class Tribunus:
    """The ESC's own state: what it reports, and what it has been programmed to.

    The parameter store is the point. A stub that answered every read with a constant
    would pass a *read* test and fail the only one that matters -- the flight controller
    re-reads a range straight after writing it (`tribDecodeWriteParamResp()`), so a value
    that was not kept comes back as the old one.
    """

    name = "scorpion"
    sig = PARAM_SIG
    boot_delay_s = BOOT_DELAY_S
    rate_hz = UNC_RATE_HZ

    def __init__(self, payload=None):
        self.payload = bytearray(payload or DEFAULT_PAYLOAD)
        self.unc_enabled = True
        #: The shared loop asks the protocol for a frame; a caller with its own operating
        #: point sets `telemetry_source` instead (which `serve()` above does).
        self.telemetry = dict(volts=25.2, amps=48.0, erpm=12_000.0, temp_c=44.0,
                              bec_volts=8.2, consumption_mah=350.0, throttle_pct=70.0,
                              pwm_pct=70.0, status=0)
        self.telemetry_source = None
        self.counts = {"read_settings": 0, "read_system": 0, "read_status": 0,
                       "write_settings": 0, "reset": 0, "unc": 0, "rejected": 0}
        self.writes = []          # (addr, bytes) in the order the flight controller sent
        self._timestamp_ms = 0

    # ------------------------------------------------------------- the payload, by name

    @staticmethod
    def _offset_of(addr):
        offset = 0
        for range_addr, length, _ in PARAM_RANGES:
            if range_addr == addr:
                return offset, length
            offset += length
        return None, None

    def range_bytes(self, addr):
        offset, length = self._offset_of(addr)
        if offset is None:
            return None
        return bytes(self.payload[offset:offset + length])

    def set_range(self, addr, data):
        offset, length = self._offset_of(addr)
        if offset is None or len(data) != length:
            return False
        self.payload[offset:offset + length] = data
        return True

    def named(self, wire=True):
        """What the payload says, in rfsuite's names -- for a human and for a probe.

        **`wire` is not a formatting choice, it is which side of the conversion you are
        holding.** The two governor gains are IQ22 on the ESC's wire and plain scaled
        integers in the buffer the flight controller hands out over MSP -- it converts on
        the way in (`tribDecodeReadParamResp()`) and back on the way out
        (`tribBuildNextParamReq()`), and rfsuite reads them as `U32`. So the ESC's own
        store is `wire=True` and anything read from MSP 217 is `wire=False`.

        Measured 2026-08-21, byte for byte across all eight ranges: the ESC held
        `00004000 c5200000` at `0x18` and the flight controller's buffer held
        `64000000 c8000000` -- 100 and 200. Every other range was identical. Reading the
        MSP buffer as if it were the wire produced *0 / 5*: plausible numbers, wrong
        question, and the only reason it was caught is that the criterion named the two
        values instead of merely requiring them to be equal.
        """
        out = {"esc_info": bytes(self.payload[:32]).split(b"\x00")[0]
                            .decode("latin1").strip()}
        for index, name in enumerate(U16_FIELDS):
            out[name] = struct.unpack_from("<H", self.payload, U16_BASE + 2 * index)[0]
        raw = bytes(self.payload[IQ22_OFFSET:IQ22_OFFSET + 8])
        gains = iq22_decode(raw) if wire else struct.unpack("<II", raw)
        out["gov_proportional"], out["gov_integral"] = gains
        out["unnamed_0x34"] = bytes(self.payload[UNNAMED_OFFSET:]).hex()
        return out

    def set_named(self, name, value):
        if name not in U16_FIELDS:
            raise KeyError(f"{name} is not a U16 field of this payload")
        struct.pack_into("<H", self.payload, U16_BASE + 2 * U16_FIELDS.index(name),
                         int(value))

    # ------------------------------------------------------------------ the two halves

    def unc_frame(self, volts, amps, erpm, temp_c, bec_volts,
                  consumption_mah=0, throttle_pct=0.0, pwm_pct=0.0, status=0,
                  timestamp_ms=None):
        """One unsolicited telemetry frame, exactly as `tribDecodeLogRecord()` reads it.

        The units are the ESC's, not ours: 0.1 A, 0.1 V, °C, RPM in 5-rpm steps, throttle
        and PWM in 0.5 %. `erpm` is electrical RPM because that is what the firmware
        stores (`escSensorData[0].erpm = rpm * 5`), so the caller converts once and both
        the ESC frame and the SITL's `motor_erpm[]` can be fed the same number.
        """
        if timestamp_ms is None:
            self._timestamp_ms = (self._timestamp_ms + 100) & 0xFFFFFF
            timestamp_ms = self._timestamp_ms
        record = bytearray(16)
        record[0:3] = struct.pack("<I", timestamp_ms & 0xFFFFFF)[:3]
        record[3] = min(255, int(round(throttle_pct * 2)))
        struct.pack_into("<H", record, 4, min(0xFFFF, int(round(amps * 10))))
        struct.pack_into("<H", record, 6, min(0xFFFF, int(round(volts * 10))))
        struct.pack_into("<H", record, 8, min(0xFFFF, int(round(consumption_mah))))
        record[10] = min(255, int(round(temp_c)))
        record[11] = min(255, int(round(pwm_pct * 2)))
        record[12] = min(255, int(round(bec_volts * 10)))
        struct.pack_into("<H", record, 13, min(0xFFFF, int(round(erpm / 5))))
        record[15] = status & 0xFF

        frame = bytearray([UNSOLICITED, 0x00, UNC_FRAME_LENGTH, 0x00])
        frame += record
        frame += struct.pack("<H", crc16_ccitt(bytes(frame)))
        self.counts["unc"] += 1
        return bytes(frame)

    def handle(self, request):
        """Answer one parameter request. Returns the reply bytes, or None if unparseable.

        The reply's header is the request's, byte for byte, except the length -- that is
        the rule `tribValidateResponseHeader()` enforces and the reason nothing here
        builds a header of its own.
        """
        if len(request) < HEADER_LENGTH:
            self.counts["rejected"] += 1
            return None
        req, addr, length = request[0], request[3], request[4]
        if (req & REQ_SIG) != REQ_SIG:
            self.counts["rejected"] += 1
            return None
        header = bytearray(request[:HEADER_LENGTH])

        if req == READ_STATUS:
            # The flight controller reads the status region to take the ESC out of
            # unsolicited mode before it programs it -- and **that is not a permanent
            # latch.** Once the dialogue is done, `tribCrankUncSetup()` leaves the state
            # machine waiting for a frame again (`rrfsmSensorProcess()`'s *frame
            # incomplete? check later*), so an ESC that stayed silent from here on would
            # leave the flight controller with no telemetry at all. Measured 2026-08-21:
            # latching it off passed every parameter criterion and reported 0.00 V.
            # What keeps the wire clear during a dialogue is the gap rule in `serve()`,
            # not this flag.
            self.counts["read_status"] += 1
            header[4] = length
            return bytes(header) + bytes(length)

        if req in (READ_SETTINGS, READ_SYSTEM):
            data = self.range_bytes(addr)
            if data is None or len(data) != length:
                self.counts["rejected"] += 1
                return None
            self.counts["read_settings" if req == READ_SETTINGS else "read_system"] += 1
            header[4] = length
            return bytes(header) + data

        if req == WRITE_SETTINGS:
            payload = request[HEADER_LENGTH:HEADER_LENGTH + length]
            if len(payload) != length or not self.set_range(addr, payload):
                self.counts["rejected"] += 1
                return None
            self.counts["write_settings"] += 1
            self.writes.append((addr, bytes(payload)))
            return bytes(header)

        if req == RESET:
            self.payload = bytearray(DEFAULT_PAYLOAD)
            self.unc_enabled = True
            self.counts["reset"] += 1
            return bytes(header)

        self.counts["rejected"] += 1
        return None

    # ---------------------------------------------------------------------- the framing

    # ------------------------------------------------- what a generic probe needs

    witness = "bailout"
    msp_payload_length = PARAM_PAYLOAD_LENGTH

    def msp_payload(self):
        """What the flight controller would publish for this ESC."""
        return bytes(self.payload)

    def load_msp_payload(self, payload):
        """Restore a set saved earlier -- the round trip of `msp_payload()`."""
        if len(payload) == len(self.payload):
            self.payload[:] = payload

    def named_from_msp(self, payload):
        return Tribunus(payload).named(wire=False)

    def msp_with_change(self, payload, name, value):
        other = Tribunus(payload)
        other.set_named(name, value)
        return bytes(other.payload)

    @property
    def streaming_enabled(self):
        return self.unc_enabled

    def telemetry_frame(self):
        """The shared wire's name for one unsolicited frame."""
        state = self.telemetry_source() if self.telemetry_source else self.telemetry
        return self.unc_frame(**state)

    @staticmethod
    def frame_length(head):
        """How many bytes this request is, once its first six are in hand.

        A write carries its payload; a read is the header alone. Same rule as
        `tribAccept()`, read from the other side of the wire.
        """
        if len(head) < HEADER_LENGTH:
            return None
        return HEADER_LENGTH + (head[4] if head[0] & REQ_WRITE else 0)


#: The wire discipline lives in `esc_wire`: it is shared by every rrfsm protocol
#: (SCORPION, OPENYGE, HOBBYWINGV5, FLYROTOR), and its properties belong to that shared
#: state machine rather than to the Tribunus. `serve()` below keeps a Scorpion-specific
#: call shape (telemetry as a callable, events with the request kind).
QUIET_GAP_S = esc_wire.QUIET_GAP_S


def serve(sock, esc, telemetry, stop=None, on_event=None, rate_hz=UNC_RATE_HZ,
          mode="auto", quiet_gap=None, stream_after=8.0):
    """Scorpion call shape, delegating to the shared loop.

    `telemetry` is a callable returning `unc_frame()`'s keyword arguments; `on_event`
    still receives `(kind, request, reply)`.
    """
    esc.telemetry_source = telemetry
    bridge = (lambda request, reply: on_event(request[0], request, reply)) if on_event         else None
    return esc_wire.serve(sock, esc, stop=stop, on_event=bridge, rate_hz=rate_hz,
                          mode=mode, quiet_gap=quiet_gap or esc_wire.QUIET_GAP_S,
                          stream_after=stream_after)
