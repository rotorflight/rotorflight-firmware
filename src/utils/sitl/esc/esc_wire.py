#!/usr/bin/env python3
"""The half-duplex ESC wire: the discipline every emulated ESC on this target needs.

    from esc_wire import serve
    serve(sock, protocol, stop=stop, on_event=printer)

**Why this is one file and not four.** Rotorflight drives four of its ESC protocols
through the same request/response state machine -- `rrfsmSensorProcess()` and friends,
shared by SCORPION (`trib*`), OPENYGE (`oyge*`), HOBBYWINGV5 (`pl5*`) and FLYROTOR
(`fly*`) -- so the properties below belong to **that state machine**, not to one ESC.
Every emulation inherits them, and a second copy of this loop would inherit its bugs
instead. Each was measured against the SITL:

- **A streaming ESC starves the flight controller's own request.** `rrfsmSensorProcess()`
  only reaches `rrfsmStartFrameAndSendPendingReq()` while nothing is arriving, so an ESC
  that talks continuously prevents the master from ever asking. Measured on the Scorpion:
  157 telemetry frames, zero requests.
- **The echo is part of the wire.** `rrfsmStartFrameAndSendPendingReq()` sets
  `readIngoreBytes = reqLength` and `rrfsmDataReceive()` drops that many bytes -- *don't
  listen to self* -- because on one wire the flight controller hears its own transmission.
  A TCP socket does not echo, so those dropped bytes would be the reply's own header.
- **The silence between frames must exceed the firmware's retry spacing.** The retry is
  ~1.4 s; a telemetry frame inside that window starves the *response* instead of the
  request, and the parameter round trip then converges one run in three.

A protocol object supplies four things and nothing else:

| member | contract |
|---|---|
| `frame_length(head)` | how many bytes the request beginning with `head` is, or `None` |
| `handle(request)` | the reply bytes, or `None` to say *unparseable* |
| `telemetry_frame()` | one unsolicited frame, or `None` if this protocol has none |
| `streaming_enabled` | whether unsolicited frames are wanted at all |
"""

import select
import time

#: How long the flight controller must have been silent before the ESC talks again.
#: Measured, not chosen: the firmware repeats an unanswered request about every 1.4 s,
#: and a frame inside that window starves the answer. 0.5 s converged one run in
#: three; 2.0 s was 9/9 three times over.
QUIET_GAP_S = 2.0

#: With no request in this long, assume the flight controller is not asking at all -- the
#: `esc_sensor_halfduplex = OFF` case -- and stream telemetry the way a one-way ESC does.
STREAM_AFTER_S = 8.0

BUFFER = 4096


def may_stream(mode, now, started, last_request, quiet_gap, stream_after):
    if mode == "stream":
        return True
    if last_request is not None:
        return now - last_request > quiet_gap
    if mode == "auto":
        return now - started > stream_after
    return False                                      # master, never yet asked


def serve(sock, protocol, stop=None, on_event=None, rate_hz=10.0,
          mode="auto", quiet_gap=QUIET_GAP_S, stream_after=STREAM_AFTER_S):
    """Answer every request at once, echo first, and stream only in the gaps.

    `mode` is `master` (the flight controller leads), `stream` (one-way telemetry) or
    `auto`, which decides by measurement: silent until asked, and streaming if nothing
    asks within `stream_after`.

    Returns `"closed"` when the flight controller closed the port, `"stopped"` otherwise.
    """
    pending = bytearray()
    started = time.monotonic()
    next_frame = started
    last_request = None

    while stop is None or not stop.is_set():
        timeout = max(0.0, next_frame - time.monotonic())
        ready, _, _ = select.select([sock], [], [], min(timeout, 0.25))
        if ready:
            chunk = sock.recv(BUFFER)
            if not chunk:
                return "closed"
            pending += chunk
            while pending:
                need = protocol.frame_length(bytes(pending))
                if need is None or len(pending) < need:
                    break
                request, pending = bytes(pending[:need]), pending[need:]
                last_request = time.monotonic()
                reply = protocol.handle(request)
                if reply is not None:
                    # The echo, then the answer. Never the answer alone: the firmware
                    # discards `reqLength` bytes first, and without the echo they are the
                    # header of this reply.
                    payload = request + reply
                    # And for a protocol whose walk is PUMPED by telemetry -- the XDFLY
                    # family, where `xdfly_decode()`'s telemetry branch is what asks for
                    # the next parameter and starts the write walk -- one frame rides
                    # along, in the same send. Free-running telemetry cannot do this job:
                    # a frame that lands between the request and the echo is eaten by
                    # `readIngoreBytes` and desynchronises the parser. Measured both ways
                    # 2026-08-21: with no telemetry the write walk stopped after one
                    # parameter, with free-running telemetry the read walk never finished.
                    if getattr(protocol, "telemetry_after_reply", False)                             and protocol.streaming_enabled:
                        payload += protocol.telemetry_frame() or b""
                    sock.sendall(payload)
                if on_event:
                    on_event(request, reply)
        now = time.monotonic()
        if now >= next_frame:
            next_frame = max(now, next_frame + 1.0 / rate_hz)
            if protocol.streaming_enabled and may_stream(mode, now, started, last_request,
                                                         quiet_gap, stream_after):
                frame = protocol.telemetry_frame()
                if frame:
                    sock.sendall(frame)
    return "stopped"


TYPE_LENGTH = {"U8": 1, "S8": 1, "U16": 2, "S16": 2, "U24": 3, "U32": 4, "U64": 8,
               "U120": 15, "U128": 16}

#: Anything wider than eight bytes is a string in every map seen so far -- rfsuite reads
#: HobbyWing's firmware, hardware and type fields as `U128` and prints them as text. A
#: 16-byte integer would be technically right and unreadable.
STRING_FROM = 9


def decode_fields(payload, spec, big=False):
    """`{name: value}` from a parameter payload and a field list.

    The spec is rfsuite's own, transcribed per protocol: `[(name, "U16"), ...]` in payload
    order. It is deliberately allowed to be SHORTER than the payload -- on every protocol
    measured so far the firmware carries more bytes than the radio names, and truncating
    the payload to the map would be inventing agreement.
    """
    out, offset = {}, 0
    for name, kind in spec:
        size = TYPE_LENGTH[kind]
        if offset + size > len(payload):
            break
        raw = bytes(payload[offset:offset + size])
        if size >= STRING_FROM:
            out[name] = raw.decode("latin1").replace(chr(0), " ").strip()
        else:
            out[name] = int.from_bytes(raw, "big" if big else "little",
                                       signed=kind.startswith("S"))
        offset += size
    out["_unmapped"] = bytes(payload[offset:]).hex()
    return out


def field_offset(spec, name):
    """(offset, length) of a named field, or (None, None)."""
    offset = 0
    for field, kind in spec:
        size = TYPE_LENGTH[kind]
        if field == name:
            return offset, size
        offset += size
    return None, None


def crc16_ccitt_reflected(data):
    """Reflected, polynomial 0x8408 -- Rotorflight's `calculateCRC16_CCITT()` (Scorpion)."""
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = (crc >> 1) ^ 0x8408 if crc & 1 else crc >> 1
    return crc & 0xFFFF


def crc16_ccitt_msb(data):
    """MSB-first, polynomial 0x1021, init 0 -- `oygeCalculateCRC16_CCITT()` (OpenYGE).

    **Two functions with the same name in the same file compute different CRCs**, and
    nothing but the caller distinguishes them: `esc_sensor.c` is the reflected one and
    `(second definition)` is this one. Emulating one protocol with the other's CRC produces a frame the
    firmware silently drops.
    """
    crc = 0
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) & 0xFFFF if crc & 0x8000 else (crc << 1) & 0xFFFF
    return crc & 0xFFFF
