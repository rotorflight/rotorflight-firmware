#!/usr/bin/env python3
"""A virtual ESC on the SITL's ESC UART -- any of the protocols the firmware can program.

    python3 src/utils/sitl/esc/esc_sim.py --protocol openyge --port 5763
    python3 src/utils/sitl/esc/esc_sim.py --protocol hobbywingv5 --port 5763 \
            --volts 24.8 --amps 62 --headspeed 1750
    python3 src/utils/sitl/esc/esc_sim.py --protocol ztw --port 5763 --seconds 60

The SITL serves UARTn on TCP 5760+n; give the ESC sensor a UART (`serial 2 1024 ...` for
UART3 = port 5763) and pick the protocol with `set esc_sensor_protocol = ...`.

The protocols live one to a module (`oyge_esc`, `pl5_esc`, `scorpion_esc`, `fly_esc`,
`xdfly_esc`), each saying where its constants come from; the wire discipline they share is
in `esc_wire`, the same for every protocol the firmware runs through its `rrfsm*`
request/response machine. This is the tool end: one socket, telemetry and the parameter dialogue,
and a summary of what the flight controller actually asked for.

**Which protocols can be programmed at all, and what each needs:**

| protocol (`esc_sensor_protocol`) | needs `esc_sensor_halfduplex = ON` |
|---|---|
| `openyge` (OPENYGE) | yes |
| `hobbywingv5` (HOBBYWINGV5) | yes |
| `scorpion` (SCORPION) | yes |
| `flyrotor` (FLYROTOR) | yes |
| `xdfly` / `omphobby` / `ztw` (XDFLY / OMPHOBBY / ZTW) | yes (since #478; OFF is telemetry only) |

Everything else the firmware speaks -- BLHELI32, HOBBYWINGV4, KONTRONIK, APD, GRAUPNER,
FBUS, RECORD -- decodes telemetry only.

**One socket, one client.** The ESC UART takes exactly one connection, so stop any other
feeder first.
"""

import argparse
import importlib
import json
import os
import socket
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)

import esc_wire                                                      # noqa: E402

PROTOCOLS = {
    "openyge": ("oyge_esc", "OpenYGE", {}),
    "hobbywingv5": ("pl5_esc", "HobbyWingV5", {}),
    "scorpion": ("scorpion_esc", "Tribunus", {}),
    "flyrotor": ("fly_esc", "FlyRotor", {}),
    "xdfly": ("xdfly_esc", "XDFly", {"flavour": "xdfly"}),
    "omphobby": ("xdfly_esc", "XDFly", {"flavour": "omphobby"}),
    "ztw": ("xdfly_esc", "XDFly", {"flavour": "ztw"}),
}

DEFAULT_POLES = 8
DEFAULT_GEAR = (63, 725)


def headspeed_to_erpm(headspeed, poles, gear):
    num, den = gear
    return headspeed * (den / num) * (poles / 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--protocol", required=True, choices=sorted(PROTOCOLS))
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, required=True, help="TCP port of the ESC UART, 5760 + n")
    ap.add_argument("--seconds", type=float, default=0.0, help="0 = until interrupted")
    ap.add_argument("--volts", type=float, default=25.2)
    ap.add_argument("--amps", type=float, default=48.0)
    ap.add_argument("--temp", type=float, default=44.0)
    ap.add_argument("--bec", type=float, default=8.2)
    ap.add_argument("--consumption", type=float, default=350.0)
    ap.add_argument("--throttle", type=float, default=70.0)
    ap.add_argument("--erpm", type=float, default=None)
    ap.add_argument("--headspeed", type=float, default=None)
    ap.add_argument("--poles", type=int, default=DEFAULT_POLES)
    ap.add_argument("--gear", default=f"{DEFAULT_GEAR[0]},{DEFAULT_GEAR[1]}")
    ap.add_argument("--mode", choices=("auto", "master", "stream"), default="auto",
                    help="auto measures which one applies; master is half duplex with the "
                         "flight controller leading; stream is one-way telemetry")
    ap.add_argument("--state", default=None,
                    help="JSON file the programmed parameters are loaded from and saved to")
    ap.add_argument("--params", default=None,
                    help="k=v[,k=v...] -- start this ESC holding these values instead of the "
                         "fixture's. For a test that has to tell a page which PARSED from a "
                         "page which merely drew its own defaults: where the two agree, the "
                         "two are indistinguishable on screen. An unknown field name is "
                         "refused rather than ignored.")
    ap.add_argument("--quiet", action="store_true")
    args = ap.parse_args()

    port = args.port

    module_name, class_name, kwargs = PROTOCOLS[args.protocol]
    esc = getattr(importlib.import_module(module_name), class_name)(**kwargs)

    restored = None
    if args.state and os.path.exists(args.state):
        with open(args.state, encoding="utf-8") as handle:
            saved = json.load(handle)
        if saved.get("protocol") == args.protocol:
            esc.load_msp_payload(bytes.fromhex(saved["payload"]))
            restored = args.state

    # Applied AFTER --state, so a saved set is the starting point and these override it --
    # the other order would make a restored ESC silently ignore the request.
    overridden = []
    if args.params:
        from esc_wire import field_offset                            # noqa: WPS433
        payload = esc.msp_payload()
        for pair in args.params.split(","):
            pair = pair.strip()
            if not pair:
                continue
            if "=" not in pair:
                ap.error(f"--params takes k=v pairs, got {pair!r}")
            name, _, value = pair.partition("=")
            name, value = name.strip(), value.strip()
            offset, size = field_offset(esc.spec, name)
            if offset is None:
                ap.error(
                    f"--params: {args.protocol} has no field {name!r}. It carries "
                    f"{', '.join(f for f, _ in esc.spec)}. A field this emulator does not "
                    f"model would be set nowhere and read back as whatever the fixture "
                    f"holds, which is the shape of a test that passes while measuring "
                    f"nothing.")
            payload = esc.msp_with_change(payload, name, int(value, 0))
            overridden.append(f"{name}={value}")
        esc.load_msp_payload(payload)

    gear = tuple(int(part) for part in args.gear.split(","))
    erpm = (args.erpm if args.erpm is not None else
            headspeed_to_erpm(args.headspeed if args.headspeed is not None else 1800,
                              args.poles, gear))
    esc.telemetry.update(volts=args.volts, amps=args.amps, erpm=erpm, temp_c=args.temp,
                         consumption_mah=args.consumption, throttle_pct=args.throttle,
                         pwm_pct=args.throttle)
    for key, value in (("bec_volts", args.bec), ("bec_temp_c", args.temp - 6),
                       ("motor_temp_c", args.temp - 4)):
        if key in esc.telemetry:
            esc.telemetry[key] = value

    def say(text):
        if not args.quiet:
            print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    if restored:
        say(f"parameters restored from {restored} -- this ESC remembers being programmed")
    if overridden:
        say("parameters overridden at start: " + ", ".join(overridden))
    say(f"virtual {args.protocol} on {args.host}:{port} -- MSP signature "
        f"0x{esc.sig:02x}, {esc.msp_payload_length} parameter byte(s)")
    say(f"telemetry {args.volts:.1f} V, {args.amps:.1f} A, {erpm:.0f} eRPM, "
        f"{args.temp:.0f} C at {esc.rate_hz:g} Hz; the flight controller is quiet for its "
        f"own {esc.boot_delay_s:.0f}s boot delay first")

    # -- the far end, said WHILE the run is going -----------------------------------------
    #
    # The summary at the bottom of this file is printed when the process stops, and that is
    # too late for a caller that has to decide something mid-run -- *did the write reach the
    # ESC* is exactly the question a test asks before it presses anything else. So every
    # programming event is announced as it happens, on stdout, in a shape a reader can parse.
    # It is here rather than in `esc_wire.serve` because only the tool end knows whether
    # anyone is listening.
    #
    # What a write LOOKS like differs per protocol and is not flattened into a common shape:
    # HobbyWing V5 sends one 48-byte block, Scorpion and FlyRotor an addressed buffer, YGE
    # and the XDFly family a single indexed value. Printing all three as "a write" would
    # throw away the half that says WHAT was programmed.
    seen = {"writes": 0, "reset": 0}
    #: The FIRST of each kind, announced as it happens. Without it the only live evidence
    #: that the parameter dialogue is running at all is the `answered` lines above, which
    #: stop after a dozen events -- and on a protocol that pings every 480 ms those dozen are
    #: gone before the flight controller's boot delay is over. A reader then cannot tell a
    #: refused write from a wire nobody ever spoke on, and those are opposite findings.
    announced = set()

    def tell(text):
        """A witness line, and `--quiet` does not silence it.

        `say` is chatter and may be turned off; these four lines are the run's evidence, and
        a caller that passed `--quiet` for the pings would otherwise lose the write with them.
        """
        print(f"[{time.strftime('%H:%M:%S')}] {text}", flush=True)

    def describe(item):
        if isinstance(item, (bytes, bytearray)):
            return f"{len(item)} byte(s) {bytes(item).hex()}"
        if isinstance(item, tuple) and len(item) == 2:
            where, what = item
            if isinstance(what, (bytes, bytearray)):
                return f"addr 0x{where:02x}: {bytes(what).hex() or '(none)'}"
            return f"index {where} = {what}"
        return repr(item)

    def on_event(request, reply):
        if reply is None:
            say(f"unparseable request {request[:8].hex()}")
        elif not args.quiet and sum(esc.counts.values()) <= 12:
            say(f"answered {request[:6].hex()} with {len(reply)} byte(s)")
        for key, value in esc.counts.items():
            if value and key not in announced:
                announced.add(key)
                tell(f"first {key}")
        writes = getattr(esc, "writes", ())
        while seen["writes"] < len(writes):
            seen["writes"] += 1
            tell(f"WRITE #{seen['writes']}: {describe(writes[seen['writes'] - 1])}")
        resets = int(esc.counts.get("reset", 0) or 0)
        while seen["reset"] < resets:
            seen["reset"] += 1
            tell(f"RESET #{seen['reset']}")

    sock = socket.create_connection((args.host, port), timeout=5)
    sock.settimeout(None)
    stop = threading.Event()
    if args.seconds:
        threading.Timer(args.seconds, stop.set).start()
    try:
        mode = args.mode if args.mode != "auto" else getattr(esc, "default_mode", "auto")
        if esc_wire.serve(sock, esc, stop=stop, on_event=on_event, rate_hz=esc.rate_hz,
                          mode=mode) == "closed":
            say("the flight controller closed the ESC UART")
    except KeyboardInterrupt:
        pass
    finally:
        sock.close()

    if args.state:
        with open(args.state, "w", encoding="utf-8") as handle:
            json.dump({"protocol": args.protocol,
                       "payload": esc.msp_payload().hex(),
                       "named": {k: v for k, v in esc.named().items()}},
                      handle, indent=2)
        say(f"parameters saved to {args.state}")

    print("\n" + ", ".join(f"{key} {value}" for key, value in esc.counts.items()))
    for write in esc.writes:
        print(f"  programmed {write}")
    return 0


if __name__ == "__main__":
    sys.exit(main() or 0)
