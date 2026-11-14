#!/usr/bin/env python3
"""Scenario tests on the SITL that need a rotor but no flight model.

    python3 src/utils/sitl/scenarios.py obj/main/rotorflight_SITL.elf [scenario ...]

The heli sits on the ground, level. A rotor model turns the SITL's main motor output into
headspeed, fed back as FDM `motor_erpm[0]` (frequency sensor, 2 poles, gear 1:1):

    dw/dt = A u V - B w - w^2 (k0 + k1 c^2)

a DC motor against rotor drag, with the constants fitted from a 200-size direct-drive
heli's blackbox logs (pack V = 16 V, collective c = 0.2 on the ground).

Scenarios:
  governor   ELECTRIC governor spool-up to gov_headspeed: reaches it within the spool-up
             time, overshoot under 10 %, steady-state error under 1 %
  profile    PID profile switch while spinning (MSP_SELECT_SETTING): the headspeed follows
             the new profile's gov_headspeed
  failsafe   RC lost while spinning: RXLOSS, throttle cut by the rxfail values, still armed
             (Rotorflight keeps stage-2 failsafe off); link back: the governor recovers
  rpmsensor  RPM sensor wiring check (MSP_FREQ_SENSOR_STATUS) with the shaft turned slowly by
             hand, disarmed: port 1 active, edges count at the shaft frequency, the pin
             toggles; no edges once the shaft stops
"""

import importlib.util
import math
import os
import socket
import struct
import sys
import tempfile
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
spec = importlib.util.spec_from_file_location("smoke", os.path.join(HERE, "..", "sitl-smoke-test.py"))
smoke = importlib.util.module_from_spec(spec)
spec.loader.exec_module(smoke)

MSP_SELECT_SETTING = 210
MSP_FREQ_SENSOR_STATUS = 168
G = 9.80665
# rotor law (dw/dt in rad/s^2; u 0..1, V volts, w rad/s, c collective -1..1)
A, B, K0, K1 = 987.7, 16.84, 7.0e-4, 1.10e-2
PACK_V, COLLECTIVE = 16.0, 0.2


class Rotor(threading.Thread):
    """FDM feed at ~500 Hz with the rotor model closed around the SITL's motor output."""

    def __init__(self):
        super().__init__(daemon=True)
        self.running = True
        self.w = 0.0
        self.hold_rpm = None    # turn the shaft at this rpm regardless of the motor
        self.throttle = 0.0
        self.out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.out.bind(("127.0.0.1", 9002))
        self.out.setblocking(False)
        self.fdm = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    @property
    def rpm(self):
        return self.w * 60 / (2 * math.pi)

    def run(self):
        t0 = last = time.monotonic()
        n = 0
        while self.running:
            try:
                while True:
                    d = self.out.recv(256)
                    if len(d) >= 16:
                        self.throttle = max(0.0, min(1.0, struct.unpack_from("<f", d, 0)[0]))
            except (BlockingIOError, OSError):
                pass
            now = time.monotonic()
            dt = min(now - last, 0.02)
            last = now
            u = self.throttle if self.throttle > 0.02 else 0.0
            drive = max(0.0, A * u * PACK_V - B * self.w) if u else 0.0
            self.w = max(0.0, self.w + (drive - self.w ** 2 * (K0 + K1 * COLLECTIVE ** 2)) * dt)
            if self.hold_rpm is not None:
                self.w = self.hold_rpm * 2 * math.pi / 60
            pkt = struct.pack("<21d", now - t0, 0, 0, 0, 0, 0, -G, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                              self.rpm, 0, 0, 0)        # 168 bytes: + motor_erpm[4]
            self.fdm.sendto(pkt, ("127.0.0.1", 9003))
            n += 1
            sleep = t0 + n * 0.002 - time.monotonic()
            if sleep > 0:
                time.sleep(sleep)

    def stop(self):
        self.running = False
        self.out.close()


class Pilot(threading.Thread):
    """MSP RX at 50 Hz: throttle, arm switch (AUX1), and whether the link is up."""

    def __init__(self, msp):
        super().__init__(daemon=True)
        self.msp, self.lock = msp, threading.Lock()
        self.throttle, self.arm, self.link = 1000, 1000, True
        self.running = True

    def run(self):
        while self.running:
            if self.link:
                with self.lock:
                    self.msp.rc(smoke.rc_frame(self.throttle, self.arm))
            time.sleep(0.02)

    def request(self, cmd, payload=b""):
        with self.lock:
            return self.msp.request(cmd, payload)

    def status(self):
        with self.lock:
            return self.msp.status()


def start(elf, commands):
    rundir = tempfile.mkdtemp(prefix="sitl-scenario-")
    s = smoke.Sitl(elf, rundir, "1-first"); s.wait(30); s.stop()
    s = smoke.Sitl(elf, rundir, "2-config")
    sock = smoke.connect_uart1(s)
    smoke.cli(sock, ["aux 0 0 0 1700 2100 0 0"] + commands + ["save"])
    sock.close(); s.wait(15); s.stop()
    rotor = Rotor(); rotor.start()
    s = smoke.Sitl(elf, rundir, "3-run")
    msp = smoke.Msp(smoke.connect_uart1(s))
    pilot = Pilot(msp); pilot.start()
    msp_lock_calibrate(pilot)
    return s, rotor, pilot


def msp_lock_calibrate(pilot):
    pilot.request(smoke.MSP_ACC_CALIBRATION)
    end = time.time() + 30
    while time.time() < end:
        armed, flags = pilot.status()
        if not flags:
            return
        time.sleep(0.2)
    raise smoke.Failure("arming blockers did not clear: %s" % flags)


def arm(pilot):
    pilot.arm = 1900
    end = time.time() + 5
    while time.time() < end:
        armed, flags = pilot.status()
        if armed:
            return
        time.sleep(0.1)
    raise smoke.Failure("did not arm: %s" % flags)


def stop(s, rotor, pilot):
    pilot.running = False
    rotor.stop()
    s.stop()


GOV = ["feature GOVERNOR", "set gov_mode = ELECTRIC", "set gov_spoolup_time = 50",
       "set motor_poles = 2"]


def spool(rotor, target, seconds):
    """Samples (t, rpm) for `seconds`; returns them."""
    out, t0 = [], time.time()
    while time.time() - t0 < seconds:
        out.append((time.time() - t0, rotor.rpm))
        time.sleep(0.05)
    return out


def scenario_governor(elf):
    target = 3000
    s, rotor, pilot = start(elf, GOV + ["set gov_headspeed = %d" % target])
    try:
        arm(pilot)
        pilot.throttle = 2000
        trace = spool(rotor, target, 12)
        reached = next((t for t, r in trace if r >= 0.95 * target), None)
        peak = max(r for _, r in trace)
        tail = [r for t, r in trace if t >= trace[-1][0] - 3]
        err = max(abs(r - target) for r in tail) / target
        if reached is None or reached > 8.0:
            raise smoke.Failure("governor: 95 %% of %d rpm not reached within 8 s (peak %.0f)" % (target, peak))
        if peak > 1.10 * target:
            raise smoke.Failure("governor: overshoot to %.0f rpm" % peak)
        if err > 0.01:
            raise smoke.Failure("governor: last 3 s off target by up to %.1f %%" % (100 * err))
        smoke.log("PASS governor: %d rpm reached in %.1f s, peak %.0f, steady within %.2f %%"
                  % (target, reached, peak, 100 * err))
    finally:
        pilot.throttle = 1000
        stop(s, rotor, pilot)


def scenario_profile(elf):
    hs = (3000, 2400)
    s, rotor, pilot = start(elf, GOV + ["profile 1", "set gov_headspeed = %d" % hs[1],
                                        "profile 0", "set gov_headspeed = %d" % hs[0]])
    try:
        arm(pilot)
        pilot.throttle = 2000
        spool(rotor, hs[0], 10)
        before = rotor.rpm
        pilot.request(MSP_SELECT_SETTING, bytes([1]))
        trace = spool(rotor, hs[1], 8)
        after = sum(r for t, r in trace if t > 6) / max(1, sum(1 for t, _ in trace if t > 6))
        if abs(before - hs[0]) > 0.02 * hs[0] or abs(after - hs[1]) > 0.02 * hs[1]:
            raise smoke.Failure("profile: headspeed %.0f -> %.0f rpm, expected %d -> %d"
                                % (before, after, hs[0], hs[1]))
        smoke.log("PASS profile switch: headspeed %.0f -> %.0f rpm (PID profiles 1 and 2 set to %d, %d)"
                  % (before, after, hs[0], hs[1]))
    finally:
        pilot.throttle = 1000
        stop(s, rotor, pilot)


def scenario_failsafe(elf):
    """Rotorflight keeps stage-2 failsafe off (failsafeStartMonitoring() is disabled), so a
    lost link never disarms: after 300 ms each channel takes its rxfail value. With the
    defaults that cuts the throttle - the motor stops, the rotor winds down, the heli
    stays armed - and when the link is back the governor brings the rotor back."""
    target = 3000
    s, rotor, pilot = start(elf, GOV + ["set gov_headspeed = %d" % target])
    try:
        arm(pilot)
        pilot.throttle = 2000
        spool(rotor, target, 10)
        before = rotor.rpm
        pilot.link = False                                  # the radio goes quiet
        t0 = time.time()
        flagged = None
        while time.time() - t0 < 4:
            armed, flags = pilot.status()
            if flagged is None and "RXLOSS" in flags:
                flagged = time.time() - t0
            time.sleep(0.1)
        cut, lost_rpm = rotor.throttle, rotor.rpm
        if flagged is None or flagged > 1.0:
            raise smoke.Failure("failsafe: RXLOSS not flagged within 1 s")
        if cut > 0.05 or lost_rpm > 0.8 * before:
            raise smoke.Failure("failsafe: throttle %.2f, rotor %.0f rpm 4 s after the loss" % (cut, lost_rpm))
        if not armed:
            raise smoke.Failure("failsafe: disarmed - Rotorflight keeps stage 2 off, it should not")
        smoke.log("PASS failsafe: RXLOSS after %.1f s, throttle cut, rotor %.0f -> %.0f rpm in 4 s, still armed"
                  % (flagged, before, lost_rpm))
        pilot.link = True                                   # link back, sticks as before
        trace = spool(rotor, target, 10)
        back = next((t for t, r in trace if r >= 0.95 * target), None)
        if back is None:
            raise smoke.Failure("failsafe: rotor not back to %d rpm within 10 s of the link returning" % target)
        smoke.log("PASS failsafe: link back, governor recovered %d rpm in %.1f s" % (target, back))
    finally:
        pilot.throttle = 1000
        stop(s, rotor, pilot)


def freq_status(pilot):
    """Port 1 of MSP_FREQ_SENSOR_STATUS: (active, pin high, edge count)."""
    d = pilot.request(MSP_FREQ_SENSOR_STATUS)
    if len(d) < 4 or d[0] < 1:
        raise smoke.Failure("rpmsensor: no frequency sensor ports reported (%r)" % d)
    flags, edges = d[1], struct.unpack_from("<H", d, 2)[0]
    return bool(flags & 1), bool(flags & 2), edges


def scenario_rpmsensor(elf):
    hz = 2.0
    s, rotor, pilot = start(elf, ["feature FREQ_SENSOR"])
    try:
        rotor.hold_rpm = hz * 60
        time.sleep(1)
        active, _, e0 = freq_status(pilot)
        if not active:
            raise smoke.Failure("rpmsensor: port 1 not active")
        t0, pins = time.time(), set()
        while time.time() - t0 < 5:
            pins.add(freq_status(pilot)[1])
            time.sleep(0.05)
        _, _, e1 = freq_status(pilot)
        elapsed = time.time() - t0
        rate = ((e1 - e0) & 0xFFFF) / elapsed
        if abs(rate - hz) > 0.25 * hz:
            raise smoke.Failure("rpmsensor: %.2f edges/s at a %.1f Hz shaft" % (rate, hz))
        if pins != {False, True}:
            raise smoke.Failure("rpmsensor: pin state never changed (%s)" % pins)
        rotor.hold_rpm = 0
        time.sleep(1)
        _, _, e2 = freq_status(pilot)
        time.sleep(2)
        _, _, e3 = freq_status(pilot)
        if e3 != e2:
            raise smoke.Failure("rpmsensor: %d edges with the shaft stopped" % ((e3 - e2) & 0xFFFF))
        smoke.log("PASS rpmsensor: %.2f edges/s at %.1f Hz, pin toggles, none when stopped" % (rate, hz))
    finally:
        stop(s, rotor, pilot)


SCENARIOS = {"governor": scenario_governor, "profile": scenario_profile, "failsafe": scenario_failsafe,
             "rpmsensor": scenario_rpmsensor}


def main():
    elf = os.path.abspath(sys.argv[1])
    names = sys.argv[2:] or list(SCENARIOS)
    failed = 0
    for name in names:
        try:
            SCENARIOS[name](elf)
        except smoke.Failure as e:
            smoke.log("FAIL %s" % e)
            failed += 1
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
