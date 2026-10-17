#!/usr/bin/env python3
"""Sensor and RC feed for the Rotorflight SITL.

Sends to the SITL (src/main/target/SITL/target.c, target.h):
  - fdm_packet to UDP 9003 at --imu-rate Hz (default 1000, the fake gyro rate):
    a level, still craft. Accelerometer (0, 0, -9.80665) m/s^2 (NED, body
    frame: gravity on z), gyro 0 rad/s unless set, attitude quaternion (1,0,0,0).
    The timestamp is CLOCK_MONOTONIC in seconds, so the SITL clock follows
    real time.
  - rc_packet to UDP 9004 at --rc-rate Hz (default 100): 16 channels in us.
    The SITL gives them to the MSP receiver (FEATURE_RX_MSP).

Control API: JSON datagrams to UDP 127.0.0.1:--ctl-port (default 9010). Each
request gets a JSON reply with the state after the change:
  {"rc": {"5": 1900}}             set channels by index (0-based)
  {"rc": [1500, 1500, ...]}       set the first N channels
  {"gyro_dps": [r, p, y]}         constant body rates in deg/s (default 0)
  {"gyro_sine": {"axis": 0, "amp_dps": 30, "hz": 2}}   add a sine (null: none)
  {"get": true}                   only read the state
  {"quit": true}                  stop the feed

Default channels (rcmap "AETRC123", pg/rx.c): 0 roll, 1 pitch, 2 throttle,
3 yaw, 4 collective, 5 AUX1 (arm switch in tools/sitl/configure.py), 6 AUX2...
Roll, pitch, yaw, collective 1500, throttle 1000, AUX 1000.

Use simctl() from this module to send a control request from Python.
"""

import argparse
import json
import math
import signal
import socket
import struct
import sys
import time

FDM_FORMAT = '<17d'      # timestamp, gyro[3], accel[3], quat[4], vel[3], pos[3]
RC_FORMAT = '<d16H'      # timestamp, channels[16]
RC_CHANNELS = 16
GRAVITY = 9.80665


def default_channels():
    ch = [1500] * RC_CHANNELS
    ch[2] = 1000         # throttle
    for i in range(5, RC_CHANNELS):
        ch[i] = 1000     # AUX
    return ch


def simctl(request, port=9010, host='127.0.0.1', timeout=2.0):
    """Send one control request to a running sim_feed and return its reply."""
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.settimeout(timeout)
    try:
        s.sendto(json.dumps(request).encode(), (host, port))
        data, _ = s.recvfrom(65536)
        return json.loads(data)
    finally:
        s.close()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--host', default='127.0.0.1', help='SITL address')
    ap.add_argument('--fdm-port', type=int, default=9003)
    ap.add_argument('--rc-port', type=int, default=9004)
    ap.add_argument('--ctl-port', type=int, default=9010)
    ap.add_argument('--imu-rate', type=float, default=1000.0)
    ap.add_argument('--rc-rate', type=float, default=100.0)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()

    out = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    ctl.bind(('127.0.0.1', a.ctl_port))
    ctl.setblocking(False)

    state = {'rc': default_channels(), 'gyro_dps': [0.0, 0.0, 0.0], 'gyro_sine': None,
             'fdm_sent': 0, 'rc_sent': 0, 'late': 0}
    running = [True]

    def stop(*_):
        running[0] = False
    signal.signal(signal.SIGTERM, stop)
    signal.signal(signal.SIGINT, stop)

    def reply_state():
        return {'rc': state['rc'], 'gyro_dps': state['gyro_dps'], 'gyro_sine': state['gyro_sine'],
                'fdm_sent': state['fdm_sent'], 'rc_sent': state['rc_sent'], 'late': state['late'],
                't_monotonic': time.monotonic()}

    def handle(req):
        if 'rc' in req:
            rc = req['rc']
            if isinstance(rc, dict):
                for k, v in rc.items():
                    state['rc'][int(k)] = int(v)
            else:
                for i, v in enumerate(rc[:RC_CHANNELS]):
                    state['rc'][i] = int(v)
        if 'gyro_dps' in req:
            state['gyro_dps'] = [float(x) for x in req['gyro_dps']]
        if 'gyro_sine' in req:
            state['gyro_sine'] = req['gyro_sine']
        if req.get('quit'):
            running[0] = False

    imu_dt = 1.0 / a.imu_rate
    rc_dt = 1.0 / a.rc_rate
    t0 = time.monotonic()
    next_imu = t0
    next_rc = t0
    if not a.quiet:
        print('sim_feed: FDM %.0f Hz -> %s:%d, RC %.0f Hz -> %s:%d, control on UDP %d'
              % (a.imu_rate, a.host, a.fdm_port, a.rc_rate, a.host, a.rc_port, a.ctl_port), flush=True)

    while running[0]:
        now = time.monotonic()
        if now >= next_imu:
            if now - next_imu > 5 * imu_dt:
                state['late'] += 1
                next_imu = now
            gyro = list(state['gyro_dps'])
            sine = state['gyro_sine']
            if sine:
                gyro[int(sine.get('axis', 0))] += float(sine['amp_dps']) * math.sin(2 * math.pi * float(sine['hz']) * (now - t0))
            gyro_rad = [math.radians(g) for g in gyro]
            # target.c updateState(): gyro y and z are negated, accel all negated
            pkt = struct.pack(FDM_FORMAT, now,
                              gyro_rad[0], -gyro_rad[1], -gyro_rad[2],
                              0.0, 0.0, -GRAVITY,
                              1.0, 0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0,
                              0.0, 0.0, 0.0)
            out.sendto(pkt, (a.host, a.fdm_port))
            state['fdm_sent'] += 1
            next_imu += imu_dt
        if now >= next_rc:
            out.sendto(struct.pack(RC_FORMAT, now, *state['rc']), (a.host, a.rc_port))
            state['rc_sent'] += 1
            next_rc += rc_dt
            if now - next_rc > 5 * rc_dt:
                next_rc = now
        while True:
            try:
                data, addr = ctl.recvfrom(65536)
            except BlockingIOError:
                break
            try:
                handle(json.loads(data))
                reply = reply_state()
            except (ValueError, KeyError, TypeError, IndexError) as e:
                reply = {'error': str(e)}
            ctl.sendto(json.dumps(reply).encode(), addr)
        sleep = min(next_imu, next_rc) - time.monotonic()
        if sleep > 0.0002:
            time.sleep(sleep - 0.0001)

    if not a.quiet:
        print('sim_feed: stopped, %d FDM and %d RC packets sent, %d late resyncs'
              % (state['fdm_sent'], state['rc_sent'], state['late']), flush=True)


if __name__ == '__main__':
    sys.exit(main())
