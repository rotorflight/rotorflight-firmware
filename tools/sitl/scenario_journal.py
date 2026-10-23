#!/usr/bin/env python3
"""Scenario: the writes that the blackbox parameter journal (blackbox_params,
RF-PARAM-1) must record, with a ground-truth timeline.

Needs a configured SITL (tools/sitl/configure.py), tools/sitl/sim_feed.py and
tools/sitl/capture.py (run it with tools/sitl/run_all.sh -s journal). Roll and
yaw sticks off centre and zero gyro: the logged axisP[0] and axisP[2] are the
roll and yaw P gains in force times a constant rate error.

Log 1 (default plan):
  1. Arm. Wait --armed1 s.
  2. While armed: roll P x2 (MSP_SET_PID_TUNING). Then --toggles more roll P
     writes (alternate between the two values, random spacing) to sample the
     PID ticks of the write points.
  3. While armed: MSP_SELECT_SETTING to PID profile 2, then back to PID profile 1.
  4. Disarm. In the grace period: yaw P + 40 (MSP_SET_PID_TUNING), re-arm less
     than 1 s after the disarm.
  5. Wait --armed2 s, disarm. In the grace period: MSP_EEPROM_WRITE.
  6. Wait for the end of the grace period (LOG_END).
--between (log 2 in the same boot): with no log open, roll P, pitch D, rate
  profile 2, PID profile 2, roll P of PID profile 2. Then arm (log 2: 'v'
  records), back to PID profile 1, roll P, disarm, wait for LOG_END.
  --log2-idle: log 2 has no writes.
--kill-ms D: after step 2, select PID profile 2 and kill -9 the SITL
  (SITL_PID) D ms after the reply. No disarm, no LOG_END.

Each write is in the timeline with 'write': host CLOCK_MONOTONIC before the
command and after the reply, the SITL time (t_sitl_us), the MSP_STATUS just
before it (cycle time, arm state, profiles), the read-back after it, and
'truth': the journal keys that it changes, {key: [old, new]} (record key
names of the spec, 2.4).
"""

import argparse
import json
import os
import random
import signal
import struct
import sys
import time

from msp import MSP
from sim_feed import simctl
from scenario_regrace import Timeline, sitl_start_time, wait_armed, arm, wait_low_load

AUX1 = 5           # RC channel index of AUX1 (rcmap AETRC123)
ROLL, PITCH, YAW = 0, 1, 3
AXIS_KEY = {'roll': 'roll', 'pitch': 'pitch', 'yaw': 'yaw'}
TERM_KEY = {'P': 'p_gain', 'I': 'i_gain', 'D': 'd_gain', 'F': 'f_gain'}


class Scenario:
    def __init__(self, fc, ctl, tl):
        self.fc, self.ctl, self.tl = fc, ctl, tl

    def snapshot(self):
        st = self.fc.status()
        return {'pid_cycle_us': st['pid_cycle_us'], 'armed': st['armed'], 'pid_profile': st['pid_profile'],
                'rate_profile': st['rate_profile'], 'max_realtime_load': st['max_realtime_load']}

    def write(self, name, fn, truth, readback=None):
        """One write: status, host time before and after, read-back. truth: {key: [old, new]}."""
        before = self.snapshot()
        t0 = time.monotonic()
        fn()
        t1 = time.monotonic()
        rb = readback() if readback else None
        t2 = time.monotonic()
        rec = self.tl.add(name, write=True, truth=truth, status=before, readback=rb,
                          t_send=round(t0, 6), t_reply=round(t1, 6), t_readback=round(t2, 6))
        if self.tl.sitl_start is not None:
            rec['t_sitl_send_us'] = int((t0 - self.tl.sitl_start) * 1e6)
            rec['t_sitl_reply_us'] = int((t1 - self.tl.sitl_start) * 1e6)
        return rec

    def set_gain(self, axis, term, value):
        pid = self.fc.pid_tuning()
        prof = self.fc.status()['pid_profile']
        old = pid[axis][term]
        pid[axis][term] = value
        key = 'p%d.%s_%s' % (prof, AXIS_KEY[axis], TERM_KEY[term])

        def readback():
            got = self.fc.pid_tuning()
            return {'pid_profile': self.fc.status()['pid_profile'], axis + '.' + term: got[axis][term]}
        return self.write('set_%s_%s' % (axis, term), lambda: self.fc.set_pid_tuning(pid),
                          {key: [str(old), str(value)]} if old != value else {}, readback)

    def select_pid(self, index):
        old = self.fc.status()['pid_profile']
        return self.write('select_pid_profile', lambda: self.fc.select_pid_profile(index),
                          {'pid_profile': [str(old), str(index)]} if old != index else {},
                          lambda: dict(self.snapshot(), roll_p=self.fc.pid_tuning()['roll']['P']))

    def select_rate(self, index):
        old = self.fc.status()['rate_profile']
        return self.write('select_rate_profile', lambda: self.fc.select_rate_profile(index),
                          {'rate_profile': [str(old), str(index)]} if old != index else {},
                          self.snapshot)

    def eeprom_write(self):
        return self.write('eeprom_write', self.fc.eeprom_write, {}, self.snapshot)

    def extra_commands(self):
        # The readback is independent of the journal decoder.
        def pid_truth(old, new):
            return {'p0.%s_%s' % (axis, TERM_KEY[term]): [str(old[axis][term]), str(new[axis][term])]
                    for axis in AXIS_KEY for term in TERM_KEY if old[axis][term] != new[axis][term]}
        old = self.fc.pid_tuning()
        default = json.loads(json.dumps(old))
        default['roll']['P'] = 50
        self.write('copy_active_pid', lambda: self.fc.request(183, bytes([0, 0, 1])), pid_truth(old, default), self.fc.pid_tuning)
        assert self.fc.pid_tuning() == default
        time.sleep(0.5)
        self.set_gain('roll', 'P', 75)
        time.sleep(0.5)
        old = self.fc.pid_tuning()
        self.write('reset_active_pid', lambda: self.fc.request(219), pid_truth(old, default), self.fc.pid_tuning)
        assert self.fc.pid_tuning() == default
        time.sleep(0.5)
        # Stored governor configuration is boot-cached. Change one numeric field.
        gov = bytearray(self.fc.request(142))
        before = struct.unpack_from('<H', gov, 1)[0]
        struct.pack_into('<H', gov, 1, before + 1)
        self.write('set_governor_config', lambda: self.fc.request(143, gov), {'gov_startup_time': [str(before), str(before + 1)]}, lambda: list(self.fc.request(142)))
        assert self.fc.request(142) == bytes(gov)
        time.sleep(0.5)
        servo = self.fc.request(125, bytes([0]))
        values = list(struct.unpack('<HhhHHHHH', servo))
        new = values.copy(); new[0] += 5
        payload = bytes([0]) + struct.pack('<HhhHHHHH', *new)
        self.write('set_servo_with_arg', lambda: self.fc.request(124, payload), {'el.servo.0': [','.join(map(str, values)), ','.join(map(str, new))]}, lambda: list(self.fc.request(125, bytes([0]))))
        assert self.fc.request(125, bytes([0])) == payload[1:]
        time.sleep(0.5)
        filt = bytearray(self.fc.request(92))
        before = struct.unpack_from('<H', filt, 2)[0]
        struct.pack_into('<H', filt, 2, before + 5)
        self.write('set_filter_config', lambda: self.fc.request(93, filt), {'gyro_lpf1_static_hz': [str(before), str(before + 5)]}, lambda: list(self.fc.request(92)))
        assert struct.unpack_from('<H', self.fc.request(92), 2)[0] == before + 5
        time.sleep(0.5)

    def adjustment(self, function, low, high, key, getter):
        self.ctl({'rc': {'7': 2000, '8': 1000}})
        time.sleep(0.1)
        payload = struct.pack('<10BHHB', 0, function, 3, 40, 120, 2, 156, 100, 0, 0, low, high, 0)
        self.fc.request(53, payload)
        old = getter()
        target = high - 1 if key == 'pid_profile' else high
        def apply():
            self.ctl({'rc': {'8': 1900}})
            deadline = time.monotonic() + 3
            while getter() != target:
                if time.monotonic() > deadline: raise RuntimeError('Adjustment readback timeout: ' + key)
                time.sleep(0.02)
        rec = self.write('adjustment_' + str(function), apply, {key: [str(old), str(target)]}, getter)
        rec['journal_source'] = 'a.' + str(function)
        self.ctl({'rc': {'8': 1000}})
        time.sleep(0.3)

    def extra_adjustments(self):
        self.adjustment(18, 50, 60, 'p0.roll_p_gain', lambda: self.fc.pid_tuning()['roll']['P'])
        self.adjustment(69, 0, 100, 'r0.setpoint_boost_gain[0]', lambda: self.fc.request(111)[25])
        self.adjustment(2, 1, 2, 'pid_profile', lambda: self.fc.status()['pid_profile'])
        self.select_pid(0)
        # One operation with enough items for continuation events.
        old = self.fc.pid_tuning(); new = json.loads(json.dumps(old))
        # Separate old/new P outputs by more than integer Blackbox quantization.
        for axis in AXIS_KEY:
            for term in TERM_KEY: new[axis][term] += 10
        truth = {'p0.%s_%s' % (axis, TERM_KEY[term]): [str(old[axis][term]), str(new[axis][term])]
                 for axis in AXIS_KEY for term in TERM_KEY}
        self.write('set_many_gains', lambda: self.fc.set_pid_tuning(new), truth, self.fc.pid_tuning)
        assert self.fc.pid_tuning() == new
        time.sleep(0.5)

    def arm(self, timeout=4.0):
        self.tl.add('arm_cmd')
        return self.tl.add('armed', failed_attempts=arm(self.fc, self.ctl, timeout=timeout))

    def disarm(self):
        self.tl.add('disarm_cmd', load_wait_s=wait_low_load(self.fc))
        self.ctl({'rc': {str(AUX1): 1000}})
        wait_armed(self.fc, False)
        return self.tl.add('disarmed')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--msp-port', type=int, default=5761)
    ap.add_argument('--ctl-port', type=int, default=9010)
    ap.add_argument('--sitl-log', help='SITL stdout file, for the SITL start time')
    ap.add_argument('--roll-stick', type=int, default=1700)
    ap.add_argument('--yaw-stick', type=int, default=1650)
    ap.add_argument('--armed1', type=float, default=2.0)
    ap.add_argument('--toggles', type=int, default=0, help='more roll P writes while armed')
    ap.add_argument('--step', type=float, default=1.5, help='wait after each write, s')
    ap.add_argument('--gap', type=float, default=0.6, help='disarm to re-arm, s (less than 1)')
    ap.add_argument('--armed2', type=float, default=2.0)
    ap.add_argument('--tail', type=float, default=2.5, help='wait after grace period + this, s')
    ap.add_argument('--between', action='store_true', help='writes between logs, then log 2')
    ap.add_argument('--log2-idle', action='store_true', help='log 2 without writes (armed --armed1 + 2 --step)')
    ap.add_argument('--kill-ms', type=float, default=None, help='kill -9 the SITL this many ms after a write')
    ap.add_argument('--cli-end', action='store_true')
    ap.add_argument('--extended', action='store_true')
    ap.add_argument('--header-write', action='store_true')
    ap.add_argument('--seed', type=int, default=1)
    ap.add_argument('--output', help='also write the timeline JSON here')
    a = ap.parse_args()
    rnd = random.Random(a.seed)

    tl = Timeline(sitl_start_time(a.sitl_log))
    ctl = lambda req: simctl(req, port=a.ctl_port)
    result = {'scenario': 'journal', 'args': vars(a)}

    def finish():
        result['sitl_start_monotonic'] = tl.sitl_start
        result['events'] = tl.events
        text = json.dumps(result, indent=1)
        if a.output:
            with open(a.output, 'w') as f:
                f.write(text + '\n')
        print(text)

    with MSP(port=a.msp_port, timeout=2.0, connect_timeout=20.0) as fc:
        sc = Scenario(fc, ctl, tl)
        info = dict(fc.api_version(), **fc.fc_version())
        bb = fc.blackbox_config()
        grace = bb['grace_period']
        result['grace_period_s'] = grace

        deadline = time.monotonic() + 30
        while True:
            st = fc.status()
            blockers = [f for f in st['arming_disabled_by'] if f != 'ARM_SWITCH']
            if not blockers:
                break
            if time.monotonic() > deadline:
                raise SystemExit('scenario: arming still disabled by %s' % blockers)
            time.sleep(0.2)
        tl.add('ready', fc=info, blackbox=bb, status=sc.snapshot(), pid=fc.pid_tuning())
        ctl({'rc': {str(ROLL): a.roll_stick, str(YAW): a.yaw_stick}, 'gyro_dps': [0, 0, 0]})
        tl.add('sticks', roll=a.roll_stick, yaw=a.yaw_stick)
        time.sleep(0.5)

        # log 1
        sc.arm()
        if a.header_write:
            sc.set_gain('roll', 'I', fc.pid_tuning()['roll']['I'] + 1)['expected_source'] = 'm'
        time.sleep(a.armed1)
        p0 = fc.pid_tuning()['roll']['P']
        values = [p0 * 2, p0]
        sc.set_gain('roll', 'P', values[0])
        time.sleep(a.step)
        for i in range(a.toggles):
            sc.set_gain('roll', 'P', values[(i + 1) % 2])
            time.sleep(rnd.uniform(0.15, 0.45))
        if a.toggles % 2:
            sc.set_gain('roll', 'P', values[0])
            time.sleep(0.3)

        if a.kill_ms is not None:
            pid = int(os.environ['SITL_PID'])
            # no read-back: the kill comes --kill-ms after the reply
            rec = sc.write('select_pid_profile', lambda: fc.select_pid_profile(1), {'pid_profile': ['0', '1']})
            delay = a.kill_ms / 1000.0
            while time.monotonic() < rec['t_reply'] + delay:
                pass
            os.kill(pid, signal.SIGKILL)
            tl.add('killed', pid=pid, delay_ms=a.kill_ms,
                   after_reply_ms=round((time.monotonic() - rec['t_reply']) * 1000, 3))
            finish()
            return

        if a.extended:
            sc.extra_commands()
            sc.extra_adjustments()
        sc.select_pid(1)
        time.sleep(a.step)
        sc.select_pid(0)
        time.sleep(a.step)

        disarmed = sc.disarm()
        sc.set_gain('yaw', 'P', fc.pid_tuning()['yaw']['P'] + 40)
        while time.monotonic() < disarmed['t_monotonic'] + a.gap:
            time.sleep(0.005)
        rearmed = sc.arm(timeout=1.0)
        if rearmed['t_monotonic'] - disarmed['t_monotonic'] >= 1.0:
            raise SystemExit('scenario: the re-arm came %.2f s after the disarm, not within 1 s'
                             % (rearmed['t_monotonic'] - disarmed['t_monotonic']))
        time.sleep(a.armed2)
        sc.disarm()
        if a.cli_end:
            from configure import run_cli
            tl.add('cli_entry', output=run_cli(fc.sock, ['get roll_p_gain'], save=False))
            time.sleep(1.0)
            finish()
            return
        time.sleep(1.0)
        sc.eeprom_write()
        time.sleep(grace + a.tail)
        tl.add('log1_end_expected')

        if a.between:
            sc.set_gain('roll', 'P', fc.pid_tuning()['roll']['P'] + 20)
            sc.set_gain('pitch', 'D', fc.pid_tuning()['pitch']['D'] + 5)
            sc.select_rate(1)
            sc.select_pid(1)
            sc.set_gain('roll', 'P', fc.pid_tuning()['roll']['P'] + 20)
            time.sleep(0.5)
            sc.arm()
            time.sleep(a.armed1)
            if a.log2_idle:
                time.sleep(2 * a.step)
            else:
                sc.select_pid(0)
                time.sleep(a.step)
                sc.set_gain('roll', 'P', 60)
                time.sleep(a.step)
            sc.disarm()
            time.sleep(grace + a.tail)
            tl.add('log2_end_expected')

        ctl({'rc': {str(ROLL): 1500, str(YAW): 1500}})
        tl.add('done', status=sc.snapshot(), pid=fc.pid_tuning())
    finish()


if __name__ == '__main__':
    main()
