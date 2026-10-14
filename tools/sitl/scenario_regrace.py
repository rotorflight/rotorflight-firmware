#!/usr/bin/env python3
"""Scenario: change a PID gain while armed, then re-arm inside the blackbox
grace period.

Needs a configured SITL (tools/sitl/configure.py), tools/sitl/sim_feed.py
and tools/sitl/capture.py. Steps:

  1. Wait until only the arm switch blocks arming. Set a roll rate error:
     roll stick --roll-stick us (setpoint) with zero gyro, so that the
     P term is P gain x (setpoint - gyro) all the time.
  2. Arm (AUX1 high; if LOAD blocks the arm, switch low and try again),
     wait --armed1 s.
  3. While armed: read the PID gains, write roll P = --new-p (MSP_SET_PID_TUNING,
     applied at once by pidLoadProfile()), read them back. Wait --after-change s.
  4. Wait until the real-time load is low (else LOAD can block the re-arm),
     disarm, wait --gap s (less than the grace period), re-arm: the log
     continues, without a new header.
  5. Wait --armed2 s, disarm, wait grace + --tail s so that the blackbox
     writes LOG_END. Restore the stick.

Prints a JSON timeline: for each step the CLOCK_MONOTONIC time, and the SITL
time (micros since the SITL start, from the "CLOCK_MONOTONIC start" line of
--sitl-log; the SITL clock runs at simRate ~1.0).
"""

import argparse
import json
import re
import sys
import time

from msp import MSP
from sim_feed import simctl

AUX1 = 5           # RC channel index of AUX1 (rcmap AETRC123)
ROLL = 0


class Timeline:
    def __init__(self, sitl_start):
        self.sitl_start = sitl_start
        self.events = []

    def add(self, event, **data):
        t = time.monotonic()
        rec = {'event': event, 't_monotonic': round(t, 6)}
        if self.sitl_start is not None:
            rec['t_sitl_us'] = int((t - self.sitl_start) * 1e6)
        rec.update(data)
        self.events.append(rec)
        print('scenario: %-16s %s' % (event, json.dumps(data) if data else ''), file=sys.stderr, flush=True)
        return rec


def sitl_start_time(path):
    if not path:
        return None
    for _ in range(50):
        try:
            with open(path) as f:
                m = re.search(r'CLOCK_MONOTONIC start (\d+)\.(\d+)', f.read())
            if m:
                return int(m.group(1)) + int(m.group(2)) / 1e9
        except OSError:
            pass
        time.sleep(0.1)
    return None


def wait_armed(fc, armed, timeout=5.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        st = fc.status()
        if st['armed'] == armed:
            return st
        time.sleep(0.02)
    raise SystemExit('scenario: the SITL did not %s in %.1f s, arming disabled by %s'
                     % ('arm' if armed else 'disarm', timeout, st['arming_disabled_by']))


def arm(fc, ctl, timeout=4.0):
    """Arm with the AUX1 switch. Return the arming flags of the failed attempts.

    A host stall of about 0.1 ms or more in the real-time tasks sets LOAD
    (fc/core.c: max real-time load > 75 %), and the flag stays for a few
    seconds. If LOAD is set when the switch goes high, the FC sets ARM_SWITCH
    and does not arm. Then put the switch low and try again."""
    deadline = time.monotonic() + timeout
    failed = []
    while time.monotonic() < deadline:
        st = fc.status()
        if [f for f in st['arming_disabled_by'] if f != 'ARM_SWITCH']:
            time.sleep(0.02)
            continue
        ctl({'rc': {str(AUX1): 1900}})
        until = min(deadline, time.monotonic() + 0.3)
        while time.monotonic() < until:
            st = fc.status()
            if st['armed']:
                return failed
            time.sleep(0.02)
        failed.append(st['arming_disabled_by'])
        ctl({'rc': {str(AUX1): 1000}})
        time.sleep(0.05)
    raise SystemExit('scenario: the SITL did not arm in %.1f s, failed attempts %s, arming disabled by %s'
                     % (timeout, failed, fc.status()['arming_disabled_by']))


def wait_low_load(fc, timeout=10.0, limit=600):
    """Wait until the max real-time load is below the LOAD limit (750), so
    that a re-arm soon after the next disarm is not blocked. The FC updates
    the LOAD flag only while disarmed, so read the load. Return the wait, s."""
    t0 = time.monotonic()
    while fc.status()['max_realtime_load'] > limit and time.monotonic() - t0 < timeout:
        time.sleep(0.05)
    return round(time.monotonic() - t0, 2)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--msp-port', type=int, default=5761)
    ap.add_argument('--ctl-port', type=int, default=9010)
    ap.add_argument('--sitl-log', help='SITL stdout file, for the SITL start time')
    ap.add_argument('--roll-stick', type=int, default=1600, help='roll channel during the test, us')
    ap.add_argument('--new-p', type=int, default=0, help='new roll P (0: twice the old value)')
    ap.add_argument('--armed1', type=float, default=3.0)
    ap.add_argument('--after-change', type=float, default=2.0)
    ap.add_argument('--gap', type=float, default=1.0, help='disarmed time before the re-arm, s')
    ap.add_argument('--armed2', type=float, default=3.0)
    ap.add_argument('--tail', type=float, default=3.0, help='wait after grace period + this, s')
    ap.add_argument('--output', help='also write the timeline JSON here')
    a = ap.parse_args()

    tl = Timeline(sitl_start_time(a.sitl_log))
    ctl = lambda req: simctl(req, port=a.ctl_port)

    with MSP(port=a.msp_port, timeout=2.0, connect_timeout=20.0) as fc:
        info = dict(fc.api_version(), **fc.fc_version())
        bb = fc.blackbox_config()
        grace = bb['grace_period']

        # 1. ready: only ARM_SWITCH (or nothing) may block arming
        deadline = time.monotonic() + 30
        while True:
            st = fc.status()
            blockers = [f for f in st['arming_disabled_by'] if f != 'ARM_SWITCH']
            if not blockers:
                break
            if time.monotonic() > deadline:
                raise SystemExit('scenario: arming still disabled by %s' % blockers)
            time.sleep(0.2)
        tl.add('ready', fc=info, blackbox=bb, arming_disabled_by=st['arming_disabled_by'],
               pid_profile=st['pid_profile'] + 1, pid_cycle_us=st['pid_cycle_us'])
        ctl({'rc': {str(ROLL): a.roll_stick}, 'gyro_dps': [0, 0, 0]})
        tl.add('roll_stick', us=a.roll_stick)
        time.sleep(0.5)

        # 2. arm
        tl.add('arm_cmd')
        tl.add('armed', failed_attempts=arm(fc, ctl))
        time.sleep(a.armed1)

        # 3. change roll P while armed
        before = fc.pid_tuning()
        new = json.loads(json.dumps(before))
        old_p = before['roll']['P']
        new['roll']['P'] = a.new_p or old_p * 2
        tl.add('pid_write_cmd', roll_p_before=old_p, roll_p_new=new['roll']['P'])
        fc.set_pid_tuning(new)
        tl.add('pid_written')
        after = fc.pid_tuning()
        tl.add('pid_readback', roll_p=after['roll']['P'], pid=after,
               armed=fc.status()['armed'])
        time.sleep(a.after_change)

        # 4. disarm, re-arm inside the grace period
        tl.add('disarm_cmd', load_wait_s=wait_low_load(fc))
        ctl({'rc': {str(AUX1): 1000}})
        wait_armed(fc, False)
        disarmed = tl.add('disarmed')
        time.sleep(a.gap)
        tl.add('rearm_cmd')
        rearm_failed = arm(fc, ctl, timeout=max(0.5, grace - a.gap - 1.0))
        rearmed = tl.add('rearmed', failed_attempts=rearm_failed)
        if rearmed['t_monotonic'] - disarmed['t_monotonic'] >= grace:
            raise SystemExit('scenario: the re-arm came %.2f s after the disarm, not inside the %d s grace period'
                             % (rearmed['t_monotonic'] - disarmed['t_monotonic'], grace))

        # 5. disarm, wait for the end of the grace period (LOG_END)
        time.sleep(a.armed2)
        tl.add('disarm2_cmd')
        ctl({'rc': {str(AUX1): 1000}})
        wait_armed(fc, False)
        tl.add('disarmed2')
        time.sleep(grace + a.tail)
        ctl({'rc': {str(ROLL): 1500}})
        tl.add('done', roll_p=fc.pid_tuning()['roll']['P'], arming_disabled_by=fc.status()['arming_disabled_by'])

    result = {'scenario': 'regrace', 'grace_period_s': grace, 'sitl_start_monotonic': tl.sitl_start,
              'events': tl.events}
    text = json.dumps(result, indent=1)
    if a.output:
        with open(a.output, 'w') as f:
            f.write(text + '\n')
    print(text)


if __name__ == '__main__':
    main()
