#!/usr/bin/env python3
"""Disable and restart SWITCH logging; the paused ending log must not resume."""
import argparse, json, time
from msp import MSP
from sim_feed import simctl
from scenario_regrace import Timeline, sitl_start_time
from scenario_journal import Scenario
p = argparse.ArgumentParser(); p.add_argument('--sitl-log'); p.add_argument('--output'); a = p.parse_args()
tl = Timeline(sitl_start_time(a.sitl_log))
with MSP(connect_timeout=20) as fc:
    sc = Scenario(fc, simctl, tl)
    bb = fc.blackbox_config()
    fc.set_blackbox_config(3, 3, bb['denom'], bb['fields'], bb['initial_erase_kib'], bb['rolling_erase'], 5)
    fc.set_mode_range(1, 26, 1, 40, 120)
    time.sleep(4)
    for run in range(2):
        simctl({'rc': {'6': 1900}}); tl.add('switch_on', run=run)
        time.sleep(3)
        sc.set_gain('roll', 'P', 70 + run)
        simctl({'rc': {'6': 1000}}); tl.add('switch_off', run=run)
        time.sleep(0.3 if run == 0 else 2)
with open(a.output, 'w') as f: json.dump({'scenario':'switch', 'events':tl.events}, f, indent=1)
