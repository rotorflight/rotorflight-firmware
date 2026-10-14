#!/usr/bin/env python3
"""Restart a captured SITL EEPROM and compare the saved active gains through MSP."""
import json, os, shutil, subprocess, sys, time
from pathlib import Path
sys.path.insert(0, '/work/sitl/tools/sitl')
from msp import MSP
source = Path('/work/sitl/obj/sitl_runs/OFF')
out = Path('/work/sitl/obj/sitl_runs/reboot-readback'); out.mkdir(exist_ok=True)
shutil.copyfile(source/'eeprom.bin', out/'eeprom.bin')
truth = next(e for e in json.loads((source/'timeline.json').read_text())['events'] if e['event']=='done')
with open(out/'sitl.log','w') as log, open(out/'feed.log','w') as feed_log:
    feed = subprocess.Popen(['python3','/work/sitl/tools/sitl/sim_feed.py'],stdout=feed_log,stderr=subprocess.STDOUT)
    fcproc = subprocess.Popen(['/work/sitl/obj/validation/rotorflight_SITL.elf'],cwd=out,stdout=log,stderr=subprocess.STDOUT)
    try:
        with MSP(connect_timeout=20) as fc:
            time.sleep(4)
            actual=fc.pid_tuning(); status=fc.status()
            assert actual==truth['pid'], (actual,truth['pid'])
            assert status['pid_profile']==truth['status']['pid_profile']
            assert status['rate_profile']==truth['status']['rate_profile']
            print(json.dumps({'persistedGains':actual,'pidProfile':status['pid_profile'],'rateProfile':status['rate_profile'],'passed':True},indent=2))
    finally:
        for proc in [fcproc,feed]: proc.terminate()
        for proc in [fcproc,feed]: proc.wait(timeout=10)
