#!/usr/bin/env bash
set -euo pipefail
cd /work/sitl
export SITL_GYRO_RATE_HZ=1000
for delay in 0 2 4 8; do
    tools/sitl/run_all.sh -n "kill-$delay" -s journal -c 'set pid_process_denom = 1' -e /work/sitl/obj/validation/rotorflight_SITL.elf -- --step 0.3 --kill-ms "$delay" > "/work/evidence/kill-$delay-run.log" 2>&1
done
node /work/firmware/tools/validation/abrupt_check.cjs obj/sitl_runs/kill-{0,2,4,8} > /work/evidence/abrupt.json
tools/sitl/run_all.sh -n switch -s switch -c 'set pid_process_denom = 1' -e /work/sitl/obj/validation/rotorflight_SITL.elf > /work/evidence/switch-run.log 2>&1
node /work/firmware/tools/validation/switch_check.cjs obj/sitl_runs/switch/blackbox.bbl > /work/evidence/switch.json
tools/sitl/run_all.sh -n stock -s journal -c 'set pid_process_denom = 1' -e /work/sitl-baseline/obj/validation/rotorflight_SITL.elf > /work/evidence/stock-run.log 2>&1
node /work/firmware/tools/validation/analyze.cjs obj/sitl_runs/stock --json /work/evidence/stock.json > /work/evidence/stock-analysis.log 2>&1
node /work/firmware/tools/validation/compare_headers.cjs obj/sitl_runs/OFF/blackbox.bbl obj/sitl_runs/stock/blackbox.bbl > /work/evidence/off-stock-headers.txt
tools/sitl/run_all.sh -n slow-serial -s journal -c 'set pid_process_denom = 1' -c 'serial 1 128 115200 57600 0 115200' -e /work/sitl/obj/validation/rotorflight_SITL.elf > /work/evidence/slow-serial-run.log 2>&1
node /work/firmware/tools/validation/analyze.cjs obj/sitl_runs/slow-serial --json /work/evidence/slow-serial.json > /work/evidence/slow-serial-analysis.log 2>&1
python3 - <<'PY'
import json
assert all(x['info']['mode']=='CHANGES' for x in json.load(open('/work/evidence/slow-serial.json'))['logs'])
PY
tools/sitl/run_all.sh -n cli-end -s journal -c 'set pid_process_denom = 1' -e /work/sitl/obj/validation/rotorflight_SITL.elf -- --step 0.3 --armed2 1 --cli-end > /work/evidence/cli-end-run.log 2>&1
node /work/firmware/tools/validation/abrupt_check.cjs obj/sitl_runs/cli-end > /work/evidence/cli-end.json
python3 /work/firmware/tools/validation/reboot_readback.py > /work/evidence/reboot-readback.json
