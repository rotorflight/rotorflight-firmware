#!/usr/bin/env bash
set -euo pipefail
cd /work/sitl
# Keep the fake gyro at 1 kHz so Docker can satisfy the unchanged LOAD arming guard.
# Divisor cases scale the fake gyro to keep the PID rate at the supported 800 Hz minimum.
export SITL_GYRO_RATE_HZ=1000
options='SITL_GYRO_RATE_HZ=1000 SITL_REALTIME_CLOCK'
make -j3 SITL ARM_SDK_DIR=/usr ARM_SDK_PREFIX= OBJECT_DIR=./obj/validation OPTIONS="$options" > /work/evidence/sitl-build.log 2>&1
make -C /work/sitl-baseline -j3 REVISION="${BASELINE_REVISION:-master}" SITL ARM_SDK_DIR=/usr ARM_SDK_PREFIX= OBJECT_DIR=./obj/validation OPTIONS="$options" > /work/evidence/sitl-baseline-build.log 2>&1
run() {
    local name=$1; shift
    tools/sitl/run_all.sh -n "$name" -s journal -c 'set pid_process_denom = 1' -e /work/sitl/obj/validation/rotorflight_SITL.elf "$@" > "/work/evidence/$name-run.log" 2>&1
    node /work/firmware/tools/validation/analyze.cjs "obj/sitl_runs/$name" --json "/work/evidence/$name.json" > "/work/evidence/$name-analysis.log" 2>&1
}
run full-between -- --between --toggles 12
run CHANGES -c 'set blackbox_params = CHANGES' -- --toggles 4
run OFF -c 'set blackbox_params = OFF'
run extended -- --extended
run header-write -- --header-write
for denom in 1 2 3 4 8; do
    SITL_GYRO_RATE_HZ=$((800 * denom)) tools/sitl/run_all.sh -n "denom-$denom" -s journal -c "set pid_process_denom = $denom" -e /work/sitl/obj/validation/rotorflight_SITL.elf -- --step 0.5 --armed1 3 --armed2 1 --toggles 4 > "/work/evidence/denom-$denom-run.log" 2>&1
    node /work/firmware/tools/validation/analyze.cjs "obj/sitl_runs/denom-$denom" --json "/work/evidence/denom-$denom.json" > "/work/evidence/denom-$denom-analysis.log" 2>&1
done
bash /work/firmware/tools/validation/fault_matrix.sh
node /work/firmware/tools/validation/protocol_faults.cjs obj/sitl_runs/extended/blackbox.bbl > /work/evidence/protocol-faults.json
