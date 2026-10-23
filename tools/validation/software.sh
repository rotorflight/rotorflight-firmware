#!/usr/bin/env bash
set -euo pipefail
cd /work/firmware
mkdir -p /work/evidence/builds
node tools/validation/iteration_time.test.cjs > /work/evidence/iteration-time.txt
python3 src/utils/blackbox_params_audit.py > /work/evidence/static-audit.txt
python3 tools/validation/static_negative.py > /work/evidence/static-negative.txt
make -C src/test -j2 test > /work/evidence/unit-all.log 2>&1
ASAN_OPTIONS=detect_leaks=1 UBSAN_OPTIONS=halt_on_error=1 make -C src/test -j2 OBJECT_DIR=../../obj/test-sanitize-abi SANITIZE=address,undefined test_blackbox_params_unittest test_blackbox_params_short_unittest test_blackbox_params_owned_unittest > /work/evidence/unit-sanitize.log 2>&1
for target in STM32G47X STM32F7X2 STM32F405 STM32F411; do
    make -j3 "$target" FC_VER_SUFFIX=bbp-pr EXTRA_FLAGS=-Werror > "/work/evidence/builds/$target.log" 2>&1
done
for target in STM32G47X STM32F7X2; do
    make -j3 "$target" FC_VER_SUFFIX=bbp-profile DEBUG=INFO OPTIONS='BLACKBOX_PARAMS_PROFILE USE_STACK_CHECK' OBJECT_DIR="./obj/profile-$target" EXTRA_FLAGS=-Werror > "/work/evidence/builds/profile-$target.log" 2>&1
done
