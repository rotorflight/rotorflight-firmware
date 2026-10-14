#!/bin/bash
# Build the SITL (if needed), configure a fresh eeprom.bin, start the SITL, the
# sensor/RC feed and the blackbox capture, run one scenario, stop everything.
#
# Usage: tools/sitl/run_all.sh [-n NAME] [-s SCENARIO] [-d DENOM] [-c CLI]... [-e ELF] [-- SCENARIO ARGS...]
#   NAME      output directory obj/sitl_runs/NAME (default: SCENARIO-<date>)
#   SCENARIO  tools/sitl/scenario_SCENARIO.py (default: regrace)
#   DENOM     blackbox rate = PID rate / DENOM (default 2: 500 Hz)
#   CLI       a CLI line for configure.py, e.g. -c 'set blackbox_params = OFF' (repeat)
#   ELF       another SITL binary (no build), e.g. the stock build of another worktree
# The scenario gets the SITL process id in the environment (SITL_PID).
#
# Output in obj/sitl_runs/NAME/: blackbox.bbl (+ .chunks.csv), timeline.json,
# capture.json, configure.json, sitl.log, feed.log, eeprom.bin.
# Linux only (the SITL target); needs gcc, make, python3.

set -euo pipefail

TOOLS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FW="$(cd "$TOOLS/../.." && pwd)"
ELF="$FW/obj/main/rotorflight_SITL.elf"
SCENARIO=regrace
NAME=""
DENOM=2
CLI=()
OTHER_ELF=""
while [ $# -gt 0 ]; do
    case "$1" in
        -n) NAME="$2"; shift 2 ;;
        -s) SCENARIO="$2"; shift 2 ;;
        -d) DENOM="$2"; shift 2 ;;
        -c) CLI+=(--cli "$2"); shift 2 ;;
        -e) OTHER_ELF="$2"; shift 2 ;;
        --) shift; break ;;
        *) echo "usage: $0 [-n NAME] [-s SCENARIO] [-d DENOM] [-c CLI]... [-e ELF] [-- SCENARIO ARGS...]" >&2; exit 2 ;;
    esac
done
NAME="${NAME:-$SCENARIO-$(date +%Y%m%d-%H%M%S)}"
[[ "$NAME" =~ ^[a-zA-Z0-9][a-zA-Z0-9_.-]*$ ]] || { echo "Invalid run name" >&2; exit 2; }
OUT="$FW/obj/sitl_runs/$NAME"

log() { echo "run_all: $*" >&2; }

# 1. build (make is incremental)
if [ -n "$OTHER_ELF" ]; then
    ELF="$(cd "$(dirname "$OTHER_ELF")" && pwd)/$(basename "$OTHER_ELF")"
    log "using $ELF (no build)"
else
    log "building SITL"
    make -C "$FW" SITL ARM_SDK_DIR=/usr ARM_SDK_PREFIX= -j"$(nproc)" > "$FW/obj/sitl_build.log" 2>&1 \
        || { tail -30 "$FW/obj/sitl_build.log" >&2; exit 1; }
fi

rm -rf "$OUT"
mkdir -p "$OUT"
cd "$OUT"

PIDS=()
cleanup() {
    for p in "${PIDS[@]}"; do kill "$p" 2>/dev/null || true; done
    wait 2>/dev/null || true
}
trap cleanup EXIT

# 2. first boot: the SITL writes the default configuration to eeprom.bin and exits
log "first boot (default configuration)"
timeout 30 "$ELF" > boot0.log 2>&1 || true
[ -f eeprom.bin ] || { log "no eeprom.bin after the first boot"; exit 1; }

# 3. feed: level, still IMU at 1 kHz, RC at 100 Hz
python3 "$TOOLS/sim_feed.py" > feed.log 2>&1 &
FEED=$!; PIDS+=("$FEED")

# 4. configure over MSP, save, reboot (the SITL exits)
log "configuring"
"$ELF" > boot1.log 2>&1 &
SITL=$!; PIDS+=("$SITL")
python3 "$TOOLS/configure.py" --denom "$DENOM" "${CLI[@]}" > configure.json
wait "$SITL" || true

# 5. the test boot, with the blackbox capture
log "starting SITL"
"$ELF" > sitl.log 2>&1 &
SITL=$!; PIDS+=("$SITL")
python3 "$TOOLS/capture.py" blackbox.bbl > capture.json &
CAPTURE=$!; PIDS+=("$CAPTURE")

log "running scenario $SCENARIO"
SITL_PID=$SITL python3 "$TOOLS/scenario_$SCENARIO.py" --sitl-log sitl.log --output timeline.json "$@" > /dev/null

# 6. stop: the capture last, after the TCP data has arrived
sleep 1
kill -TERM "$CAPTURE"; wait "$CAPTURE" || true
kill -TERM "$SITL" "$FEED" 2>/dev/null || true
wait "$SITL" "$FEED" 2>/dev/null || true
trap - EXIT

log "done: $OUT"
python3 - "$OUT" <<'EOF'
import json, os, sys
out = sys.argv[1]
cap = json.load(open(os.path.join(out, 'capture.json')))
print(json.dumps({'output': out, 'blackbox_bytes': cap['bytes'],
                  'files': sorted(os.listdir(out))}, indent=1))
EOF
