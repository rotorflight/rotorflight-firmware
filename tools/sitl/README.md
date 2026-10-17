# Rotorflight SITL logging tests

Runs Rotorflight as a Linux process with a simulated IMU and receiver, drives configuration changes over MSP,
and captures Blackbox output over TCP. No helicopter or Linux host is required: use Docker as described in
[the validation instructions](../validation/README.md). The validation report identifies the tested revisions.

Apply `tools/validation/sitl.patch` to a separate firmware copy first. It repairs the simulator build, TCP thread
ownership, fake-sensor calibration and clock support. The patch is excluded from hardware builds.

## Run a scenario

```sh
make -j3 SITL ARM_SDK_DIR=/usr ARM_SDK_PREFIX= OBJECT_DIR=./obj/validation OPTIONS='SITL_GYRO_RATE_HZ=1000 SITL_REALTIME_CLOCK'
SITL_GYRO_RATE_HZ=1000 tools/sitl/run_all.sh -n journal -s journal \
    -c 'set pid_process_denom = 1' -e obj/validation/rotorflight_SITL.elf
```

`run_all.sh` starts with fresh EEPROM, boots the simulator, calibrates the accelerometer, configures logging,
saves and restarts, runs the selected scenario, and stops its processes. It replaces `obj/sitl_runs/NAME` when
that name already exists. `-c LINE` adds a CLI configuration command; `-e ELF` selects an existing executable;
arguments after `--` go to the scenario. Run only one scenario at a time within a container because ports are fixed.

Outputs in `obj/sitl_runs/NAME/`:

| File | Contents |
|---|---|
| `blackbox.bbl` | Blackbox stream from UART2 |
| `blackbox.bbl.chunks.csv` | Host monotonic time, offset and size of received chunks |
| `timeline.json` | Commands, independent MSP readbacks and send/reply times |
| `configure.json`, `capture.json` | Applied configuration and capture statistics |
| `sitl.log`, `boot0.log`, `boot1.log`, `feed.log` | Process output |
| `eeprom.bin` | Saved configuration for restart/readback checks |

## Configuration and interfaces

The harness configures ARM on AUX1 (RC channel index 5), UART1 for MSP, UART2 for Blackbox at 2 Mbaud,
and ARMED logging with a five-second grace period. The default Blackbox divisor is 2. Each process reboot exits;
the harness starts another process with the same EEPROM. No motors or servos are configured.

| Port | Purpose |
|---|---|
| UDP 9003 | IMU/attitude packets, normally level and stationary, at 1 kHz |
| UDP 9002 | Simulator motor-output packets |
| UDP 9004 | Receiver channels at 100 Hz |
| UDP 9010 | JSON control of the feed, such as `{"rc":{"5":1900}}` |
| TCP 5761 | MSP commands and CLI |
| TCP 5762 | Blackbox capture; opens when a log starts |

The validation matrix uses the monotonic real-time clock so `t_sitl_us` is host time minus the boot time printed
by the simulator. The normal simulator clock instead scales time from IMU packets and is not used for these checks.

## Scenarios

- `journal`: change gains while armed, switch PID profiles, change gains during the grace period, rearm, save,
  and finish the log. Options cover between-log changes, extended configuration commands and receiver adjustments,
  header-time changes, forced process termination, and CLI entry. Each write records expected stored values and
  independent MSP readbacks.
- `regrace`: focused gain-change and grace-period rearming test.
- `switch`: stop and restart logging through its receiver switch, checking independent headers and profile state.

`tools/validation/simulator.sh` runs the scenarios and analyzes the resulting logs with the feature decoder and
released Blackbox 2.3.0. It also compares OFF-mode headers with an upstream-master simulator built with the same patch.

## Limits

Docker scheduling delays can activate the unchanged LOAD arming guard at an 8 kHz fake gyro rate. The matrix uses
1 kHz with divisor 1 for standard scenarios. Divisor cases set the fake gyro to 800 times the requested divisor
(1/2/3/4/8), maintaining a supported 800 Hz PID rate. Arming still requires valid RC,
calibration and acceptable load. Transient LOAD failures are recorded and retried within bounded scenario timeouts.

The simulator has no helicopter dynamics, motor loads, radio transport, storage-chip timing or MCU stack behavior.
TCP does not model a UART baud rate, although the configured baud rate still selects the firmware's header policy.
Frame-time intervals accommodate host pauses; frame continuity, journal integrity, stored values and phase ordering
are checked separately. Hardware timing, stack headroom and flight behavior require measurements on the actual board.
