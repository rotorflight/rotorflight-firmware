## SITL

`TARGET=SITL` builds Rotorflight as a native Linux executable: the firmware above the hardware
drivers (scheduler, filters, PID, governor, mixer, rescue, MSP, CLI) runs as a process, and an
external flight dynamics model takes the place of the sensors and outputs.

### Build and run

```sh
make arm_sdk_install        # once; the makefile checks for it even for SITL
make TARGET=SITL            # -> obj/main/rotorflight_SITL.elf
./obj/main/rotorflight_SITL.elf   # first start: writes eeprom.bin with defaults, exits 0
./obj/main/rotorflight_SITL.elf   # second start: stays up
```

On Windows, build natively from an MSYS2 shell (or any shell with GNU make, curl and unzip):

```sh
make arm_sdk_install        # once, as above
make mingw_sdk_install      # once: standalone MinGW-w64 GCC into tools/mingw64
make TARGET=SITL            # -> obj/main/rotorflight_SITL.elf, a static Windows executable
```

The output keeps the `.elf` name; it is a Windows executable all the same (start it from a
script or rename it to `.exe` to run it from Explorer).

`python3 src/utils/sitl-smoke-test.py obj/main/rotorflight_SITL.elf` builds nothing; it runs the
executable and checks first start, MSP identity, a CLI save, arming over MSP RX with a level FDM
feed, motor output and disarming. CI runs it on every PR (`.github/workflows/sitl.yml`).

### Interfaces

| Direction | Address | Payload |
|---|---|---|
| simulator -> FC | `udp://127.0.0.1:9003` (`SITL_FDM_PORT`) | `fdm_packet`, 136 bytes: timestamp, gyro, accelerometer, attitude quaternion, velocity, position; optional tail told apart by length: 168 + motor eRPM[4], 176 + pressure, 192 + MCU vref/temperature, 208 + BEC/bus rails, 224 + pack voltage/current |
| simulator -> FC | same port | `baro_packet`, 24 bytes: timestamp, pressure, temperature (Wingflight's layout) |
| FC -> simulator | `udp://127.0.0.1:9002` (`SITL_OUTPUT_ADDR`) | `servo_packet`, 68 bytes: `motor_speed[4]` (M1-M4, commanded fraction 0..1), `servo[8]` (S1-S8, us, 0 = absent), `mixer[5]` (stabilized roll, pitch, yaw, collective, throttle); the first 48 bytes are Wingflight's layout |

Wingflight's SITL accepts the same FDM tail, so a simulator written for either firmware
works with both.
| UARTn <-> client | `tcp://127.0.0.1:576n` | one client per port; UART1 (5761) and UART2 (5762) carry MSP and the CLI by default |

Both structs are in [target.h](target.h). By default RC arrives as `MSP_SET_RAW_RC` (MSP RX).
A radio can instead speak CRSF on any UART, with CRSF telemetry back on the same port, for
example on UART2 (TCP 5762):

```
feature -RX_MSP
feature RX_SERIAL
feature TELEMETRY
serial 1 64 115200 57600 0 115200
set serialrx_provider = CRSF
save
```

Point the Configurator at `tcp://127.0.0.1:5762` (or 5761 when nothing else uses UART1).
A reboot ends the SITL process ("Save and Reboot", leaving the CLI tab, `MSP_REBOOT`), so
start it through `src/utils/sitl/sitl-run.py <elf> --dir <rundir>`: that starts it again
on the same `eeprom.bin` after every reset the firmware announces (`[system]Reset!`) and
the Configurator reconnects as it would to a board. A crash is reported, not restarted.

Defaults for a simulator (`config.c`, applied on a config reset): MSP on UART2 as well as UART1,
and the frequency sensor on with a 2-pole main motor, so FDM `motor_erpm[0]` reads 1:1 as motor
rpm for the governor and the RPM filter. A model sending real electrical rpm sets `motor_poles`.

### ESC on a UART

The ESC sensor works on any TCP UART. `src/utils/sitl/esc/esc_sim.py` is a virtual ESC for it,
with telemetry and the parameter (programming) dialogue the Configurator's ESC tab uses:

```
feature ESC_SENSOR
serial 2 1024 115200 57600 0 115200
set esc_sensor_protocol = OPENYGE        # or HOBBYWINGV5, SCORPION, FLYROTOR, XDFLY, OMPHOBBY, ZTW
set esc_sensor_halfduplex = ON           # selects the parameter dialogue
save
```

`save` reboots, which ends a SITL started directly: start it again (or run it through
`sitl-run.py`, which does that for you), then
`python3 src/utils/sitl/esc/esc_sim.py --protocol openyge --port 5763`.
`src/utils/sitl/esc/esc_check.py <elf>` runs read, telemetry and write end to end for each
emulated protocol.

The firmware does not estimate attitude here (`USE_IMU_CALC` is off): it takes the quaternion from
the FDM packet. Its clock follows the packet timestamps (`simRate`, measured over at least 50 ms),
so the simulator should send faster than 50 Hz, from one sender only.

### Radio in the loop

`src/utils/sitl/radio/radio_bridge.py` puts EdgeTX itself on the CRSF UART: Companion's
simulator library (`edgetx_sim.py`) runs a radio with its own SD card, its mixer outputs go
to the SITL as CRSF RC frames and the SITL's CRSF telemetry goes back into the radio, so a
Lua suite on the card talks to the simulated flight controller over CRSF/MSP as it would
to a real one. Linux, from Companion's AppImage (`--appimage-extract`, then
`LD_LIBRARY_PATH=squashfs-root/usr/lib`, library dir `squashfs-root/usr/lib/companion212`);
under WSL1 strip `.note.ABI-tag` from its `libQt6Core.so.6`. The Windows library binds but
its `start()` faults (2.12.5). `radio_check.py` runs link, stick, arming and telemetry
end to end, with the EdgeTX Lua suite's `simulator/` card.

### Tests

- `src/utils/sitl-smoke-test.py <elf>`: start, MSP, CLI, arming, outputs, CRSF, clock, lockstep
- `src/utils/sitl/sitl-run-check.py <elf>`: restarts after reboots (see sitl-run.py above)
- `src/utils/sitl/scenarios.py <elf> [governor|profile|failsafe]`: a ground-run rotor model
  closes the loop through `motor_erpm[0]`; governor spool-up, a PID profile switch moving the
  headspeed, and RX loss (Rotorflight keeps stage-2 failsafe off: the rxfail values cut the
  throttle, the heli stays armed, the governor recovers when the link is back)

All run in CI (`.github/workflows/sitl.yml`) on Linux and Windows; the radio check on Linux.

### eeprom.bin

The configuration is `eeprom.bin` in the working directory (`EEPROM_SIZE`, 32768 bytes). A save
that reboots ends the process: start it again on the store it just wrote. A store written by a
different build is replaced with defaults.

### Blackbox

The SITL has onboard dataflash: `blackbox_flash.bin` next to `eeprom.bin`, a file-backed 128 MiB
NOR chip (`drivers/flash_file.c`, ported from Wingflight). Blackbox logs to it by default, and
flashfs, rolling erase, the CLI `flash_*` commands and the Configurator's dataflash download and
erase work on it unchanged. Every write is flushed, so a log survives the process being killed.
The RTC starts from the host clock, so logs carry real start times.

### Lockstep

By default the firmware clock runs in real time, scaled to the rate at which FDM timestamps
arrive. With `SITL_LOCKSTEP=1` it is the simulator's clock instead: once the main loop runs, the
firmware's time is the FDM timestamp. A simulator that steps its physics, sends the state and waits
for the output packet gets exactly one output packet per FDM packet, sent after the scheduler pass
that runs the control step for the new time. With 250 us steps that is one physics step per
gyro/PID cycle, independent of host load and as fast as the host allows (2-3x real time on a
laptop). Without FDM packets the clock stands still.

### Host timing

The main loop idles 50 us between scheduler passes where the host can sleep that briefly. At
start the SITL measures a 50 us sleep; where it takes more than 75 us (WSL1 about 0.7 ms, many
cloud VMs), the gyro/PID loop would starve, so the loop only yields the CPU instead (one busy
core). The log says which: `[system]main loop idle: ...`. `SITL_LOOP_SLEEP_US` overrides the
choice. On Windows, delays under 1 ms are waited out rather than slept.
