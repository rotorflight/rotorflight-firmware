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
| simulator -> FC | `udp://127.0.0.1:9003` | `fdm_packet` (136 bytes): timestamp, gyro, accelerometer, attitude quaternion, velocity, position |
| FC -> simulator | `udp://127.0.0.1:9002` | `servo_packet` (48 bytes): `motor_speed[4]` (M1-M4, 0..1), `servo[8]` (S1-S8, us, 0 = absent) |
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

Defaults for a simulator (`config.c`, applied on a config reset): MSP on UART2 as well as UART1,
and the frequency sensor on with a 2-pole main motor, so FDM `motor_erpm[0]` reads 1:1 as motor
rpm for the governor and the RPM filter. A model sending real electrical rpm sets `motor_poles`.

The firmware does not estimate attitude here (`USE_IMU_CALC` is off): it takes the quaternion from
the FDM packet. Its clock follows the packet timestamps (`simRate`, measured over at least 50 ms),
so the simulator should send faster than 50 Hz, from one sender only.

### eeprom.bin

The configuration is `eeprom.bin` in the working directory (`EEPROM_SIZE`, 32768 bytes). A save
that reboots ends the process: start it again on the store it just wrote. A store written by a
different build is replaced with defaults.

### Host timing

The main loop idles 50 us between scheduler passes where the host can sleep that briefly. At
start the SITL measures a 50 us sleep; where it takes more than 75 us (WSL1 about 0.7 ms, many
cloud VMs), the gyro/PID loop would starve, so the loop only yields the CPU instead (one busy
core). The log says which: `[system]main loop idle: ...`. `SITL_LOOP_SLEEP_US` overrides the
choice. On Windows, delays under 1 ms are waited out rather than slept.
