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
| UARTn <-> client | `tcp://127.0.0.1:576n` | one client per port; UART1 (5761) carries MSP and the CLI |

Both structs are in [target.h](target.h). RC arrives as `MSP_SET_RAW_RC` (MSP RX). Point the
Configurator at `tcp://127.0.0.1:5761`.

The firmware does not estimate attitude here (`USE_IMU_CALC` is off): it takes the quaternion from
the FDM packet. Its clock follows the packet timestamps (`simRate`, measured over at least 50 ms),
so the simulator should send faster than 50 Hz, from one sender only.

### eeprom.bin

The configuration is `eeprom.bin` in the working directory (`EEPROM_SIZE`, 32768 bytes). A save
that reboots ends the process: start it again on the store it just wrote. A store written by a
different build is replaced with defaults.

### Host timing

`SITL_LOOP_SLEEP_US` sets the idle between scheduler passes (default 50 us on Linux). On a host
whose sleep granularity is coarser than a gyro period, such as WSL1 (about 0.7 ms for any
sleep), set it to 0: the main loop then only yields the CPU. On Windows that is the default,
since Windows cannot sleep for less than a timer tick; delays under 1 ms are waited out there.
