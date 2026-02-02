# Changes in Rotorflight Firmware

This file is collecting the changes in the firmware that are affecting
the APIs or flight performance.


## Flight Performance

The decimator is changed to use a Bessel filter (#287). It should give more
consistent D-term reaction on transients.

PID Mode 4 is introduced for testing new features (#293). The current default
PID Mode 3 is maintained for backward compatibility.


## MSP Changes

### MSP_PID_PROFILE

- `error_rotation` parameter is unused (#294)

### MSP_SET_PID_PROFILE

- `error_rotation` parameter is unused (#294)

### MSP_PILOT_CONFIG

- added `modelFlags` parameter (#317)

### MSP_FLIGHT_STATS

- added `MSP_FLIGHT_STATS` and `MSP_SET_FLIGHT_STATS` (#317)

### MSP_SET_MOTOR_OVERRIDE

A 1.0s timeout is added to the override. The MSP call must be repeated
at least once per second to keep the override active (#304).

### MSP_SET_MOTOR

This legacy MSP call is disabled, as it does not have a timeout (#304).

### MSP_SET_PID_PROFILE

The `pid_mode` parameter can be now changed.

### MSP_GOVERNOR_PROFILE

Multiple changes (#314) (#353).

### MSP_GOVERNOR_CONFIG

Multiple changes (#314) (#353).

### MSP_GET_FBUS_MASTER_CONFIG

New MSP command (161) to retrieve the FBUS Master configuration.

### MSP_SET_FBUS_MASTER_CHANNEL

New MSP command (162) to configure individual FBUS Master channel settings.

### MSP_GET_FBUS_MASTER_CHANNEL

New MSP command (163) to retrieve individual FBUS Master channel configuration.

### MSP_GET_SBUS_OUTPUT_CONFIG

Allow querying a single sbus servo via msp (#372)

### MSP_GET_MIXER_INPUT

Add msp call to allow retrieving a single mixer line at a time (#361)

### MSP_GET_ADJUSTMENT_RANGE

Add msp call to allow retrieving a single adjustment line at a time (#362)

### MSP_SET_SERVO_CENTER

Add msp call to set just the servo center point (#366)

### MSP_RC_TUNING

The `cyclic_ring` parameter is added (#345).


## CLI Changes

`pid_process_denom` is a divider for the PID loop speed vs. the gyro
output data rate (ODR). With #291 the output rate is halved, dropping
the PID loop rate to half too.

`error_rotation` parameter is removed in #294.

`model_set_name` parameter added (ON/OFF). Corresponds with bit 0 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should set the name of the model on the radio.

`model_tell_capacity` parameter added (ON/OFF). Corresponds with bit 1 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should announce the remaining capacity of the battery.

`deadband` parameter maximum value is changed from 32 to 100 (#327).

`rc_arm_throttle` parameter is removed (#332).

`rc_min_throttle` and `rc_max_throttle` parameters default to zero, indicating that
the actual values are calculated automatically (#332).

`gov_mode` now accepts values `OFF`, `LIMIT`, `DIRECT`, `ELECTRIC`, `NITRO`.

`gov_throttle_type` is added, with possible values `NORMAL`, `SWITCH`, `FUNCTION`.

`gov_spooldown_time` is added. Value in 1/10s increments.

`gov_idle_throttle` is added. Value in 0%..25%, with 0.1% steps.

`gov_auto_throttle` is added. Value in 0%..25%, with 0.1% steps.

`gov_bypass_throttle` is added. Value array of 9, with values in 0..200. Step is 0.5%.

`gov_use_<xyz>` flags have been added. Value is `OFF` or `ON`.

`gov_fallback_drop` is added. Value in 0..50%.

`gov_dyn_min_throttle` is added. Value in 0..100%.

`gov_collective_curve` is added. Value in 5..40.

`gov_autorotation_bailout_time` is removed.

`gov_autorotation_min_entry_time` is removed.

`gov_lost_headspeed_timeout` is removed.

`gov_spoolup_min_throttle` is removed.

`blackbox_log_governor` flag is added.

`fbus_master_source_type` parameter added. Array of 16 uint8 values defining the source type for each FBUS Master channel (MIXER=0, RX=1). Source index is always equal to the channel number. MIXER uses -1000 to 1000 range, RX uses 1000 to 2000 range.

`fbus_master_frame_rate` parameter added. Value in 25..550 Hz, controls the FBUS Master output frame rate.

`fbus_master_pinswap` parameter added (ON/OFF). Swaps TX/RX pins on the FBUS Master serial port.

`fbus_master_inverted` parameter added (ON/OFF). Controls electrical inversion of the FBUS Master UART output.

`rates_type` accepts `ROTORFLIGHT` (#345).

`cyclic_ring` meaning is changed. The value indicates % of the max rate.


## Defaults

`cbat_alert_percent` changed from 10 to 35 to better reflect heli usage.

`rescue_flip` default is changed from OFF to ON.

`deadband` and `yaw_deadband` defaults changed to 5.

`rc_min_throttle` and `rc_max_throttle` defaults are changed to 0.

`motor_poles` default is changed to 0,0,0,0.

`fbus_master_source_type` defaults to MIXER (0) for first 8 channels, RX (1) for channels 8-15.

`fbus_master_frame_rate` defaults to 500 Hz.

`fbus_master_pinswap` defaults to OFF (0).

`fbus_master_inverted` defaults to ON (SERIAL_INVERTED), which is the standard for FBUS receivers.

`rates_type` default is changed to `ROTORFLIGHT`.

`cyclic_ring` default is changed to 150%.


## Features

### Drop gyro ODR to 4k on F4 and F7 (#291)

The gyro output data rate is changed from 8k to 4k on F4 and F7.
This lowers the real-time load considerably, and gives more headroom for
other functions. It should not affect performance.

### Use Bessel filter in the decimator (#287)

Using a Bessel filter in decimator should give better phase response
near the cutoff frequency. This should give more consistent D-term
reaction to fast movements.

### Motor Override (#304)

A safety mechanism is added to the Motor Override that will turn off the throttle
if the override command is not repeated continuously. This guarantees that
the motor is not left running if the connection to the FC is interrupted.

### ELRS Custom Telemetry maximum frame size (#323)

The maximum size of custom telemetry frames is reduced to 32.
This will improve telemetry reception in poor radio conditions.

### ELRS RPM and Temperature telemetry frame types (#326)

The native ELRS telemetry can now send RPM and temperature data.
The RPM frame (0x0C) supports sending headspeed and tail speed.
The temperature frame (0x0D) supports MCU and ESC temperatures.

Two new RF Telemetry sensors are added: RPM (108) and TEMP (109).

### Throttle Range calculated automatically (#332)

The input throttle range is now calculated automatically from `rc_deflection`.
It can be still set by the user with `rc_min_throttle` and `rc_max_throttle`.
The parameter `rc_arm_throttle` is removed, and arming is allowed when
input throttle is well below `rc_min_throttle`.

### Motor Pole Count (#333)

The default pole count is now zero, which is effectively disabling the RPM input.
This forces the user to enter the correct number, before the RPM input can be used.

### PID Mode 4 (#293)

All new features and changes to the PID controller are done in the new PID mode 4.
The current PID Mode 3 will be kept as-is for backward compatibility.

**Changes:**
- axis_error changed from actual angle to I-term units
- I-gains, O-gains and F-gains forced to be the same for roll & pitch
- I-term decay forced if I-gain is zero (Rate Mode)
- Pitch B-gain scaled x10
- Roll B-gain and D-gain scaled /5
- Yaw precomp cutoff scaled by /10

### Governor Refactoring (#314) (#343) (#353)

The Governor has been refactored to accomodate I.C./nitro and other new features.

### Rotorflight Rates (#345)

A new Rates systems is added for helicopter applications. `ROTORFLIGHT` rates is
controlled by three parameters: maximum rate, expo, and shape.


## Bug Fixes

### S.PORT telemetry Scaling for attitude sensors

The attitiude sensors where found to be out by a factor of 10.  The scaling
in the firmware has been adjusted to set these correctly. (#313)