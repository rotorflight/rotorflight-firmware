# Changes in Rotorflight Firmware

This file is collecting the changes in the firmware that are affecting
the APIs or flight performance.


## Flight Performance

The decimator is changed to use a Bessel filter (#287). It should give more
consistent D-term reaction on transients.

PID Mode 4 is introduced for testing new features (#293). The current default PID Mode 3
is maintained for backward compatibility.


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


## CLI Changes

`pid_process_denom` is a divider for the PID loop speed vs. the gyro
output data rate (ODR). With #291 the output rate is halved, dropping
the PID loop rate to half too.

`error_rotation` parameter is removed in #294.

`model_set_name` parameter added (ON/OFF). Corresponds with bit 0 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should set the name of the model on the radio.

`model_tell_capacity` parameter added (ON/OFF). Corresponds with bit 1 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should announce the remaining capacity of the battery.


## Defaults

`cbat_alert_percent` changed from 10 to 35 to better reflect heli usage.
`rescue_flip` default is changed from OFF to ON.

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

### PID Mode 4 (#293)

All new features and changes to the PID controller are done in the new PID mode 4.
The current PID Mode 3 will be kept as-is for backward compatibility.

**Changes:**
- axis_error changed from actual angle to I-term units
- I-gains, O-gains and F-gains forced to be the same for roll & pitch
- I-term decay forced if I-gain is zero (Rate Mode)
- Pitch B-gain scaled x10
- Roll B-gain and D-gain scaled /5


## Bug Fixes

### S.PORT telemetry Scaling for attitude sensors

The attitiude sensors where found to be out by a factor of 10.  The scaling
in the firmware has been adjusted to set these correctly. (#313)