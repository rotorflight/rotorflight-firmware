# Changes in Rotorflight Firmware

This file is collecting the changes in the firmware that are affecting
the APIs or flight performance.

# 4.7.0

## Features

- Spektrum SRXL2 ESC support: new motor protocol, ESC telemetry protocol and serial function (#421, #489)
- Spektrum full size receivers (e.g. AR6610T) supported; requires `srxl2_unit_id = 0` (#486)
- Spektrum bind supports pin swap; initial bind glitch fixed (#496)
- FrSky RPM and temperature sensor via FBUS/S.Port (#461)
- XDFLY/ZTW/OMPHOBBY ESC telemetry works in both half duplex (bidirectional) and receive-only mode (#478)
- Alternative takeoff detection based on stick response and Z-acceleration (#480)
- Separate angle limit for Horizon mode (#479)
- Speed-dependent cyclic I-term decay to prevent wind-up on cyclic input (#457)
- Deadband on continuous adjustment channels stops values from toggling on pot noise (#507)
- CMS compiled out on all targets (#492)
- GHOST, RX_PPM and RX_PARALLEL_PWM removed from unified targets to free flash (#514)
- Tune advisor: in-flight rate-loop statistics per axis over MSP, for tuning advice on the radio (#523)
- FrSky XACT servo programming over the F.Bus master link, ported from WingFlight (#518)
- New `SYSTEM_STATUS` (120; S.Port `0x5140`, CRSF `0x1230`) and `SYSTEM_CONFIG` (121; S.Port `0x5141`, CRSF `0x1231`) telemetry sensors pack FC status, governor and rescue state, profiles and config flags into two bitfields for radio dashboards; layout in `src/main/telemetry/status.h`. Existing sensors are unchanged. Both are added to the default `telemetry_sensors` (`PG_TELEMETRY_CONFIG` v8)

## Bug Fixes

- ICM42605 gyro uses the correct ODR (#475)
- Servos are no longer set to midpoint at startup (#466)
- Forwarding of S.Port master sensors fixed (#481)
- FBUS/S.Port current sensor accumulates consumed capacity (#513)
- Bus servo speed limit uses the real frame time; SBUS and F.Bus keep separate state (#515)
- `MSP_COPY_PROFILE` reloads the correct rate profile (#520)

## Flight Performance

Horizon mode uses a per-axis cubic leveling curve, ramps in over 500ms
on activation, and has its own `horizon_angle_limit` (#479).

Cyclic I-term decay can be scaled with setpoint via `error_decay_gain_cyclic`
(#457). Disabled by default.

Bus servos with a `speed` set previously moved much slower than configured
(about 20x at 50Hz SBUS). They now move at the configured speed (#515).

## MSP Changes

- API version 12.10 (#484)

### MSP_PID_PROFILE / MSP_SET_PID_PROFILE

- added `error_decay_gain_cyclic` (#457)

### MSP_MOTOR_CONFIG / MSP_ESC_SENSOR_CONFIG

- `SRXL2` inserted in the motor and ESC sensor protocol lists; `DISABLED` and `RECORD` values are shifted by one (#421)

### MSP2_GET_FBUS_SENSORS / MSP2_CLEAR_FBUS_SENSORS

- new commands (0x5F07, 0x5F08) to list and clear observed FBUS/S.Port sensors (#482)

### MSP2_GET_FBUS_MASTER_CONFIG / MSP2_SET_FBUS_MASTER_CONFIG

- new commands (0x5F09, 0x5F0A) to get/set forwarded sensors; applied without reboot (#482)

### MSP2_GET_TUNE_ADVISOR / MSP2_CLEAR_TUNE_ADVISOR

- new commands (0x5F10, 0x5F11): read one axis of the tune advisor statistics, or clear them (#523).
  Counted only while spooled up, airborne and in plain rate flight; RAM only, cleared when the
  tune changes (checked on arming). New commands only, the API version is unchanged.
- request `U8 axis` (0 roll, 1 pitch, 2 yaw); reply (67 bytes), payload v1:
  `U8 version, U8 collecting, U16 seconds, U8 axis`,
  `U16 P, U16 F, U16 B, U8 iterm_relax_cutoff, U8 rates_type, U8 rc_rate, U8 s_rate`,
  `U16 ffCount, S16 ffGain, S16 ffCorr, U16 ffLagMs` (ffGain = gyro / setpoint at the best delay),
  3 x `S16 gain, U16 count` by request (40-100, 100-200, 200+ deg/s),
  3 x `S16 gain, U16 count` by |collective| (<25%, 25-50%, 50%+),
  `U16 fullCount, U16 fullSatCount, S16 fullRatio, U16 fullMaxRate`,
  `U16 releases, U16 bigRebounds, S16 meanRebound, S16 meanOvershoot, S16 meanCounter,
  S16 meanIterm` (meanCounter is P+I+D+B, so tail and pitch precomp do not count).
  Ratios are x1000, counts saturate at 65535.

### MSP_SET_XACT_SCAN

New MSP command (161) to restart discovery of XACT servos on the F.Bus master link. No payload.
Returns an error if F.Bus master is not enabled or the system is armed (#518).

### MSP_XACT_SERVO_LIST

New MSP command (165) to list the XACT servos discovered since the last scan (#518).
Returns: U8 count, then per servo: U8 phyID, U8 appIdOffset, U8 conflict, U8 duplicateAppId, U8 ready, U8 channel.

### MSP_XACT_PARAMS

New MSP command (162) to read all parameters of one discovered XACT servo (#518). Payload: U8 phyID.
Starts a read if none has completed yet; repeat until `ready` is 1. `ready` stays 0 while any field
except the firmware version is unanswered, and the next request retries the read.
Returns: U8 ready, U8 conflict, U8 duplicateAppId, U8 physicalId, U8 appIdOffset, U8 firmwareVersion,
U16 dataRate, U8 range, U8 direction, U8 pulseType, U8 channel, S8 center, U8 holdingStrength,
U8 operationSmoothing, U8 deadband, U8 hasExtendedParams, U8 workingMode, U16 maxAngle.

### MSP_SET_XACT_PARAMS

New MSP command (163) to write the parameters of one discovered XACT servo (#518). Only changed fields are
written, followed by a save to the servo's flash. Payload: U8 targetPhyID, U8 physicalId, U8 appIdOffset,
U16 dataRate, U8 range, U8 direction, U8 pulseType, U8 channel, S8 center, U8 holdingStrength,
U8 operationSmoothing, U8 deadband, U8 workingMode, U16 maxAngle.
Returns an error if F.Bus master is not enabled, the system is armed, the servo is unknown, its
parameters have not been read yet, physicalId is above 26 or appIdOffset above 15, another servo
shares its App ID (unless the write changes it to an unused one), or earlier saves are still being sent. Succeeds without writing if no field
differs from the last read. No XACT traffic is sent while armed.

## CLI Changes

- added `airborne_mode`, `airborne_gyro_threshold`, `airborne_acc_threshold` (#480)
- added `horizon_angle_limit` (#479)
- added `error_decay_gain_cyclic` (#457)
- added `srxl2esc` command (#421)
- `SRXL2` added to `motor_pwm_protocol` and `esc_sensor_protocol` (#421)
- new serial function `FUNCTION_SRXL2_ESC` (2097152) (#421)

## Defaults

- `airborne_mode = CONSERVATIVE`, `airborne_gyro_threshold = 10`, `airborne_acc_threshold = 15` (#480)
- `horizon_angle_limit = 55` (#479)
- `error_decay_gain_cyclic = 0` (#457)
- Bus servo scale (`rneg`/`rpos`) changed from 1000 to 500, matching PWM servos; saved configs are unchanged (#515)

# 4.6.0

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

### MSP_SET_4WIF_ESC_FWD_PROG

New MSP command (244) to select an ESC for forward programming over the 4-way interface. Payload: U8 ESC id; values in `0..MAX_SUPPORTED_MOTORS-1` select that ESC, while values `>= MAX_SUPPORTED_MOTORS` (for example `0xFF`) are treated as deselect/exit and return success. The only error conditions for command `244` are: the system is armed, the payload length is not exactly 1 byte, or the 4-way selection fails.

When a 4-way ESC is selected, `MSP_ESC_PARAMETERS` / `MSP_SET_ESC_PARAMETERS` now expose the detected target EEPROM payload for both AM32 and BLHeli_S SiLabs targets. AM32 continues to use the compact 48-byte payload; BLHeli_S uses the 0x70-byte BLHeli_S EEPROM layout and erases the containing settings page before writes.

### MSP_SET_PID_PROFILE

The `pid_mode` parameter can be now changed.

### MSP_GOVERNOR_PROFILE

Multiple changes (#314) (#353).

### MSP_GOVERNOR_CONFIG

Multiple changes (#314) (#353).

### MSP_BUS_SERVO_CONFIG

New MSP command (152) to retrieve BUS servo source configuration (18 channels).

### MSP_SET_BUS_SERVO_CONFIG

New MSP command (153) to configure individual BUS servo source settings. Payload: U8 index (0-17) + U8 sourceType (0=MIXER, 1=RX).

### MSP_GET_BUS_SERVO_CONFIG

New MSP command (157) to retrieve individual BUS servo source configuration. Payload: U8 index (0-17). Returns: U8 sourceType.

### MSP_SET_SERVO_CONFIG

New MSP command (124) to configure individual servo settings (PWM and BUS servos). Payload: U8 index + 8x U16 fields (mid, min, max, rneg, rpos, rate, speed, flags).

### MSP_GET_SERVO_CONFIG

New MSP command (125) to retrieve individual servo configuration. Payload: U8 index. Returns: 8x U16 fields (mid, min, max, rneg, rpos, rate, speed, flags).

### MSP_SET_SERVO_OVERRIDE_ALL

New MSP command (196) to set servo overrides for all servos in one call. Payload: U16 value (0=enable/center, 2001=disable).

### MSP_SET_SERVO_CENTER

New MSP command (213) to set just the servo center point. Payload: U8 index + U16 mid.

### MSP_GET_MIXER_INPUT

Add msp call to allow retrieving a single mixer line at a time (#361)

### MSP_GET_ADJUSTMENT_RANGE

Add msp call to allow retrieving a single adjustment line at a time (#362)

### MSP_SERVO

Modified to support bus servos:

- Condition: when bus servos are configured — e.g. the `SBUS_OUT` or `FBUS_MASTER` serial function is enabled.
- Returns: configured PWM servo outputs `S1–Sn` (up to `getServoCount()`), followed by all bus servo outputs `S9–S26` (starting at `BUS_SERVO_OFFSET`).
- Skipping behavior: any unconfigured PWM servos between `getServoCount()` and `BUS_SERVO_OFFSET` are skipped.

### MSP_SERVO_CONFIGURATIONS

Modified to support bus servos. When bus servos are configured:

- Mapping (indices → servo outputs):
	- indices `0` to `getServoCount()-1` → PWM servos `S1–S{getServoCount()}`
	- indices `getServoCount()` to `getServoCount()+17` → bus servos `S9–S26`

- Note: `getServoCount()` is used to determine how many PWM servos are present; any unconfigured PWM servos between `getServoCount()` and `BUS_SERVO_OFFSET` are skipped when assembling the returned list.

### MSP_SET_SERVO_CONFIGURATION

Modified to support bus servos. When bus servos are configured, the index parameter is mapped: indices 0 to (getServoCount()-1) map to PWM servos S1-Sn, indices getServoCount() to (getServoCount()+17) map to bus servos S9-S26.

### MSP_RC_TUNING

The `cyclic_ring` parameter is added (#345).

The `cyclic_polar` parameter is added (#426).

### MSP_GET_ADJUSTMENT_FUNCTION_IDS

Added a call to deliver the function id in use per slot (#398)

### MSP_BATTERY_STATE

The `batteryProfile` field is added. (#415)

### MSP_BATTERY_CONFIG

The `batteryCapacity` array is added. (#415)

The `batteryCellCount`, `vbatmincellvoltage`, `vbatmaxcellvoltage`,
`vbatfullcellvoltage` and `vbatwarningcellvoltage` arrays are added, one value
per battery profile. The legacy single-value fields report the active profile.

### MSP_SET_BATTERY_CONFIG

The `batteryCapacity` array is added. (#415)

The `batteryCellCount`, `vbatmincellvoltage`, `vbatmaxcellvoltage`,
`vbatfullcellvoltage` and `vbatwarningcellvoltage` arrays are added (optional).
The legacy single-value fields are stored into the active profile.

### MSP_BATTERY_PROFILE

New MSP command to get the active battery profile. (#415)

### MSP_SET_BATTERY_PROFILE

New MSP command to set the active battery profile. (#415)

### MSP_SETPOINT

New MSP command to get the current setpoint. (#443)

### MSP2_GET_SMARTFUEL_CONFIG

New Rotorflight MSPv2 command (`0x4000`) to get the SmartFuel configuration.

### MSP2_SET_SMARTFUEL_CONFIG

New Rotorflight MSPv2 command (`0x4001`) to set the SmartFuel configuration.


## CLI Changes

`pid_process_denom` is a divider for the PID loop speed vs. the gyro
output data rate (ODR). With #291 the output rate is halved, dropping
the PID loop rate to half too.

`error_rotation` parameter is removed in #294.

`model_set_name` parameter added (ON/OFF). Corresponds with bit 0 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should set the name of the model on the radio.

`model_tell_capacity` parameter added (ON/OFF). Corresponds with bit 1 of `pilotConfig_t.modelFlags` and is used to indicate whether the Lua scripts should announce the remaining capacity of the battery.

`board_name`, `board_design`, and `manufacturer_id` now display a detailed
incompatible-configuration warning and halt the system when an attempt is made
to change them after they have been set. Previously only an error was shown.

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

`bus_servo_source_type` parameter added. Array of 18 uint8 values where element `bus_servo_source_type[i]` selects the source type for BUS servo channel `i` (array indices 0–17 map to channel numbers 0–17). Value selects the source type: `MIXER` = 0, `RX` = 1. The "source index" is equal to the channel number — i.e. when `RX` is selected the RX input index equals the channel number, and when `MIXER` is selected the mixer channel index equals the channel number.

`fbus_master_frame_rate` parameter added. Value in 25..550 Hz, controls the FBUS Master output frame rate.

`fbus_master_pinswap` parameter added (ON/OFF). Swaps TX/RX pins on the FBUS Master serial port.

`fbus_master_inverted` parameter added (ON/OFF). Controls electrical inversion of the FBUS Master UART output.

`freq_input_minhz` parameter added. Value in 1..100 Hz, sets the minimum accepted frequency on the input sensor.

`rates_type` accepts `ROTORFLIGHT` (#345).

`cyclic_ring` meaning is changed. The value indicates % of the max rate.

`resource GYRO_CLK` added. Sets the pin for the gyro synchronisation clock, if supported.

`gyro_offset_yaw` is removed (#391).

`bat_capacity` parameter changed from a single value to an array of 6 values (one for each battery profile).

`bat_profile` parameter added. Value in 0-5, selects the active battery profile.

`battery_cell_count`, `vbat_max_cell_voltage`, `vbat_full_cell_voltage`,
`vbat_min_cell_voltage` and `vbat_warning_cell_voltage` changed from a single
value to an array of 6 values (one for each battery profile). This allows
profiles with different cell counts (e.g. 3S and 4S) and chemistries
(e.g. LiPo and LiHV). A single value in an old `diff` only sets profile 0.
Each profile must be ordered `min` <= `warning` <= `full` <= `max`; a profile
that is not ordered is reset to the defaults.

`pid_gyro_filter_type` and `yaw_precomp_filter_type` parameters are removed (#414).

`serialrx_provider` extended to include `IBUS2` as a protocol option

`smartfuel` parameter added (`OFF`/`VOLTAGE`/`CURRENT`/`COMBINED`). Selects the SmartFuel charge estimator mode.

`smartfuel_voltage_drop_rate` is in **millivolts per second** (mV/s): maximum downward slew of the internal filtered **per-cell** voltage used by the estimator. Range 0..250, default 10.

`smartfuel_charge_drop_rate` is in **0.01% units per second** (maximum drop rate of the displayed percentage once armed). Range 0..250, default 50.

`smartfuel_sag_gain` scales sag compensation from cyclic and collective stick load while airborne. Range 0..100, default 40.


## Defaults

`cbat_alert_percent` changed from 10 to 35 to better reflect heli usage.

`rescue_flip` default is changed from OFF to ON.

`deadband` and `yaw_deadband` defaults changed to 5.

`rc_min_throttle` and `rc_max_throttle` defaults are changed to 0.

`motor_poles` default is changed to 0,0,0,0.

`bus_servo_source_type` defaults to MIXER (0) for first 8 channels, RX (1) for channels 8-17.

`fbus_master_frame_rate` defaults to 500 Hz.

`fbus_master_pinswap` defaults to OFF (0).

`fbus_master_inverted` defaults to ON (SERIAL_INVERTED), which is the standard for FBUS receivers.

`rates_type` default is changed to `ROTORFLIGHT`.

`cyclic_ring` default is changed to 150%.

`rc_threshold` default for collective (4th element) is changed from 50 to 100 (5% to 10% stick) for airborne/hands-on detection.

`gyro_decimation_hz` default is changed to 500Hz (#405).

`blackbox_mode` default is changed to ARMED (#412).

`blackbox_log_governor` default is changed to ON (#412).

`blackbox_rolling_erase` default is changed to ON (#412).

`roll_srate` default is changed to 12 (#413).

`pitch_srate` default is changed to 12 (#413).

`yaw_srate` default is changed to 12 (#413).

`collective_srate` default is changed to 12 (#413).

`error_limit` default is changed to 45,45,60 (#425).

`offset_limit` default is changed to 90,90 (#425).

`stats_min_armed_time_s` default is changed to 15 (#460).


## CRSF Custom Telemetry

`BATTERY_PROFILE` sensor 0x1214 type U8 is added.


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

### Gyro calibration (#391)

The previous gyro calibration (sum/variance and sample counting) is replaced
with a filter-based flow: raw data is passed through a Bessel noise filter,
then split into DC (bias) and high-frequency components via PT filters.
When the minimum sample count (from `gyro_calib_duration`) is reached and
the smoothed high-frequency envelope is below `gyro_calib_noise_limit` on
all axes, the DC estimate is stored as the gyro zero and calibration
completes. The existing CLI parameters are unchanged.

### RPM Filter Presets (#406)

Minor changes introduced to all three presets for better match to common
use cases.

### Servo/Mixer Override disables arming (#431)

A new arming disabled flag `OVERRIDE` was added. It is activated if either
servo or mixer override is active.

### SmartFuel Battery Charge Estimator (#463)

A new battery charge estimator is added that provides a monotonically
non-increasing remaining-charge percentage suitable for telemetry, audible
warnings, and Lua scripts.

The estimator runs at the voltage task rate and combines:
- a slew-rate-limited per-cell voltage (`smartfuel_voltage_drop_rate`),
- a stick-load voltage-sag compensation while airborne, scaled by
  `smartfuel_sag_gain` (cyclic + collective deflection),
- a sigmoid voltage-to-charge curve clamped between `vbat_min_cell` and
  `vbat_full_cell`,
- optional coulomb counting from the measured current and the active
  `bat_capacity` profile,
- a slew-rate limit on the displayed charge level while armed
  (`smartfuel_charge_drop_rate`), so the reported percentage cannot
  bounce back up during flight.

The mode is selected by the `smartfuel` parameter (`OFF` / `VOLTAGE` /
`CURRENT` / `COMBINED`). When enabled, `getBatteryChargeLevel()` returns
the SmartFuel estimate instead of the legacy capacity-based percentage,
and the value is always reported as available regardless of whether
`bat_capacity` is configured.

### RPM Filter is disabled when no RPM source is configured

`RPM_FILTER` requires a real-time RPM source (a frequency sensor or
bidirectional DSHOT telemetry) to engage; ESC telemetry is too slow.
Enabling the feature without one previously left the aircraft unable to
arm, with no clear indication why. The feature is now automatically
disabled instead, at config validation time, if no usable source is
configured.

### No active gyro vibration filter disables arming

Flying with neither the RPM Filter nor the Dynamic Notch Filter actively
engaged leaves the gyro signal unfiltered, which is unsafe. A new arming
disable flag, `NO_NOTCH_FILTER`, is set if neither filter ends up active.
This adds a new bit to `arming_disable_flags` (MSP_STATUS) and shifts
`ARMING_DISABLED_ARM_SWITCH` up by one bit.

## Receiver Protocols

### IBUS 2 Support (#424)

Support for the IBUS2 protocol for control link and basic telemetry using the ibus hub protocol.

## Bug Fixes

### S.PORT telemetry Scaling for attitude sensors

The attitiude sensors where found to be out by a factor of 10.  The scaling
in the firmware has been adjusted to set these correctly. (#313)
