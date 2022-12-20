
# Alpha Development version of Rotorflight-2.0 (4.3.x)

This is a snapshot of the ongoing Rotorflight 4.3 development.
It is called _ALPHA_ because lots of things are broken.

# Configuration

## Hardware Config

Should be the same as in RF1.

All hardware related CLI commands (dump hardware) should work.


## Features

Currently there are no changes to feature flags.


## Servos

The servo configuration has been refactored, and has slightly different parameters.

### Servo Configuration

The CLI `servo` command has now the following syntax:

```
servo <center> <min> <max> <neg-scale> <pos-scale> <update-rate> <flags>

<center>       zero position in us (servo arm level position)
<min>          minimum pulse length (for avoiding binding)
<max>          maximum pulse length (for avoiding binding)
<neg-scale>    scaling factor on negative side (below center level)
<pos-scale>    scaling factor on positive side (above center level)
<update-rate>  PWM update rate in Hz
<flags>        flags (binary values)
```

### Servo Flags

The servo flags has the following binary values:

```
1             Servo reverse
2             Geometry correction
```

Servo _geometry correction_ for correcting the rotational geometry.
It is _not_ needed with linear servos.


## Motors

### `motor_rpm_factor = <value,value,value,value>`

This is a correction factor for any systematic scaling error on the Dshot RPM reading.
The value or 0 is no correction, 100 is +1.00% correction, -200 is -2.00%, etc.

There is a separate value for each motor.


## Modes / AUX

The CLI commands are the same, but the range (PWM) resolution
has been increased from 25 to 5us.


## Adjustment Functions

Adjustment functions have been totally rewritten.

There are now two new ranges in each adjustment, for setting the Rx ranges where
the increment/decrement step is triggered, respectively.

With the continuous adjustment, the first range sets the input range of the control channel.

In RF1, the decrement range was 1000..1300, and the increment range was 1700..2000.
The continuous input range was 1000..2000.

The CLI command format is:

```
adjfunc <index> <func> <enable channel> <start> <end>
        <value channel> <dec start> <dec end> <inc start> <inc end>
        <step size> <value min> <value max>
```

This allows one channel to inc/dec at least 8 different adjustment functions.


## Gyro Filtering

Most gyro filtering related CLI commands have been renamed in BF4.3.
There are further changes in Rotorflight.

### Filter Task Speed

It is now possible to run the filtering task at a different speed from the PID loop.
This is needed if the PID loop is running at a low speed (1Khz or lower).

There is a new parameter `filter_process_denom` that works the same as `pid_process_denom`.
A special value 0 indicates that the filter task is running at PID loop speed.
Other values are relative to the gyro speed.

### Filter Configuration

CLI commands listed below.

```
set gyro_decimation_hz = 250
set gyro_lpf1_type = BIQUAD
set gyro_lpf1_static_hz = 150
set gyro_lpf1_dyn_min_hz = 25
set gyro_lpf1_dyn_max_hz = 200
set gyro_lpf2_type = PT1
set gyro_lpf2_static_hz = 0
set gyro_notch1_hz = 0
set gyro_notch1_cutoff = 0
set gyro_notch2_hz = 0
set gyro_notch2_cutoff = 0
set dterm_lpf1_type = PT1
set dterm_lpf1_static_hz = 15
set dterm_lpf1_dyn_min_hz = 0
set dterm_lpf1_dyn_max_hz = 0
set dterm_lpf2_type = PT1
set dterm_lpf2_static_hz = 0
set dterm_notch_hz = 0
set dterm_notch_cutoff = 0
```

## Blackbox

Blackbox has been refactored in BF4.3, and further in RF2.

#### `blackbox_mode = <OFF|NORMAL|ARMED|SWITCH>`

```
OFF     = No logging
NORMAL  = logging when both ARMED and BLACKBOX switch active
ARMED   = logging when ARMED
SWITCH  = logging when BLACKBOX switch active
```

In NORMAL mode, logging is paused if BLACKBOX switch is inactive.

#### `blackbox_sample_rate = <1/N>`

Blackbox sampling is now locked to the PID loop, and can be reduced
by a factor of 2,4,8,16,32,etc.


#### `blackbox_log_XYZ = <ON|OFF>`

Each blackbox field can be now separately enabled/disabled.

```
set blackbox_log_command = ON
set blackbox_log_setpoint = ON
set blackbox_log_mixer = ON
set blackbox_log_pid = ON
set blackbox_log_gyro_raw = ON
set blackbox_log_gyro = ON
set blackbox_log_acc = ON
set blackbox_log_mag = OFF
set blackbox_log_alt = ON
set blackbox_log_battery = ON
set blackbox_log_rssi = ON
set blackbox_log_rpm = ON
set blackbox_log_motors = ON
set blackbox_log_servos = ON
set blackbox_log_gps = OFF
```

## Debug

Debug has now eight 32bit values, instead of four 16bit values.

This will require changes in the Blackbox Explorer eventually.

Some debug modes are logging only one axis, which is selected
with the `debug_axis` parameter.

This parameter replaces the following BF parameters:

- gyro_filter_debug_axis
- acro_trainer_debug_axis
- rc_smoothing_debug_axis


## Sensors

New settings for sensor update speed:

```
set vbat_update_hz = 100
set ibat_update_hz = 100
set esc_sensor_update_hz = 100
```

## Mixer

#### `collective_correction = <ON|OFF>`

There is a new parameter for correcting the collective lift changes due to TTA.


## Profile (PID Profile)

PID control has been totally rewritten.

There are now multiple PID controllers to choose from.

#### `pid_mode = 0`

This is a test mode with the FF term only. Useful for debugging the firmware.

#### `pid_mode = 1`

Rotorflight-1.0 compatibility mode. This is exactly the same as RF1.
Parameters have different names, unfortunately.

#### `pid_mode = 2`

New PID mode under development. This is the current idea what RF2 could be.

#### `pid_mode = 9`

Test PID mode with a ridiculous number of parameters. Used for testing new ideas.

This mode has separate PID gains for yaw CW & CCW rotation.

The "other" direction gains are called "way" gains (as in yaw moving backwards...)

```
set way_p_gain = 50
set way_i_gain = 60
set way_d_gain = 10
set way_f_gain = 0
```

#### `pid_dterm_mode = <0|1>`

Selects between D-term on gyro signal (0) vs. on error signal (1).

Available in modes 2 and 9.


### PID Gains

The PID gain parameters have been renamed. Should be obvious.

#### `yaw_[c]cw_stop_gain = <N>`

Stop gain for yaw. Works differently in each mode.

Mode 1: Same as RF1.0. P and D gains multiplied by stop gain.

Mode 2: Only P gain multiplied by stop gain.

Mode 9: Not used


### PID Filters

There are a few extra filters in the PID controller (for evaluation).

#### `pitch_d_cutoff = <Hz>`

D-term bandwidth limit, separate for each axis.

Available in modes 2 and 9. Equivalent to RF1 D-term PT1 filter.
Must be set to a reasonable value for D-term to work.
Typical values 15..30.

#### `pitch_f_cutoff = <Hz>`

FF-term highpass filter cutoff. 0 = disabled.

A new idea about how to do FF. Needs evaluation.

Available in modes 2 and 9.

#### `pitch_error_cutoff = <Hz>`

Error term bandwidth limit. 0 = disabled.

Available in modes 1, 2 and 9.

#### `pitch_gyro_cutoff = <Hz>`

Gyro signal bandwidth limit, separate for each axis. 0 = disabled.
This is an extra PT1 filter for each gyro axis.

Available in modes 2 and 9.

#### `iterm_relax_type = <OFF|RP|RPY>`

Selects the I-term relax type, i.e. which axis to apply to.

#### `iterm_relax_level = <N>`

Sets the setpoint limit/level for I-term relax. This is fixed to 40 in BF and RF1.

#### `iterm_relax_cutoff = <R,P,Y>`

I-term relax filter cutoffs for roll, pitch, and yaw.


## Rates

The rates system has been rewritten for RF2.0.

There is now a curve also for collective. It is mostly useful for setting
the collective min/max, but also expo.

#### `collective_rc_rate = <N>`

#### `collective_srate = <N>`

#### `collective_expo = <N>`

In addition to the rate curves, it is now possible to set
_limits_, _acceleration_, and _smoothness_.

#### `yaw_rate_accel = <N>`

Set waw setpoint max acceleration.

#### `yaw_rate_limit = <N>`

Set absolute maximum for yaw setpoint. The curve will usually control what to value is at the stick extreme, so this is an extra limit.

#### `rates_smoothness = <N>`

Set overall smoothness for setpoint changes. This is equivalent to "Style" or "Smoothness" in some FBLs.
Higher is smoother. Typical values 25-200.

#### `cyclic_ring = <N>`

Set a ring limiter for cyclic setpoints.
Range 0 = no limit ... 100 = round circle


## Governor

#### `gov_yaw_ff_weight = <N>`

A new parameter for yaw-to-throttle precompensation. This is only for main motor
driven tails. For motorised tails, this parameter must be set to zero.

#### `gov_startup_time = <N>`

A time constant for throttle rampup during start. This is applied only
until the motor has started up and the RPM signal is detected.


## ESC Telemetry

#### `esc_sensor_protocol = <NONE|KISS|HOBBYWINGV4|KONTRONIK>`

#### `esc_sensor_hw4_current_offset = <N>`

An offset value for calibrating the zero current with Hobbywing V4 ESCs.

#### `esc_sensor_hw4_voltage_gain = <N>`

Voltage gain - different for each HW V4 model.

```
 *
 * Voltage Gain:
 *   3-6S  (LV):    gain = 110
 *   3-8S  (LVv2):  gain = 154
 *   5-12s (HV):    gain = 210
 *
```

#### `esc_sensor_hw4_current_gain = <N>`

Current gain - different for each HW V4 model.

```
 *
 * Current Gain:
 *   60A:           gain = 60
 *   80A:           gain = 78
 *   100A:          gain = 90
 *   120A:          gain = 100
 *   130A:          gain = 113
 *   150A:          gain = 129
 *   160A:          gain = 137
 *   200A:          gain = 169
 *
```

## Rescue

#### `rescue_mode = <OFF|CLIMB|ALT_HOLD>`

Rescue mode. Climb or Altitude Hold.

Altitude Hold requires a good quality barometer.

#### `rescue_flip = <OFF|ON>`

Enable flipping the heli upright, if rescue started inverted.

#### `rescue_flip_gain = <N>`

Gain for initial pull up and for flipping the heli upright.

#### `rescue_level_gain = <N>`

Gain for leveling the heli.

This is 2.5x less effective vs. RF1, e.g. you had 40, now you need 100.

#### `rescue_pull_up_time = <N>`

Time for the initial pull-up. (1/10s steps)

#### `rescue_climb_time = <N>`

Time for climbing. (1/10s steps)

#### `rescue_flip_time = <N>`

Time(out) for flipping. (1/10s steps)
If flip did not complete withint this time, climb state is entered regardless.

#### `rescue_exit_time = <N>`

Time for slowly exiting rescue. (1/10s steps)

#### `rescue_pull_up_collective = <N>`

Collective value for pull-up. (1/1000 steps)

#### `rescue_max_rate = <N>`

Maximum cyclic rate for any rescue action.

#### `rescue_max_accel = <N>`

Maximum cyclic acceleration for any rescue action.

### Rescue Climb Mode parametes

#### `rescue_climb_collective = <N>`

Collective value for climb.

#### `rescue_hover_collective = <N>`

Collective value for hover.

### Rescue Altitude Hold Mode parameters

#### `rescue_hover_altitude = <N>`

Hovering altitude in cm.

#### `rescue_alt_a_gain = <N>`

Altitude to climb-rate gain.

#### `rescue_alt_p_gain = <N>`

Climb Rate P-gain.

#### `rescue_alt_i_gain = <N>`

Climb Rate I-gain.

#### `rescue_max_climb_rate = <N>`

Maximum climb rate for Altitude Hold mode (cm/s).

