# Rotorflight Governor

The purpose of the governor is to keep the headspeed constant, regardless of the flight
conditions, amount of collective applied, the battery voltage level, and so on.

It is also ensuring safe operation by limiting the spool up and down speeds,
and dealing with failure situations.

## Basic Operation

The Rotorflight RF Governor offers a variety of features, providing flexibility
for both traditional throttle-curve setups and modern switch-based (flight-mode)
setups. It fully supports electric helicopters as well as nitro/IC (glow) engines.

## Governor Modes

The governor mode controls what type of governor is used - if any.

### Governor Mode `OFF`

The governor is completely disabled. The input throttle % is used as the output throttle.

### Governor Mode `EXTERNAL`

The speed governing function in Rotorflight is disabled, but other features, like
slow spoolup and autorotation bailout, remain available. This mode is intended for
use with ESCs that have their own built-in governor, or with traditional throttle
curves in he transmitter.

This mode does not require an RPM signal.

### Governor Mode `ELECTRIC`

Full Rotorflight governor functionality is enabled, optimised for electric motors.
The governor actively maintains the requested headspeed throughout the flight.

Requires an RPM signal from the ESC or a separate RPM sensor.

### Governor Mode `NITRO`

Full Rotorflight governor functionality is enabled, optimised for nitro/IC engines.

Requires a magnetic RPM sensor mounted on the engine or main gear.


## Throttle Channel

The throttle channel is the main control for the motor and the governor.

The throttle channel has a defined _active range_, in which the motor is running.
Below this active range, the motor remains stopped.
The microsecond values for the active range are configured in the Receiver settings
and represented as 0%-100% in the governor.

**Example** with 1100µs-1900µs active range in ELRS:
```
 ┌ stop ┐┌──────────────────────── active ─────────────────────────────┐

         0%                                                          100%
 ├┄┄┄┄┄┄┄┾━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
988    1100                                                          1900    2012
```

**Note!** In order to arm, the throttle channel must be within the _stop_ range.

### Handover Throttle

The handover throttle is a point in the active range, above which the governer
is allowed to operate. Throttle values below this point are considered _idle_.

**Example** with a 25% handover point:
```
        ┌───── idle ─────┬────────── governor enabled ─────────────────┐

        0%              25%                                          100%
 ├┄┄┄┄┄┄┾━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
```

**Note!** In Nitro/IC, the clutch _must_ engage below the handover point, i.e. in
the idle region.


## Throtle Channel Types

Rotorflight provides three throttle channel types, allowing the user to freely
choose the one that best matches their preference and transmitter setup.

### Throttle Type `NORMAL`

The throttle channel is controlling the motor throttle directly and continuously.
The user can use either _throttle-on-stick_ or _throttle-on-switch_. The governor
will engage once the targeted headspeed is reached, and will maintain it until
the throttle is dropped below the handover level.

Slow spoolup can be controlled with the spoolup parameters.

**NOTE!** `NORMAL` requires the throttle channel to have full resolution.
The _wide_ mode channels in ExpressLRS are not suitable for this purpose.

### Throttle Type `SWITCH`

In the idle region, the throttle channel is directly controlling the throttle.
In the active region, the throttle channel governs the headspeed instead.

The transition between idle and run throttle levels shall be instantaneous.
A transmitter switch is typically assigned to toggle between these positions.
In the active region, multiple discrete throttle positions may be used
to select different target headspeeds.

**Example** with 10% idle; 80% and 90% headspeeds:
```
              10%                                    HS:80%  HS:90%
        0%     ▼        25%                             ▼       ▼    100%
 ├┄┄┄┄┄┄┾━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
```

**NOTE!** This throttle type is not compatible with _throttle-on-stick_.

**NOTE!** `SWITCH` requires the throttle channel to have full resolution.
The _wide_ mode channels in ExpressLRS are not suitable for this purpose.

### Throttle Type `FUNCTION`

The desired governor function is selected using a switch on the Tx. The throttle
level determines one of the following functions: OFF, IDLE, AUTO, RUN.

This throttle type is particularly useful when using limited-resolution channels,
like the _wide_ mode channels in ExpressLRS.

```
 ┌ OFF ┐ ┌─────── IDLE ───────┬─────── AUTO ───────┬─────── RUN ────────┐

         0%                  33%                  66%                 100%
 ├┄┄┄┄┄┄┄┾━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
```

**Example setup for ELRS**

A Throttle Cut switch sets the throttle channel to the OFF level.
When TC is not active, another 3-position switch selects between IDLE/AUTO/RUN.

```
 OFF          IDLE                     AUTO                    RUN
988µs        1250µs                   1500µs                  1750µs
 ▼       0%     ▼            33%         ▼        66%            ▼    100%
 ├┄┄┄┄┄┄┄┾━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
```

The ouput throttle for the IDLE and AUTO functions is set with the `gov_idle_throttle`
and `gov_auto_throttle` parameters.


## Output Throttle Settings

The output throttle (servo/ESC) parameters: `min_throttle`, `max_throttle`, and
`min_command` must be set properly for the correct governer operation.

The `min_throttle` and `max_throttle` set the _active range_ of the output throttle,
corresponding to the 0%-100% range in the governor. Any throttle change within this
range must have an actual effect on the motor. There must not be any deadbands,
where any throttle changes have no effect on the motor RPM or power.

On electric, `min_throttle` is a value where the motor is not running, but would start
just above it. And `max_throttle` is a value where full throttle (WOT) is reached.
It is imperative that the motor will start below 5% throttle, and reach WOT at 100%.

The `min_command` is a throttle level _below_ `min_throttle`, where the motor is
**guaranteed to stop** and not run under any circumstances. On electric it's also
the level that allows the ESC to arm.

**NOTE.** `min_throttle` and `max_throttle` define the throttle range, that is
expressed as 0%..100% in the logs and in the telemetry.

**NOTE.** It is strongly adviced _against_ setting the `min_throttle` to the motor
idle level. Instead, the `min_throttle` should be set to the lowest possible position
usable for running the motor, and `gov_idle_throttle` should be used for adjusting the
idling throttle level. Both `min_throttle` and `max_throttle` are setup parameters that
can't be changed at runtime, whereas `gov_idle_throttle` can be adjusted.

### Idle Throttle

In the idle throttle region, the throttle value is passed to the output, and
the user can configure the desired idling throttle level in the transmitter.

Sometimes a single transmitter model is used with multiple helicopters, and
it is preferred to store the idling throttle level in the FC instead.

When `gov_idle_throttle` is set to a non-zero value, it establishes a minimum
motor output throttle. This allows the desired idle throttle level to be defined
in the flight controller, while the transmitter may still temporarily output
a higher idle throttle value.

**Example** with 8% idle throttle:
```
        ┌──── input ─────────┄┄

        0%    8%                                                     100%
 ├┄┄┄┄┄┄┾━━━━━┷━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤

              └─ output ─────┄┄
```

## Autorotation

The governor can perform a fast bailout from autorotation if it can detect
when autorotation is being attempted. Autorotation is detected when, while
the governor is active, the throttle is suddenly reduced to a level within
a predefined autorotation range.

In order to enable autorotation detection, the autorotation timeout and
the autorotation throttle range must be configured.

### Auto Timeout

The _autorotation timeout_ defines the maximum duration of an autorotation.
After this timeout expires, the governor will no longer initiate a bailout.

Autorotation bailout is disabled unless this timeout is explicitly configured.

### Auto Throttle

The autorotation throttle region is defined by the `gov_auto_throttle` parameter.
Autorotation is detected when the input throttle is above `gov_auto_throttle` and
below `gov_handover_throttle`.

Autorotation is canceled when the input throttle falls below the autorotation region.
A bailout from autorotation is triggered when the input throttle rises above
the handover throttle.

**Example** with 12% auto throttle:
```
        ┌ idle ─┬─ auto ─┬────────── governor enabled ─────────────────┐

        0%     12%      25%                                          100%
 ├┄┄┄┄┄┄┾━━━━━━━┷━━━━━━━━┿━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━┽┄┄┄┄┄┄┄┤
```


## States

The governor operates in multiple states. Each state represents a different scenario, with
different functionality and different parameters. For example, _spoolup_, _active_, etc.

The governor is automatically moving between the states, based on the throttle channel
value, the motor behaviour, and the previous state.

The governor state is available as a telemetry sensor, and can be displayed on
the transmitter.

#### State `THROTTLE_OFF`

The motor is turned off. The value `min_command` is sent to the ESC/servo.

#### State `THROTTLE_IDLE`

The throttle is below _handover_, and the motor is either stopped or running slow.
The input throttle is passed to the output, but change speed is limited by
`gov_startup_time`.

The helicopter is expected to be on the ground.

#### State `THROTTLE_HOLD`

The input throttle was switched off while in the `ACTIVE` state. The motor is spooling
down. If the throttle is restored within `gov_throttle_hold_timeout`, a fast recovery
is performed.

This is a safeguard against an accidential Throttle Cut activation.

#### State `SPOOLUP`

The input throttle was increased above _handover_, and the motor is spooling up to
the required headspeed. The change speed is limited by `gov_spoolup_time`.

Once the target headspeed is reached, the governor moves to the `ACTIVE` state.

#### State `RECOVERY`

The governor is recoving from `FALLBACK` or `THROTTLE_HOLD`, i.e. going back to `ACTIVE`.
This is done with a much faster spoolup speed than in `SPOOLUP`, as the helicopter is
likely airborne. The throttle change is limited by `gov_recovery time`.

#### State `ACTIVE`

The governor is locked on the target headspeed, and the throttle is controlled by
the PID loop. Precomps from collective, cyclic, and yaw are applied.

If the target headspeed is altered, the change speed is limited by `gov_tracking_time`.

#### State `FALLBACK`

RPM signal is unreliable or lost, and the headspeed can't be controlled.
Instead, an estimated throttle value is used, with optional precomps, so
that the pilot can land the helicopter safely.

The parameter `gov_fallback_drop` sets the percentage of throttle drop for
indicating this condition, in order to notify the pilot.

#### State `AUTOROTATION`

Throttle is above _autorotation throttle_, but below _handover_, and autorotation is
active. Similar to `THROTTLE_IDLE`, but the helicopter is expected to be airborne.
If the throttle is returned, a fast `BAILOUT` is performed.

After a succesful landing, the pilot is expected to disable autorotation by moving to
an idle throttle level, and proceed with a normal `SPOOLUP`.

#### State `BAILOUT`

The autorotation attempt was abandoned, and an _autorotation bailout_ is performed.
Throttle is ramped up quickly, according to the `gov_recovery_time` parameter.

#### State `BYPASS`

The governor is temporarily disabled, and the throttle is passed to the ESC unaltered.
The throttle value is calculated from the collective position, using `gov_bypass_throttle`.


# Settings

The governor has both global and profile settings. Global settings are always valid.
Profile settings are applied only when a profile is active.

## Global settings

### Governor Mode `gov_mode`

The governor mode setting chooses what type of governor is used:
`OFF`, `EXTERNAL`, `ELECTRIC`, `NITRO`

### Throttle Channel Type `gov_throttle_type`

The type of the Throttle channel: `NORMAL`, `SWITCH`, `FUNCTION`

### Startup time `gov_startup_time`

The throttle change speed limit for `IDLE` state.

### Spoolup time `gov_spoolup_time`

The throttle change speed limit for `SPOOLUP` state.

### Tracking time `gov_tracking_time`

The throttle change speed limit for `ACTIVE` state.

### Recovery time `gov_recovery_time`

The throttle change speed limit for `RECOVERY` and `BAILOUT` states.

### Recovery time `gov_spooldown_time`

The throttle reduction speed limit for `THROTTLE_IDLE` state.

### Throttle Hold timeout `gov_throttle_hold_timeout`

The timeout for `THROTTLE_HOLD` state.

### Throttle Hold timeout `gov_autorotation_timeout`

The timeout for `AUTOROTATION` state.

### Handover Throttle `gov_handover_throttle`

The minimum throttle level required for the governor to activate.

### Idle throttle value `gov_idle_throttle`

The minimum throttle value in the `IDLE` state.

### Autorotation throttle value `gov_auto_throttle`

The minimum throttle value in the `AUTOROTATION` state.

### Voltage Filter `gov_pwr_filter`

A cutoff frequency for the battery voltage filter.

### RPM Filter `gov_rpm_filter`

A cutoff frequency for the headspeed filter.

### TTA Filter `gov_tta_filter`

A cutoff frequency for the TTA control filter.

### Feedforward Filter `gov_ff_filter`

A cutoff frequency for the feedforward/precomp filter.

### D-term Filter `gov_d_filter`

A cutoff frequency for the D-term internal filter.

### Throttle Curve for BYPASS `gov_bypass_throttle`

A curve for calculating throttle value from the collective position.
Used only in the governor bypass.

An array of 9 values in 0..200, indicating 0..100% throttle with 0.5% steps.


## Profile Settings

### Target headspeed `gov_headspeed`

The (full) headspeed for the profile.

### Minimum throttle while active `gov_min_throttle`

The minimum throttle value in the `ACTIVE` state.
If set, it must be higher than handover throttle.

### Maximum throttle used `gov_max_throttle`

The maximum throttle value, in any state.

### Master Gain `gov_gain`

The master PID gain.

### P-Gain `gov_p_gain`

The governor PID loop P-gain.

### I-Gain `gov_i_gain`

The governor PID loop I-gain.

### D-Gain `gov_d_gain`

The governor PID loop D-gain.

### F-gain `gov_f_gain`

The total _feedforward_ gain, i.e. a common gain for all precomps.

### P-limits `gov_p_limit`

The P-term range is `[-gov_p_limit, gov_p_limit]`.

### I-limit `gov_i_limit`

The I-term range is `[-gov_i_limit, gov_i_limit]`.

### D-limit `gov_d_limit`

The D-term range is `[-gov_d_limit, gov_d_limit]`.

### F-limit `gov_f_limit`

The F-term range is `[-gov_f_limit, gov_f_limit]`.

### Yaw Precomp Weight `gov_yaw_ff_weight`

A relative weight for the yaw precomp.

### Cyclic Precomp Weight `gov_cyclic_ff_weight`

A relative weight for the cyclic precomp.

### Collective Precomp Weight `gov_collective_ff_weight`

A relative weight for the collective precomp.

### Collective Precomp Curve `gov_collective_curve`

Collective curve exponent (/10). Values 15..25 are likely good.

### Dynamic Throttle Minimum `gov_dyn_min_throttle`

The percentage that the throttle is allowed to suddenly drop from
a steady state level. This prevents the throttle from dropping
too low during overspeed situations.

### TTA-gain `gov_tta_gain`

A gain for _Tail Torque Assist_.

### TTA-limit `gov_tta_limit`

A limit for headspeed increase in % due to TTA.


## Profile Flags

### Flag `gov_use_voltage_comp`

Use _battery voltage compensation_. This is useful when the battery has a high internal
resistance. For example, when using Li-ion or LiFePo4 batteries.

### Flat `gov_use_pid_spoolup`

Use a PID controlled spoolup in the `SPOOLUP` state.

### Flag `gov_use_dyn_min_throttle`

Use _Dynamic Minimum Throttle_ in the `ACTIVE` state. In case the one-way bearing
disengages, the output throttle is not allowed to drop below a value that is
calculated dynamically.

### Flag `gov_use_fallback_precomp`

Enable the _precompensations_ in the `FALLBACK` state.


## Mode Switches

There are three new mode switches for the governor.

### `FALLBACK`

Trigger an _RPM error_ temporarily, for testing the `FALLBACK` state behaviour.

### `SUSPEND`

Disable PID control in the `ACTIVE` state - only use the F-term.

### `BYPASS`

Enable _throttle bypass_, effectively disabling all governor functions.

In `BYPASS` mode, no safeguards or throttle rate limiting are applied.
The throttle output is taken directly from the value derived from
the collective position.


# Examples

### Simple One Headspeed (electric)

The most simple setup is to have the throttle channel controlled by
a Throttle Cut switch. The headspeed is set in the governor profile.

```
set gov_throttle_type = NORMAL
set gov_headspeed = 2400
```
No other settings are needed.

### Simple Throttle-on-Stick (electric)

The traditional throttle-on-stick can be set up with the throttle curve in
the Tx. The flat part of the curve should be set to 100%.

The required headspeed is set in the governor profile.

```
set gov_throttle_type = NORMAL
set gov_headspeed = 2400
```

It is strongly suggested to have a _Throttle Cut_ switch with this setup.



