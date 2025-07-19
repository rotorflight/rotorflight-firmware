# Rotorflight Governor

The purpose of the governor is to keep the headspeed constant, regardless of the flight
conditions, amount of collective applied, the battery voltage level, and so on.

It is also ensuring safe operation by limiting the spool up and down speeds,
and dealing with failure situations.

# Basic Operation

First, the Tx throttle channel is controlling the throttle, with a limited change rate.
Once the motor is spinning fast enough, and the headspeed is reached, the governor
takes over, and the input throttle does not matter any more, as long as it is in
the _active_ range, i.e. higher than _handover throttle_.

If the Tx throttle channel drops below _handover throttle_, the governor disengages.

There are multiple settings and flags to alter the details.

## Motor Throttle

The output throttle (servo/ESC) parameters: `min_throttle`, `max_throttle`, and `min_command`
must be set so that `min_throttle` is the lowest running throttle position, and `max_throttle`
is the highest. The range in between must have an active effect on the motor RPM, i.e. there
must _not_ be any deadband near `min_throttle` or `max_throttle`, where a small change would
have no effect at all.

In electric, `min_throttle` is a value where the motor is not running, but would start just
above it. And `max_throttle` is a value where full throttle (WOT) is reached.
It is imperative that the motor will start below 5% throttle, and reach WOT at 100%.

The same applies to I.C. It is strongly adviced _against_ setting the `min_throttle` to
the motor idle level - there is another parameter for setting the idle level. Instead,
the `min_throttle` should be set close to stopping the motor.

The `min_command` is a throttle level _below_ `min_throttle`, where the motor is **guaranteed
to stop** and not run. With an ESC it's also a level that allows the ESC to arm.

**NOTE.** `min_throttle` and `max_throttle` defines the throttle range, that is later
expressed as 0%..100% in the logs and in the telemetry.

# States

The governor operates in multiple states. Each state is used for a specific purpose,
and has its own parameters.

#### State `THROTTLE_OFF`

The motor is turned off.

#### State `THROTTLE_IDLE`

The throttle is below _handover_, and the motor is either off or running slow.
The input throttle is passed to the output, but change speed is limited by
`gov_startup_time`.

The helicopter is expected to be on the ground.

#### State `THROTTLE_HOLD`

The input throttle was switched off while in the `ACTIVE` state. The motor is spooling
down. If the throttle is restored within `gov_throttle_hold_timeout`, a fast recovery
is performed.

This is a safeguard against an accidential throttle cut/hold activation.

#### State `SPOOLUP`

The input throttle was increased above _handover_, and the motor is spooling up to
the required headspeed. The change speed is limited by `gov_spoolup_time`.

Once the target headspeed is reached, the governor moves to the `ACTIVE` state.

#### State `RECOVERY`

The governor is recoving from `FALLBACK` or `THROTTLE_HOLD`, i.e. going back to `ACTIVE`.
This is done with a much faster spoolup speed than in `SPOOLUP`, as the helicopter is likely
airborne. The throttle change is limited by `gov_recovery time`.

#### State `ACTIVE`

The governor is locked on the target headspeed, and the throttle is controlled by
the PID loop. Precomps from collective, cyclic, and yaw are applied.

If the target headspeed is altered, the change speed is limited by `gov_tracking_time`.

#### State `FALLBACK`

RPM signal is unreliable or lost, and the headspeed can't be controlled.
Instead, a throttle value `gov_base_throttle` is applied, with optional precomps, so
that the pilot can land the helicopter safely.

#### State `AUTOROTATION`

Throttle is below _handover_, and _Autorotation_ is enabled. Similar to `THROTTLE_IDLE`,
but the helicopter is expected to be airborne. If the throttle is returned, a fast
`BAILOUT` is performed.

After a succesful landing, the pilot is expected to disable _Autorotation_, and proceed
with a normal `SPOOLUP`.

#### State `BAILOUT`

The autorotation attempt was abandoned, and an _autorotation bailout_ is performed.
Throttle is ramped up quickly.

#### State `DISABLED`

The governor is temporarily disabled, and the throttle is passed to the ESC unaltered.


# Settings

The governor has both global and profile settings. Global settings are always valid.
Profile settings are applied only when a profile is active.

## Global settings

### Governor Modes `gov_mode`

The governor mode setting chooses what type of governor is used, if any.

#### Mode `OFF`

The governor is disabled. The throttle level is passed through unmodified.

#### Mode `EXTERNAL`

An external governor, or throttle curves can be used. The slow spoolup and failure handling
is still done in Rotorflight, but the headspeed stabilisation is left out.

#### Mode `ELECTRIC`

A governor for electric motors is chosen. Rotorflight is taking care of the headspeed
stabilisation and soft start, among other things.

#### Mode `NITRO`

A governor for I.C. / nitro motors is chosen. Rotorflight is taking care of the headspeed
stabilisation and soft start, among other things.

### Startup time `gov_startup_time`

The throttle change speed limit for `IDLE` state.

### Spoolup time `gov_spoolup_time`

The throttle change speed limit for `SPOOLUP` state.

### Tracking time `gov_tracking_time`

The throttle change speed limit for `ACTIVE` state.

### Recovery time `gov_recovery_time`

The throttle change speed limit for `RECOVERY` and `BAILOUT` states.

### Throttle Hold timeout `gov_throttle_hold_timeout`

The timeout for `THROTTLE_HOLD` state.

### Handover Throttle `gov_handover_throttle`

The throttle level required for the governor to activate.

### Voltage Filter `gov_pwr_filter`

A cutoff frequency for the battery voltage filter.

### RPM Filter `gov_rpm_filter`

A cutoff frequency for the headspeed filter.

### TTA Filter `gov_tta_filter`

A cutoff frequency for the TTA control filter.

### Feedforward Filter `gov_ff_filter`

A cutoff frequency for the feedforward/precomp filter.

### D-term Cutoff Frequency `gov_d_cutoff`

A cutoff frequency for the D-term internal filter.


## Profile Settings

### Target headspeed `gov_headspeed`

### Idle throttle value `gov_idle_throttle`

The minimum throttle value in the `IDLE` state.

### Base throttle value `gov_base_throttle`

The throttle value used in the `FALLBACK` state.

### Autorotation throttle value `gov_auto_throttle`

The minimum throttle value in the `AUTOROTATION` state.

### Minimum throttle while active `gov_min_throttle`

The minimum throttle value in the `ACTIVE` state.

### Maximum throttle used `gov_max_throttle`

The maximum throttle value, in any state.

### Master Gain `gov_gain`

The master PID gain.

### P-Gain `gov_p_gain`

### I-Gain `gov_i_gain`

### D-Gain `gov_d_gain`

### F-gain `gov_f_gain`

The total _feedforward_ gain, i.e. a common gain for all precomps.

### P-limits `gov_p_limit`

The P-term range is `[-gov_p_limit, gov_p_limit]`.

### I-limit `gov_i_limit`

The I-term range is `[-gov_i_limit, gov_i_limit]`.

### D-limit `gov_d_limit`

The D-term range is `[-gov_d_limit, gov_d_limit]`.

### F-limit `gov_f_limit`

The F-term range is `[0, gov_f_limit]`.

### Yaw Precomp Weight `gov_yaw_ff_weight`

A relative weight for the yaw precomp.

### Cyclic Precomp Weight `gov_cyclic_ff_weight`

A relative weight for the cyclic precomp.

### Collective Precomp Weight `gov_collective_ff_weight`

A relative weight for the collective precomp.

### Collective Precomp Curve `gov_collective_curve`

Collective curve selector 0..4. Zero is linear curve, higher values have more curvature.

### TTA-gain `gov_tta_gain`

A gain for _Tail Torque Assist_.

### TTA-limit `gov_tta_limig`

A limit for headspeed increase in % due to TTA.


## Profile Flags

### Flag `gov_use_bypass`

The governor is disabled in this profile. The throttle is passed to the output unaltered.
Can be used in I.C. for disabling the governor and flying with the Tx curves only.

### Flag `gov_use_suspend`

The governor PID function in `ACTIVE` state is suspended. Only the F-term is active.
Used for tuning the precomps.

### Flag `gov_use_three_pos_switch`

The input throttle channel is a three-position switch: OFF/IDLE/ACTIVE.

### Flag `gov_use_tx_throttle_curve`

Use the throttle input as the precompensation level, instead of calculating it in the FC.
This allows setting the throttle curve in the Tx.

### Flag `gov_use_fallback_precomp`

Enable the _precompensations_ in the `FALLBACK` state.

### Flag `gov_use_voltage_comp`

Use _battery voltage compensation_. This is useful when the battery has a high internal
resistance. For example, when using Li-ion or LiFePo4 batteries.

### Flat `gov_use_pid_spoolup`

Use a PID controlled spoolup in the `SPOOLUP` state.

### Flag `gov_use_hs_adjustment`

Headspeed target is controlled by the input throttle. The `gov_headspeed` value is
multiplied by the input throttle, giving the target headspeed.
The input throttle must be above `gov_handover_throttle`.

### Flag `gov_use_dyn_min_throttle`

Use _Dynamic Minimum Throttle_ in the `ACTIVE` state. In case the one-way bearing
disengages, the output throttle is not allowed to drop below a value that is
calculated dynamically.

### Flag `gov_use_autorotation`

This is an autorotation profile. Throttle Hold always enables autorotation.


## Mode Switches

There are four new mode switches for the governor.

### `AUTOROTATION`

Enable _autorotation_. Dropping the throttle below `gov_handover_throttle` in `ACTIVE`
causes the `AUTOROTATION` state to be intered, instead of `THROTTLE_IDLE`.

### `FALLBACK`

Trigger an _RPM error_ temporarily, for testing the `FALLBACK` state behaviour.

### `SUSPEND`

Disable PID control in the `ACTIVE` state. Only use the F-term.

### `BYPASS`

Enable _throttle bypass_, effectively disabling all governor functions.

There are no safeguards or change rate limits in `BYPASS`. The pilot **MUST**
ensure safe operation with a suitable programming in the Tx.
