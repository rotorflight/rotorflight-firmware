## Tail Guard

Tail Guard is a new optional tail-protection feature intended to reduce the chance of tail blowout under high collective load, high yaw demand, or reduced headspeed.

Current first-pass behavior:

- Estimates a `tail guard risk` value from yaw demand, collective demand, and headspeed drop.
- Reduces yaw command as risk rises.
- Increases yaw feedforward / precomp as risk rises.
- Can be disabled, always enabled, or enabled from an AUX mode.

## User-Facing Options

### `tail_guard_mode`

Controls how Tail Guard is enabled:

- `OFF`: Tail Guard disabled.
- `ON`: Tail Guard always active for the selected PID profile.
- `AUX`: Tail Guard active only while the `TAIL GUARD` mode switch is enabled.

### `tail_guard_strength`

Overall strength multiplier for the Tail Guard risk calculation.

- Higher values make Tail Guard react sooner and more strongly.
- Lower values make it less active.

### `tail_guard_yaw_limit`

Maximum amount of yaw-command reduction applied as Tail Guard risk increases.

- Higher values allow more yaw limiting to protect tail authority.
- Lower values preserve more commanded yaw rate.

### `tail_guard_boost`

Extra yaw feedforward / precomp boost applied as Tail Guard risk increases.

- Higher values add more proactive tail support.
- Lower values keep the effect more subtle.

### `tail_guard_collective_gain`

How strongly collective demand contributes to Tail Guard risk.

- Higher values make pitch-load changes influence Tail Guard more.
- Lower values reduce the collective contribution.

### `tail_guard_headspeed_gain`

How strongly headspeed drop contributes to Tail Guard risk.

- Higher values make lost headspeed increase Tail Guard intervention more.
- Lower values reduce headspeed sensitivity.

## AUX Mode

A new AUX mode is added:

- `TAIL GUARD`

This mode is used when `tail_guard_mode = AUX`.

## Initial Default Values

- `tail_guard_mode = OFF`
- `tail_guard_strength = 100`
- `tail_guard_yaw_limit = 35`
- `tail_guard_boost = 25`
- `tail_guard_collective_gain = 100`
- `tail_guard_headspeed_gain = 125`

## Notes

- This is intentionally a conservative first-pass implementation.
- The current implementation is CLI-configurable.
- The goal is to make the tail hold together better before a blowout starts, not to replace proper mechanical setup.
