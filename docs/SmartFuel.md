# SmartFuel

SmartFuel provides an intelligent remaining-fuel percentage for the flight pack, derived either from measured current consumption or from voltage-based estimation when no current sensor is available.

It is intended to give a more useful "fuel remaining" value than a simple battery percentage, especially for telemetry display on the radio.

## Configuration

SmartFuel supports two sources:

- `CURRENT` uses battery capacity and consumed mAh from the current meter.
- `VOLTAGE` estimates remaining fuel from pack voltage behaviour when no current sensor is available.

CLI examples:

```text
set smartfuel = ON
set smartfuel_source = CURRENT
```

```text
set smartfuel = ON
set smartfuel_source = VOLTAGE
set smartfuel_stabilize_delay = 1500
set smartfuel_stable_window = 15
set smartfuel_voltage_fall_limit = 5
set smartfuel_fuel_drop_rate = 10
set smartfuel_sag_multiplier = 70
```

## SmartFuel tuning parameters

CLI parameter names:

1. `smartfuel_stabilize_delay`
2. `smartfuel_stable_window`
3. `smartfuel_voltage_fall_limit`
4. `smartfuel_fuel_drop_rate`
5. `smartfuel_sag_multiplier`

The first two parameters apply in both `CURRENT` and `VOLTAGE` modes:

- `smartfuel_stabilize_delay`
- `smartfuel_stable_window`

The remaining three parameters are only used in `VOLTAGE` mode.

Default values:

```text
smartfuel_stabilize_delay = 1500
smartfuel_stable_window = 15
smartfuel_voltage_fall_limit = 5
smartfuel_fuel_drop_rate = 10
smartfuel_sag_multiplier = 70
```

## Parameter guide

### `smartfuel_stabilize_delay`

How long SmartFuel waits after reset, battery connection or configuration change before trying to decide that pack voltage is stable.

This applies in both `CURRENT` and `VOLTAGE` modes.

- Increase it if the reported value is noisy or jumps around immediately after plugging in.
- Decrease it if you want the value to settle sooner.

Units: milliseconds.

### `smartfuel_stable_window`

How tightly grouped the recent voltage samples must be before the pack is considered stable.

This applies in both `CURRENT` and `VOLTAGE` modes.

- Increase it if SmartFuel takes too long to settle on a value.
- Decrease it if SmartFuel settles too early while the pack voltage is still moving around.

Units: centivolts.

### `smartfuel_voltage_fall_limit`

Maximum allowed downward movement of the filtered voltage in voltage mode.

- Increase it if SmartFuel reacts too slowly to real pack depletion.
- Decrease it if throttle punches or brief sag make SmartFuel fall too quickly.

Units: centivolts per second.

### `smartfuel_fuel_drop_rate`

Maximum allowed SmartFuel percentage drop rate once the model has flown.

- Increase it if the displayed percentage lags too much behind the real pack condition.
- Decrease it if the percentage drops too aggressively during load spikes.

Units: tenths of a percent per second.

### `smartfuel_sag_multiplier`

Amount of sag compensation applied in voltage mode.

- Increase it if SmartFuel is too pessimistic under load.
- Decrease it if SmartFuel is too optimistic during hard collective or cyclic loading.

Units: percent.

## Tuning guidance

### Current mode

`CURRENT` is the preferred mode when a real current sensor is available.

Before tuning SmartFuel itself, make sure:

- battery voltage calibration is correct
- current meter calibration is correct
- battery capacity is configured correctly for the active battery profile
- the consumption warning percentage is set to the reserve you actually want to keep

If those values are right, current-mode SmartFuel should usually need little or no tuning.

The only SmartFuel tuning values that still matter in `CURRENT` mode are:

- `smartfuel_stabilize_delay`
- `smartfuel_stable_window`

### Voltage mode

`VOLTAGE` is intended for setups without a current sensor. Start with the defaults and change one parameter at a time.

Suggested workflow:

1. Fly with the default values.
2. Check whether SmartFuel settles quickly and sensibly after plugging in.
3. Watch how it behaves during strong climb-outs, hard pitch pumps and unloaded recovery.
4. Compare the displayed remaining fuel at landing with the pack voltage and with how much reserve you wanted to keep.

Use these adjustment patterns:

- Too jumpy before flight: increase `smartfuel_stabilize_delay` or reduce `smartfuel_stable_window`.
- Settles too slowly before flight: increase `smartfuel_stable_window` or reduce `smartfuel_stabilize_delay`.
- Drops too hard during load: reduce `smartfuel_fuel_drop_rate` or increase `smartfuel_sag_multiplier`.
- Feels too pessimistic overall under load: increase `smartfuel_sag_multiplier`.
- Feels too optimistic overall: reduce `smartfuel_sag_multiplier`.
- Tracks real depletion too slowly throughout the flight: increase `smartfuel_voltage_fall_limit` or increase `smartfuel_fuel_drop_rate`.

## Practical notes

- Tune with the battery type and flying style you actually use.
- Large changes are rarely needed; make small adjustments and re-fly.
- Different packs may want slightly different behaviour, but the defaults are intended to be a reasonable starting point.
- Voltage mode is still an estimate. Current mode will generally be more repeatable when a well-calibrated current sensor is available.

## Telemetry

When `smartfuel = OFF`, telemetry keeps the existing master behaviour.

When `smartfuel = ON`, SmartFuel reuses the existing battery fuel and consumption outputs instead of adding new telemetry sensors:

- the existing fuel/charge-level output reports SmartFuel remaining percentage
- the existing consumption output reports measured mAh in `CURRENT` mode
- the existing consumption output reports estimated mAh in `VOLTAGE` mode
