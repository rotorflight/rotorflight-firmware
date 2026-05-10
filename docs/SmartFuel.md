# SmartFuel

SmartFuel provides an intelligent remaining-charge percentage for the flight pack, derived from pack voltage. It is intended to give a more useful "fuel remaining" value than the simple linear voltage-to-percent curve, especially for telemetry display on the radio.

SmartFuel is a voltage-only estimator. It does not use current, consumption, or battery capacity. Measured consumption (mAh used) is reported separately by the battery monitoring code and is unaffected by SmartFuel.

## How charge level is chosen

`getBatteryChargeLevel()` picks the most accurate source available, in this order:

1. **SmartFuel**, if `smartfuel = ON`. SmartFuel always wins when enabled.
2. **Consumption-based**, if `bat_capacity` is non-zero. Reported as `100 × (capacity − used) / capacity`. Without a configured current sensor `used` stays at zero, so this path will report `100%` indefinitely — set `bat_capacity = 0` to fall through to the linear-voltage estimate instead.
3. **Linear voltage**, if cell count is known. A simple linear interpolation between `vbat_min_cell_voltage` and `vbat_max_cell_voltage`.

## Configuration

SmartFuel is enabled by setting `smartfuel = ON`. Set `smartfuel = OFF` to disable it; the firmware then falls back to consumption-based or linear-voltage charge level as described above.

SmartFuel anchors its initial percent on the first voltage sample after the battery monitoring code declares the pack present (i.e. once cell-count auto-detection has completed). It does not run its own settling timer.

CLI example:

```text
set smartfuel = ON
set smartfuel_voltage_fall_rate = 5
set smartfuel_charge_drop_rate = 10
set smartfuel_sag_multiplier = 70
```

## SmartFuel tuning parameters

CLI parameter names:

1. `smartfuel_voltage_fall_rate`
2. `smartfuel_charge_drop_rate`
3. `smartfuel_sag_multiplier`

Default values:

```text
smartfuel_voltage_fall_rate = 5
smartfuel_charge_drop_rate = 10
smartfuel_sag_multiplier = 70
```

## Parameter guide

### `smartfuel_voltage_fall_rate`

Maximum allowed downward movement of the filtered voltage used as input to the charge-level estimator.

- Increase it if SmartFuel reacts too slowly to real pack depletion.
- Decrease it if throttle punches or brief sag make SmartFuel fall too quickly.

Units: centivolts per second.

### `smartfuel_charge_drop_rate`

Maximum allowed SmartFuel percentage drop rate once the model has been armed (active during arm and from then on).

- Increase it if the displayed percentage lags too much behind the real pack condition.
- Decrease it if the percentage drops too aggressively during load spikes.

Units: tenths of a percent per second.

### `smartfuel_sag_multiplier`

Amount of sag compensation applied to the voltage reading before it is mapped to a percentage.

- Increase it if SmartFuel is too pessimistic under load.
- Decrease it if SmartFuel is too optimistic during hard collective or cyclic loading.

Units: percent.

## Tuning guidance

Start with the defaults and change one parameter at a time.

Suggested workflow:

1. Fly with the default values.
2. Check whether SmartFuel settles quickly and sensibly after plugging in.
3. Watch how it behaves during strong climb-outs, hard pitch pumps and unloaded recovery.
4. Compare the displayed remaining fuel at landing with the pack voltage and with how much reserve you wanted to keep.

Use these adjustment patterns:

- Drops too hard during load: reduce `smartfuel_charge_drop_rate` or increase `smartfuel_sag_multiplier`.
- Feels too pessimistic overall under load: increase `smartfuel_sag_multiplier`.
- Feels too optimistic overall: reduce `smartfuel_sag_multiplier`.
- Tracks real depletion too slowly throughout the flight: increase `smartfuel_voltage_fall_rate` or increase `smartfuel_charge_drop_rate`.

## Practical notes

- Tune with the battery type and flying style you actually use.
- Large changes are rarely needed; make small adjustments and re-fly.
- Different packs may want slightly different behaviour, but the defaults are intended to be a reasonable starting point.
- SmartFuel is still an estimate. A consumption-based percentage from a well-calibrated current sensor and a correctly configured `bat_capacity` will generally be more repeatable; if you have that data and want to use it, set `smartfuel = OFF`.

## Telemetry

When `smartfuel = ON`, SmartFuel drives the existing fuel/charge-level telemetry output (CRSF, FrSky, iBUS, MSP_BATTERY_STATE, LED strip, …) — anything routed through `getBatteryChargeLevel()`. The on-FC OSD elements are not routed through that path; they read `bat_capacity` and used mAh directly, so they continue to show the consumption-based percentage regardless of `smartfuel`. Battery consumption telemetry is also unchanged regardless of `smartfuel`: it always reports the measured `currentMeter` capacity, which is `0` when no current sensor is configured.
