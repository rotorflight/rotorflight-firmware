# SmartFuel

SmartFuel provides an intelligent remaining-fuel percentage for the flight pack, derived either from measured current consumption or from voltage-based estimation when no current sensor is available.

It is intended to give a more useful "fuel remaining" value than a simple battery percentage, especially for telemetry display on the radio.

## Configuration

SmartFuel supports two sources:

- `CURRENT` uses battery capacity and consumed mAh from the current meter.
- `VOLTAGE` estimates remaining fuel from pack voltage behaviour when no current sensor is available.

CLI examples:

```text
set smartfuel_source = CURRENT
```

```text
set smartfuel_source = VOLTAGE
set smartfuel_params = 1500,15,5,10,2,70
```

## SmartFuel parameter order

`smartfuel_params` is a 6-value array in this order:

1. `stabilize_delay_ms`
2. `stable_window_centi_volts`
3. `voltage_fall_centi_volts_per_sec`
4. `fuel_drop_tenths_percent_per_sec`
5. `fuel_rise_tenths_percent_per_sec`
6. `sag_multiplier_percent`

The first two parameters apply in both `CURRENT` and `VOLTAGE` modes:

- `stabilize_delay_ms`
- `stable_window_centi_volts`

The remaining four parameters are only used in `VOLTAGE` mode.

Default values:

```text
smartfuel_params = 1500,15,5,10,2,70
```

## Parameter guide

### `stabilize_delay_ms`

How long SmartFuel waits after reset, battery connection or configuration change before trying to decide that pack voltage is stable.

This applies in both `CURRENT` and `VOLTAGE` modes.

- Increase it if the reported value is noisy or jumps around immediately after plugging in.
- Decrease it if you want the value to settle sooner.

### `stable_window_centi_volts`

How tightly grouped the recent voltage samples must be before the pack is considered stable.

This applies in both `CURRENT` and `VOLTAGE` modes.

- Increase it if SmartFuel takes too long to settle on a value.
- Decrease it if SmartFuel settles too early while the pack voltage is still moving around.

### `voltage_fall_centi_volts_per_sec`

Maximum allowed downward movement of the filtered voltage in voltage mode.

- Increase it if SmartFuel reacts too slowly to real pack depletion.
- Decrease it if throttle punches or brief sag make SmartFuel fall too quickly.

### `fuel_drop_tenths_percent_per_sec`

Maximum allowed SmartFuel percentage drop rate once the model has flown.

- Increase it if the displayed percentage lags too much behind the real pack condition.
- Decrease it if the percentage drops too aggressively during load spikes.

### `fuel_rise_tenths_percent_per_sec`

Maximum allowed SmartFuel percentage rise rate once the model has flown.

- Increase it if recovery after unloading is too slow.
- Decrease it if the percentage bounces back unrealistically after hard manoeuvres.

### `sag_multiplier_percent`

Amount of sag compensation applied in voltage mode.

- Increase it if SmartFuel is too pessimistic under load.
- Decrease it if SmartFuel is too optimistic during hard collective or cyclic loading.

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

- `stabilize_delay_ms`
- `stable_window_centi_volts`

### Voltage mode

`VOLTAGE` is intended for setups without a current sensor. Start with the defaults and change one parameter at a time.

Suggested workflow:

1. Fly with the default values.
2. Check whether SmartFuel settles quickly and sensibly after plugging in.
3. Watch how it behaves during strong climb-outs, hard pitch pumps and unloaded recovery.
4. Compare the displayed remaining fuel at landing with the pack voltage and with how much reserve you wanted to keep.

Use these adjustment patterns:

- Too jumpy before flight: increase `stabilize_delay_ms` or reduce `stable_window_centi_volts`.
- Settles too slowly before flight: increase `stable_window_centi_volts` or reduce `stabilize_delay_ms`.
- Drops too hard during load: reduce `fuel_drop_tenths_percent_per_sec` or increase `sag_multiplier_percent`.
- Recovers too much after unloading: reduce `fuel_rise_tenths_percent_per_sec`.
- Feels too pessimistic overall under load: increase `sag_multiplier_percent`.
- Feels too optimistic overall: reduce `sag_multiplier_percent`.
- Tracks real depletion too slowly throughout the flight: increase `voltage_fall_centi_volts_per_sec` or increase `fuel_drop_tenths_percent_per_sec`.

## Practical notes

- Tune with the battery type and flying style you actually use.
- Large changes are rarely needed; make small adjustments and re-fly.
- Different packs may want slightly different behaviour, but the defaults are intended to be a reasonable starting point.
- Voltage mode is still an estimate. Current mode will generally be more repeatable when a well-calibrated current sensor is available.

## Telemetry

SmartFuel is exported as the `BATTERY_SMARTFUEL` telemetry sensor for supported telemetry protocols.

Supported telemetry protocols also export `BATTERY_SMARTCONSUMPTION`:

- In `CURRENT` mode it mirrors the measured consumed mAh.
- In `VOLTAGE` mode it estimates consumed mAh from the SmartFuel remaining percentage and the usable configured pack capacity.
