# SmartFuel

SmartFuel provides an intelligent remaining-charge percentage for the flight pack. It is intended to give a more useful "fuel remaining" value than the simple linear voltage-to-percent curve, especially for telemetry display on the radio.

SmartFuel always uses pack voltage as its primary input. A configured battery voltage sensor is required; if no voltage sensor is configured, SmartFuel forces itself to `OFF` at startup. In `CURRENT` mode, when both `bat_capacity` and used mAh are non-zero, the result combines the voltage-derived estimate with a consumption-based ceiling derived from the same mAh counter (see **Modes** below).

Measured consumption is reported separately by the battery monitoring code and is unaffected by which SmartFuel mode is selected.

## Modes

`smartfuel` is a three-way lookup parameter:

- `OFF` — SmartFuel does not run; `getBatteryChargeLevel()` falls through to the legacy sources.
- `VOLTAGE` — voltage-only estimator. Pack voltage drives the percentage (with stick-based sag compensation once airborne); current/consumption are ignored.
- `CURRENT` — when both `bat_capacity` and used mAh are non-zero, the level is the **lower** of the voltage-derived estimate and a consumption ceiling `initial − used / capacity`, where `initial` is the first voltage-derived fraction (0–1) after the pack is seen and `used / capacity` is the fraction of configured capacity counted by the current meter. When either capacity or used is zero, this mode behaves the same as `VOLTAGE`.

In both `VOLTAGE` and `CURRENT`, the displayed percentage never rises above the initial anchor sampled when the pack is first seen. The consumption ceiling in `CURRENT` can pull the value down faster than voltage alone when mAh used is significant.

If you plug in a pack that is not “full” (the first sample is well below 100%), treat `initial − used / capacity` as a simple fold-in of the mAh counter against that starting anchor, not as a full state-of-charge model on its own.

## How charge level is chosen

`getBatteryChargeLevel()` picks the most accurate source available, in this order:

1. **SmartFuel**, if `smartfuel != OFF`. SmartFuel always wins when enabled.
2. **Consumption-based**, if `bat_capacity` is non-zero. Reported as `100 × (capacity − used) / capacity`. Without a configured current sensor `used` stays at zero, so this path will report `100%` indefinitely — set `bat_capacity = 0` to fall through to the linear-voltage estimate instead.
3. **Linear voltage**, if cell count is known. A simple linear interpolation between `vbat_min_cell_voltage` and `vbat_max_cell_voltage`.

## Configuration

SmartFuel is enabled by setting `smartfuel` to `VOLTAGE` or `CURRENT`. Set `smartfuel = OFF` to disable it; the firmware then falls back to consumption-based or linear-voltage charge level as described above.

SmartFuel anchors its initial percent on the first voltage sample after the battery monitoring code declares the pack present (i.e. once cell-count auto-detection has completed). It does not run its own settling timer.

CLI example:

```text
set smartfuel = VOLTAGE
set smartfuel_voltage_drop_rate = 10
set smartfuel_charge_drop_rate = 50
set smartfuel_sag_gain = 40
```

## SmartFuel tuning parameters

CLI parameter names:

1. `smartfuel_voltage_drop_rate`
2. `smartfuel_charge_drop_rate`
3. `smartfuel_sag_gain`

Default values:

```text
smartfuel_voltage_drop_rate = 10
smartfuel_charge_drop_rate = 50
smartfuel_sag_gain = 40
```

## Parameter guide

### `smartfuel_voltage_drop_rate`

Maximum allowed downward slew of the filtered **per-cell** voltage used for the fuel estimate. It only slows **drops**; when the voltage recovers, the estimate is not held down by this parameter.

- Increase it (more mV/s) if SmartFuel reacts too slowly to real pack depletion.
- Decrease it if throttle punches or brief sag make the estimate fall too quickly before sag compensation catches up.

Units: **millivolts per second** (mV/s). Valid range **0**–**250** in the CLI and MSP. The value applies to **per-cell** voltage, not the full-pack total, so it does not depend on cell count.

### `smartfuel_charge_drop_rate`

Maximum allowed SmartFuel percentage drop rate on the **voltage** path once the model is armed or has ever been armed in this power cycle (`ARMED` or `WAS_EVER_ARMED`). It limits how fast the displayed percentage can fall when the voltage-based estimate would otherwise drop faster.

- Increase it if the displayed percentage lags too much behind the real pack condition.
- Decrease it if the percentage drops too aggressively during load spikes.

Units: hundredths of a percent per second.

In `CURRENT` mode with non-zero used mAh, the level is the minimum of the voltage-derived estimate and the consumption ceiling — that combination does **not** go through this drop-rate limiter (only the `VOLTAGE` path and the `CURRENT` fallback when consumption data are unavailable do).

### `smartfuel_sag_gain`

Amount of sag compensation applied to the voltage reading before it is turned into a percentage. Sag compensation is only active while the model is airborne and is driven by cyclic and collective stick load.

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

- Drops too hard during load: reduce `smartfuel_charge_drop_rate` or increase `smartfuel_sag_gain`.
- Feels too pessimistic overall under load: increase `smartfuel_sag_gain`.
- Feels too optimistic overall: reduce `smartfuel_sag_gain`.
- Tracks real depletion too slowly throughout the flight: increase `smartfuel_voltage_drop_rate` (mV/s) or increase `smartfuel_charge_drop_rate`.

## Practical notes

- Tune with the battery type and flying style you actually use.
- Large changes are rarely needed; make small adjustments and re-fly.
- Different packs may want slightly different behaviour, but the defaults are intended to be a reasonable starting point.
- SmartFuel is still an estimate. If you have a well-calibrated current sensor and a correctly configured `bat_capacity`, `smartfuel = CURRENT` takes the more pessimistic of the voltage track and a ceiling derived from used mAh. To bypass SmartFuel entirely, set `smartfuel = OFF`; `getBatteryChargeLevel()` then uses the legacy behaviour (consumption-based when `bat_capacity` is non-zero, otherwise linear voltage when cell count is known).

## Telemetry

When `smartfuel` is `VOLTAGE` or `CURRENT`, SmartFuel drives the existing fuel/charge-level telemetry output (CRSF, FrSky hub fuel, Flysky iBUS shared fuel, MSP battery state, LED strip, …) — anything routed through `getBatteryChargeLevel()`.

**FrSky D-series hub** (`ID_FUEL_LEVEL`) **and Flysky iBUS shared fuel** used to send consumed mAh when `bat_capacity` was zero; they now always send the same 0–100 charge level as the other `getBatteryChargeLevel()` paths. Set `bat_capacity` and use consumption telemetry if you need mAh on the radio.

The on-FC OSD elements are not routed through `getBatteryChargeLevel()`; they read `bat_capacity` and used mAh directly, so they continue to show the consumption-based percentage regardless of `smartfuel`. Battery **consumption** telemetry (mAh used) is unchanged regardless of `smartfuel`: it always reports the measured `currentMeter` capacity, which stays at `0` when no current sensor is configured.

## MSP (Rotorflight MSPv2)

When the firmware is built with `USE_SMARTFUEL`:

- **`MSP2_GET_SMARTFUEL_CONFIG` (`0x4000`)** — response: U8 mode (`0` = OFF, `1` = VOLTAGE, `2` = CURRENT); U8 `smartfuel_voltage_drop_rate` (mV/s, 0–250); U8 `smartfuel_charge_drop_rate`; U8 `smartfuel_sag_gain`.
- **`MSP2_SET_SMARTFUEL_CONFIG` (`0x4001`)** — payload: same four fields in the same order. Values use the same limits as the CLI parameters.
