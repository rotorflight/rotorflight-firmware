# Tail link kinematics

Offline Python tools for modelling a **variable-pitch tail rotor** linkage and generating
`tail_link_curve` polynomial coefficients for the Rotorflight mixer. This directory is
not part of the firmware build.

Rotorflight applies the curve in `mixerTailServoDeflection()` when `tail_rotor_mode` is
`VARIABLE` and at least one coefficient is non-zero. See `Changes.md` for CLI details.

## Files

| File | Purpose |
|------|---------|
| `tail_kinematics.py` | Linkage model, round-trip self-test, coefficient export |
| `tail_curves.py` | Plots real vs polynomial fit and approximation error |
| `*.yaml` | Example geometry for specific helicopters |

## Dependencies

```bash
pip install numpy pyyaml matplotlib
```

`matplotlib` is only required for `tail_curves.py`.

## Kinematics chain

The model maps **tail blade pitch angle** to **tail servo angle** through four stages:

1. **Blade grip** — blade angle to slider linear travel
2. **Slider / pivot** — slider position to pivot angle
3. **Pushrod** — pivot angle to rod deflection
4. **Servo horn** — rod deflection to servo angle

## YAML parameters

All lengths are in **millimetres**, angles in **degrees** (unless noted).

| Parameter | Description |
|-----------|-------------|
| `blade_angle_min`, `blade_angle_max` | Blade pitch range used for fitting and self-test |
| `blade_grip_arm` | Blade grip arm length |
| `blade_grip_link` | Link length between grip and slider |
| `slider_arm` | Slider arm length |
| `pivot_arm` | Pivot lever arm |
| `pivot_dist` | Pivot-to-shaft distance |
| `pivot_angle` | Pivot geometry angle (typically 90) |
| `pivot_zero` | Pivot zero position |
| `servo_arm` | Servo horn radius |
| `servo_zero` | Servo zero angle |

Start from a nearby example (`Trex450.yaml`, `Trex500.yaml`, `R42.yaml`) and adjust
dimensions to match your airframe. Measure linkage geometry carefully; small errors in arm
lengths materially affect the fitted curve.

## Usage

Run from this directory:

```bash
# Print quantised 7th-order coefficients (scale 10000) for copy into CLI
./tail_kinematics.py Trex500.yaml

# Plot linkage sweep and polynomial fit quality
./tail_curves.py Trex500.yaml
```

`tail_kinematics.py` fits blade angle (rad) → servo angle (rad) with `numpy.polyfit`,
quantises coefficients to `int32` at the chosen scale (default **10000**, matching firmware),
and prints them as a comma-separated list with a `//` comment line. Coefficients are ordered
`c0, c1, …, c7` for the Horner evaluation in `src/main/flight/mixer.c`.

Example output:

```
// degree=7, coefficients in radians, scale=10000
123, 456, ...
```

## Loading coefficients in Rotorflight

Set the eight `tail_link_curve` values (CLI array, same order as printed).
With all coefficients zero the curve is disabled and yaw passes through
linearly (aside from `tail_center_trim`).

When the coefficients are set correctly, the Yaw Calibration in mixer should
be set to 100.0%. If this does not give the correct blade angles, then the curve/measurements
are incorrect.

## Firmware mapping

The mixer evaluates the polynomial on stabilised yaw (after scaling to radians), then
scales the result back to servo deflection units. Motorized and bidirectional tail modes do
not use this curve.
