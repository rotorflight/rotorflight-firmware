#!/usr/bin/env python3

import math
import sys
import matplotlib.pyplot as plt
import numpy as np

from tail_kinematics import (
    heli,
)


class config():
    # Plotting
    plot_min_deg         = -50.0
    plot_max_deg         = 50.0


## Plotting stuff

def plot_all(h: heli):
    angle_degrees = np.arange(config.plot_min_deg, config.plot_max_deg + 1.0, 1.0)
    slider_positions = []
    pivot_angles_deg = []
    rod_deflections = []
    servo_angles_deg = []

    for angle_deg in angle_degrees:
        alpha = math.radians(angle_deg)
        slider = h.blade_angle_to_slider(alpha)
        pivot = h.slider_to_pivot_angle(slider)
        rod_deflection = h.pivot_angle_to_rod_deflection(pivot)
        servo_angle = h.rod_deflection_to_servo_angle(rod_deflection)
        slider_positions.append(slider)
        pivot_angles_deg.append(math.degrees(pivot))
        rod_deflections.append(rod_deflection)
        servo_angles_deg.append(math.degrees(servo_angle))

    blade_angles_deg = angle_degrees
    blade_angles_rad = np.radians(blade_angles_deg)
    servo_real_rad = np.array([h.blade_angle_to_servo_angle(alpha) for alpha in blade_angles_rad])
    servo_real_deg = np.degrees(servo_real_rad)
    coeff_scale = 10000

    fig, axes = plt.subplots(2, 2, figsize=(16, 12), dpi=200, sharex=True)
    ax_sweep = axes[0, 0]
    ax_curve = axes[0, 1]
    ax_error = axes[1, 0]
    ax_int_error = axes[1, 1]

    ax_sweep.plot(angle_degrees, slider_positions, label="Slider distance (mm)")
    ax_sweep.plot(angle_degrees, pivot_angles_deg, label="Pivot angle (deg)")
    ax_sweep.plot(angle_degrees, rod_deflections, label="Rod deflection (mm)")
    ax_sweep.plot(angle_degrees, servo_angles_deg, label="Servo angle (deg)")
    ax_sweep.set_ylabel("Output value")
    ax_sweep.set_title("Tail sweep: blade angle to slider and pivot")
    ax_sweep.legend()
    ax_sweep.grid(True)

    ax_curve.plot(blade_angles_deg, servo_real_deg, label="Real curve")

    for degree in range(4, 10):
        h.print_poly_coeffs_int32(degree, coeff_scale)
        coeffs = h.polynomial_approx_blade_to_servo(degree=degree)
        servo_approx_rad = np.array([h.evaluate_polynomial(coeffs, angle_rad) for angle_rad in blade_angles_rad])
        servo_approx_deg = np.degrees(servo_approx_rad)
        servo_error_deg = servo_approx_deg - servo_real_deg
        _, coeffs_quantized = h.quantize_coeffs(coeffs, scale=coeff_scale)
        servo_approx_int_rad = np.array([h.evaluate_polynomial(coeffs_quantized, angle_rad) for angle_rad in blade_angles_rad])
        servo_approx_int_deg = np.degrees(servo_approx_int_rad)
        servo_error_int_deg = servo_approx_int_deg - servo_real_deg
        ax_curve.plot(blade_angles_deg, servo_approx_deg, "--", label=f"Polynomial approx (deg={degree})")
        ax_error.plot(blade_angles_deg, servo_error_deg, label=f"Error (deg={degree})")
        ax_int_error.plot(blade_angles_deg, servo_error_int_deg, label=f"Int coeff error (deg={degree})")

    ax_curve.set_ylabel("Servo angle (deg)")
    ax_curve.set_title("Blade angle to servo angle: real vs polynomial approx sweep (deg 2..6)")
    ax_curve.legend()
    ax_curve.grid(True)

    ax_error.set_xlabel("Blade angle (deg)")
    ax_error.set_ylabel("Float coeff error (deg)")
    ax_error.legend()
    ax_error.grid(True)

    ax_int_error.set_xlabel("Blade angle (deg)")
    ax_int_error.set_ylabel("Int coeff error (deg)")
    ax_int_error.legend()
    ax_int_error.grid(True)

    fig.tight_layout()
    plt.show()


def main():
    RF = heli()
    RF.load(sys.argv[1])
    RF.set(sys.argv[2:])
    plot_all(RF)

if __name__ == "__main__":
    main()
