#!/usr/bin/env python3

import math
import sys
import numpy as np
import yaml


class heli():
    def __init__(self):
        self.approx_points        = 100
        self.blade_angle_min      = None
        self.blade_angle_max      = None
        self.blade_grip_arm       = None
        self.blade_grip_link      = None
        self.blade_grip_offset    = None
        self.slider_arm           = None
        self.slider_offset        = None
        self.pivot_arm            = None
        self.pivot_dist           = None
        self.pivot_angle          = None
        self.pivot_zero           = None
        self.pivot_offset         = None
        self.pivot_theta          = None
        self.pivot_beta           = None
        self.servo_arm            = None
        self.servo_zero           = None
        self.servo_offset         = None
        self.servo_theta          = None

    def load(self, path: str):
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        for key, value in data.items():
            if not hasattr(self, key):
                raise ValueError(f"Unknown heli parameter: {key!r}")
            setattr(self, key, value)
        self.update()

    def set(self, args):
        for arg in args:
            key, sep, value = arg.partition('=')
            if not sep:
                raise ValueError(f"Expected name=value, got {arg!r}")
            if not hasattr(self, key):
                raise ValueError(f"Unknown heli parameter: {key!r}")
            setattr(self, key, yaml.safe_load(value))
        self.update()

    def update(self):
        ry = self.blade_grip_arm - self.slider_arm
        rr = self.blade_grip_link
        self.blade_grip_offset = math.sqrt(rr*rr - ry*ry)
        self.servo_theta = math.radians(self.servo_zero)
        self.servo_offset = math.sin(self.servo_theta) * self.servo_arm
        self.pivot_theta = math.radians(self.pivot_zero)
        self.pivot_offset = math.sin(self.pivot_theta) * self.pivot_arm
        self.pivot_beta = math.radians(self.pivot_angle - 90.0)
        self.slider_offset = math.sin(self.pivot_beta + self.pivot_theta) * self.pivot_dist
        self.unit_test()


    def rod_deflection_to_servo_angle(self, deflection: float) -> float:
        theta = math.asin((deflection + self.servo_offset) / self.servo_arm)
        return theta - self.servo_theta

    def servo_angle_to_rod_deflection(self, angle: float) -> float:
        deflection = math.sin(angle + self.servo_theta) * self.servo_arm
        return deflection - self.servo_offset


    def pivot_angle_to_rod_deflection(self, angle: float) -> float:
        rx = math.sin(angle) * self.pivot_arm
        return rx - self.pivot_offset

    def rod_deflection_to_pivot_angle(self, deflection: float) -> float:
        theta = math.asin((deflection + self.pivot_offset) / self.pivot_arm)
        return theta


    def slider_to_pivot_angle(self, slider: float) -> float:
        theta = math.atan2(slider + self.slider_offset, self.pivot_dist)
        return theta - self.pivot_beta

    def pivot_angle_to_slider(self, angle: float) -> float:
        slider = math.tan(angle + self.pivot_beta) * self.pivot_dist
        return slider - self.slider_offset


    def blade_angle_to_slider(self, alpha: float) -> float:
        ax = math.sin(alpha) * self.blade_grip_arm
        ay = math.cos(alpha) * self.blade_grip_arm
        ly = ay - self.slider_arm
        lr = self.blade_grip_link
        lx = math.sqrt(lr**2 - ly**2)
        d = lx + ax - self.blade_grip_offset
        return d

    def slider_to_blade_angle(self, slider: float) -> float:
        R = self.blade_grip_arm
        S = self.slider_arm
        L = self.blade_grip_link
        X = slider + self.blade_grip_offset
        A = 2 * R * S
        B = 2 * R * X
        C = math.sqrt(A*A + B*B)
        K = X*X + R*R + S*S - L*L
        return math.atan2(B, A) - math.acos(K / C)


    def blade_angle_to_servo_angle(self, alpha: float) -> float:
        slider = self.blade_angle_to_slider(alpha)
        pivot = self.slider_to_pivot_angle(slider)
        rod_deflection = self.pivot_angle_to_rod_deflection(pivot)
        servo_angle = self.rod_deflection_to_servo_angle(rod_deflection)
        return servo_angle

    def servo_angle_to_blade_angle(self, servo_angle: float) -> float:
        rod_deflection = self.servo_angle_to_rod_deflection(servo_angle)
        pivot = self.rod_deflection_to_pivot_angle(rod_deflection)
        slider = self.pivot_angle_to_slider(pivot)
        blade_angle = self.slider_to_blade_angle(slider)
        return blade_angle


    def blade_angle_to_error(self, alpha: float) -> float:
        servo_angle = self.blade_angle_to_servo_angle(alpha)
        blade_angle = self.servo_angle_to_blade_angle(servo_angle)
        return abs(alpha - blade_angle)


    def unit_test(self):
        angles_deg = np.linspace(self.blade_angle_min, self.blade_angle_max, self.approx_points)
        max_error = max(self.blade_angle_to_error(math.radians(deg)) for deg in angles_deg)
        if max_error > 1e-9:
            sys.exit(f"unit_test failed: max error {max_error:.3e} rad exceeds tolerance")


    ## Approximation stuff

    def polynomial_approx_blade_to_servo(self, degree: int):
        blade_angles_deg = np.linspace(self.blade_angle_min, self.blade_angle_max, self.approx_points)
        blade_angles_rad = np.radians(blade_angles_deg)
        servo_angles_rad = np.array([self.blade_angle_to_servo_angle(alpha) for alpha in blade_angles_rad])
        coeffs = np.polyfit(blade_angles_rad, servo_angles_rad, degree)
        return coeffs

    @staticmethod
    def evaluate_polynomial(coeffs, blade_angle_rad: float) -> float:
        return float(np.polyval(coeffs, blade_angle_rad))

    @staticmethod
    def quantize_coeffs(coeffs, scale: int = 16384):
        coeffs_int32 = np.round(np.array(coeffs) * scale).astype(np.int32)
        coeffs_quantized = coeffs_int32.astype(np.float64) / scale
        return coeffs_int32, coeffs_quantized

    def print_poly_coeffs_int32(self, degree: int, scale: int = 16384):
        coeffs = self.polynomial_approx_blade_to_servo(degree)
        coeffs_int32, _ = self.quantize_coeffs(coeffs, scale=scale)
        coeffs_str = ", ".join([str(int(c)) for c in coeffs_int32[::-1]])
        print(f"// degree={degree}, coefficients in radians, scale={scale}")
        print(coeffs_str)


if __name__ == "__main__":
    RF = heli()
    RF.load(sys.argv[1])
    RF.set(sys.argv[2:])
    RF.print_poly_coeffs_int32(7, 10000)
