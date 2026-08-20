#!/usr/bin/env python3

import math
import unittest

from ackermann_shadow_plant import AckermannPlant
from stm32_ackermann_calibration import (
    angle_to_pwm,
    center_yaw_rate_from_right_wheel_angle,
    pwm_to_angle,
    twist_to_right_wheel_angle,
)


def make_plant(*, speed_gain=1.0):
    return AckermannPlant(
        wheelbase_m=0.144,
        min_turning_radius_m=0.35,
        speed_limit_mps=0.50,
        tau_speed_sec=0.10,
        tau_steering_sec=0.24,
        max_accel_mps2=0.53129,
        max_decel_mps2=0.76369,
        legacy_max_yaw_accel_radps2=2.0,
        speed_gain=speed_gain)


class AckermannShadowPlantTest(unittest.TestCase):
    def test_yaw_rate_is_kinematically_derived(self):
        plant = make_plant()
        for _ in range(100):
            state = plant.step(0.50, 1.20, 0.02)
        expected = center_yaw_rate_from_right_wheel_angle(
            state.speed_mps, state.steering_rad)
        self.assertAlmostEqual(state.yaw_rate_radps, expected, places=12)
        self.assertLessEqual(
            abs(state.yaw_rate_radps),
            abs(state.speed_mps) / plant.min_turning_radius_m + 1.0e-12)

    def test_stopped_vehicle_cannot_keep_rotating(self):
        plant = make_plant()
        for _ in range(100):
            plant.step(0.50, 1.20, 0.02)
        for _ in range(300):
            state = plant.step(0.0, 0.0, 0.02)
        self.assertLess(abs(state.speed_mps), 1.0e-9)
        self.assertEqual(state.yaw_rate_radps, 0.0)

    def test_low_speed_respects_minimum_radius(self):
        plant = make_plant()
        for _ in range(100):
            state = plant.step(0.02, 10.0, 0.02)
        self.assertLessEqual(
            abs(state.yaw_rate_radps),
            abs(state.speed_mps) / 0.35 + 1.0e-12)

    def test_stm32_twist_conversion_recovers_center_yaw_rate(self):
        for requested_w in (-1.2, -0.4, 0.4, 1.2):
            angle = twist_to_right_wheel_angle(0.5, requested_w)
            expected = math.copysign(min(abs(requested_w), 0.5 / 0.35), requested_w)
            self.assertAlmostEqual(
                center_yaw_rate_from_right_wheel_angle(0.5, angle),
                expected, places=10)

    def test_asymmetric_servo_table_round_trip(self):
        self.assertEqual(angle_to_pwm(0.0), 1500)
        self.assertLess(angle_to_pwm(0.30), 1500)
        self.assertGreater(angle_to_pwm(-0.30), 1500)
        for angle in (-0.45, -0.20, 0.10, 0.30, 0.60):
            recovered = pwm_to_angle(angle_to_pwm(angle))
            self.assertAlmostEqual(recovered, angle, delta=0.003)

    def test_motor_gain_does_not_change_exact_stm32_steering_target(self):
        nominal = make_plant(speed_gain=1.0).step(0.40, 0.80, 0.02)
        identified = make_plant(speed_gain=1.05268).step(0.40, 0.80, 0.02)
        expected = twist_to_right_wheel_angle(0.40, 0.80)
        self.assertAlmostEqual(nominal.steering_target_rad, expected, places=12)
        self.assertAlmostEqual(identified.steering_target_rad, expected, places=12)
        self.assertEqual(nominal.steering_target_pwm,
                         identified.steering_target_pwm)


if __name__ == '__main__':
    unittest.main()
