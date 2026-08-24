#!/usr/bin/env python3
"""Small Ackermann plant used by the native Nav2 virtual-vehicle harness.

The steering angle is the lateral actuator state.  Yaw rate is always derived
from longitudinal speed and steering angle, so a stopped virtual vehicle can
never continue rotating as it could in the legacy independent-v/w model.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from ackermann_vehicle_model import (
    STEERING_LEFT_MAX_RAD,
    STEERING_RIGHT_MIN_RAD,
    angle_to_pwm,
    center_yaw_rate_from_right_wheel_angle,
    twist_to_right_wheel_angle,
)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


@dataclass(frozen=True)
class AckermannPlantState:
    speed_mps: float
    steering_rad: float
    yaw_rate_radps: float
    steering_target_rad: float
    steering_target_pwm: int


class AckermannPlant:
    def __init__(self, *, wheelbase_m: float, min_turning_radius_m: float,
                 speed_limit_mps: float, tau_speed_sec: float,
                 tau_steering_sec: float, max_accel_mps2: float,
                 max_decel_mps2: float, legacy_max_yaw_accel_radps2: float,
                 speed_gain: float = 1.0,
                 tau_steering_left_sec: float | None = None,
                 tau_steering_right_sec: float | None = None,
                 steering_gain_left: float = 1.0,
                 steering_gain_right: float = 1.0):
        if wheelbase_m <= 0.0 or min_turning_radius_m <= 0.0:
            raise ValueError('wheelbase and minimum turning radius must be positive')
        self.wheelbase_m = wheelbase_m
        self.min_turning_radius_m = min_turning_radius_m
        self.speed_limit_mps = speed_limit_mps
        self.tau_speed_sec = tau_speed_sec
        self.tau_steering_sec = tau_steering_sec
        self.max_accel_mps2 = max_accel_mps2
        self.max_decel_mps2 = max_decel_mps2
        self.legacy_max_yaw_accel_radps2 = legacy_max_yaw_accel_radps2
        self.speed_gain = speed_gain
        self.tau_steering_left_sec = (
            tau_steering_sec if tau_steering_left_sec is None
            else tau_steering_left_sec)
        self.tau_steering_right_sec = (
            tau_steering_sec if tau_steering_right_sec is None
            else tau_steering_right_sec)
        self.steering_gain_left = steering_gain_left
        self.steering_gain_right = steering_gain_right
        self.speed_mps = 0.0
        self.steering_rad = 0.0
        self.yaw_rate_radps = 0.0
        self.steering_target_rad = 0.0
        self.steering_target_pwm = 1500

    def step(self, command_v_mps: float, command_w_radps: float,
             dt_sec: float) -> AckermannPlantState:
        dt = max(1.0e-6, dt_sec)
        command_v = _clamp(
            command_v_mps, -self.speed_limit_mps, self.speed_limit_mps)
        target_v = _clamp(
            self.speed_gain * command_v,
            -self.speed_limit_mps, self.speed_limit_mps)

        # The lower-level Ackermann conversion centers steering whenever either
        # commanded v or w is zero. Otherwise Twist specifies curvature w/v.
        # Use the exact incoming command here: speed_gain belongs to the motor
        # response model and must not change the STM32 steering/PWM target.
        command_delta = twist_to_right_wheel_angle(
            command_v, command_w_radps,
            minimum_radius_m=self.min_turning_radius_m)
        self.steering_target_rad = command_delta
        self.steering_target_pwm = angle_to_pwm(command_delta)
        target_delta = command_delta * (
            self.steering_gain_left if command_delta >= 0.0
            else self.steering_gain_right)

        # Match the identified discrete-time model: first-order motor response
        # with an outer measured acceleration/braking envelope.  Applying the
        # envelope before the first-order step would attenuate it by dt/tau a
        # second time and make the shadow vehicle much slower than R3.
        speed_alpha = min(1.0, dt / max(dt, self.tau_speed_sec))
        requested_speed_step = speed_alpha * (target_v - self.speed_mps)
        accel_limit = (
            self.max_accel_mps2
            if abs(target_v) >= abs(self.speed_mps)
            else self.max_decel_mps2)
        self.speed_mps += _clamp(
            requested_speed_step, -accel_limit * dt, accel_limit * dt)
        if target_v == 0.0 and abs(self.speed_mps) < 1.0e-9:
            self.speed_mps = 0.0

        # The identified steering tau already includes servo/linkage/vehicle
        # build-up.  Do not apply the legacy independent-yaw acceleration cap
        # here: that would double-limit the response and invalidate the fit.
        # Keep the constructor argument only for old runner compatibility.
        steering_tau = (
            self.tau_steering_left_sec if target_delta >= 0.0
            else self.tau_steering_right_sec)
        steering_alpha = min(1.0, dt / max(dt, steering_tau))
        self.steering_rad += steering_alpha * (
            target_delta - self.steering_rad)
        self.steering_rad = _clamp(
            self.steering_rad, STEERING_RIGHT_MIN_RAD,
            STEERING_LEFT_MAX_RAD)
        if target_delta == 0.0 and abs(self.steering_rad) < 1.0e-9:
            self.steering_rad = 0.0

        self.yaw_rate_radps = center_yaw_rate_from_right_wheel_angle(
            self.speed_mps, self.steering_rad)
        return AckermannPlantState(
            self.speed_mps, self.steering_rad, self.yaw_rate_radps,
            self.steering_target_rad, self.steering_target_pwm)
