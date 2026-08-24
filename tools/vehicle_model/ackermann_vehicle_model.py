#!/usr/bin/env python3
"""Exact OrigincarPro STM32 Ackermann command and servo calibration.

The lower controller stores the front-right wheel angle.  A Twist command is
first converted to that angle using ``Vz_to_Akm_Angle`` and then to PWM using
the asymmetric piecewise cubic table in ``BALANCE/servo_fit.c``.  This module
keeps the command-side calibration separate from effective steering inferred
from IMU/odometry; the vehicle has no physical steering-angle sensor.
"""

from __future__ import annotations

import math
from dataclasses import dataclass


WHEELBASE_M = 0.144
TRACK_WIDTH_M = 0.162
MIN_TURNING_RADIUS_M = 0.35
STEERING_RIGHT_MIN_RAD = -0.513127
STEERING_LEFT_MAX_RAD = 0.733038
SERVO_PWM_MID = 1500


@dataclass(frozen=True)
class ServoSegment:
    x0: float
    x1: float
    c0: float
    c1: float
    c2: float
    c3: float


LEFT_ANGLE_TO_PWM = (
    ServoSegment(0.000000, 0.111701, 787.525041, -817.742896, -813.729849, 1500.0),
    ServoSegment(0.111701, 0.207258, 24020.106696, -3127.888935, -966.937209, 1400.0),
    ServoSegment(0.207258, 0.333707, -9667.489661, 2138.998758, -906.729365, 1300.0),
    ServoSegment(0.333707, 0.448550, 5424.303418, -982.082684, -829.511792, 1200.0),
    ServoSegment(0.448550, 0.571770, -3153.056509, 623.108917, -840.461006, 1100.0),
    ServoSegment(0.571770, 0.689405, -33540.580791, 3779.248031, -830.522474, 1000.0),
    ServoSegment(0.689405, 0.733038, 298322.361850, -34973.572767, -1333.784866, 900.0),
)

RIGHT_ANGLE_TO_PWM = (
    ServoSegment(-0.513127, -0.461058, 567.754811, -865.037328, -1877.026588, 2200.0),
    ServoSegment(-0.461058, -0.411200, -14862.361483, -125.336977, -1962.491971, 2100.0),
    ServoSegment(-0.411200, -0.365123, 290465.961739, -15216.915444, -2085.826669, 2000.0),
    ServoSegment(-0.365123, -0.286234, -44846.729763, 8234.131928, -1638.085568, 1900.0),
    ServoSegment(-0.286234, -0.194779, -13187.127664, 2111.356212, -1176.227553, 1800.0),
    ServoSegment(-0.194779, -0.107774, 19430.153951, -2017.294767, -1120.932231, 1700.0),
    ServoSegment(-0.107774, 0.000000, 1697.073811, 771.370202, -1030.712531, 1600.0),
)

LEFT_PWM_TO_ANGLE = (
    ServoSegment(800.0, 900.0, 0.000000016979, -0.000005397978, -0.000066407, 0.733038),
    ServoSegment(900.0, 1000.0, 0.000000051252, -0.000010523180, -0.000637092, 0.689405),
    ServoSegment(1000.0, 1100.0, 0.000000007193, -0.000001005076, -0.001204190, 0.571770),
    ServoSegment(1100.0, 1200.0, -0.000000009565, 0.000001360685, -0.001188629, 0.448550),
    ServoSegment(1200.0, 1300.0, 0.000000023678, -0.000002976024, -0.001203663, 0.333707),
    ServoSegment(1300.0, 1400.0, -0.000000020740, 0.000003403700, -0.001089496, 0.207258),
    ServoSegment(1400.0, 1500.0, 0.000000000629, -0.000000932970, -0.001030091, 0.111701),
)

RIGHT_PWM_TO_ANGLE = (
    ServoSegment(1500.0, 1600.0, 0.000000001107, 0.000000927738, -0.001181923, 0.000000),
    ServoSegment(1600.0, 1700.0, -0.000000011447, 0.000002072456, -0.000962698, -0.107774),
    ServoSegment(1700.0, 1800.0, 0.000000009027, -0.000001130828, -0.000891535, -0.194779),
    ServoSegment(1800.0, 1900.0, 0.000000014894, -0.000000907450, -0.000847064, -0.286234),
    ServoSegment(1900.0, 2000.0, -0.000000013915, 0.000002601294, -0.000582108, -0.365123),
    ServoSegment(2000.0, 2100.0, 0.000000000884, -0.000000284921, -0.000478742, -0.411200),
    ServoSegment(2100.0, 2200.0, 0.000000000024, -0.000000115333, -0.000509267, -0.461058),
)


def _eval(segments: tuple[ServoSegment, ...], value: float) -> float:
    segment = segments[0]
    if value >= segments[-1].x1:
        segment = segments[-1]
        value = segment.x1
    elif value <= segment.x0:
        value = segment.x0
    else:
        for candidate in segments:
            if candidate.x0 <= value <= candidate.x1:
                segment = candidate
                break
    t = value - segment.x0
    return ((segment.c0 * t + segment.c1) * t + segment.c2) * t + segment.c3


def angle_to_pwm(right_wheel_angle_rad: float) -> int:
    angle = max(STEERING_RIGHT_MIN_RAD,
                min(STEERING_LEFT_MAX_RAD, right_wheel_angle_rad))
    if angle > 0.0:
        return int(_eval(LEFT_ANGLE_TO_PWM, angle) + 0.5)
    if angle < 0.0:
        return int(_eval(RIGHT_ANGLE_TO_PWM, angle) + 0.5)
    return SERVO_PWM_MID


def pwm_to_angle(pwm: int) -> float:
    pwm = max(800, min(2200, int(pwm)))
    if pwm < SERVO_PWM_MID:
        return _eval(LEFT_PWM_TO_ANGLE, float(pwm))
    if pwm > SERVO_PWM_MID:
        return _eval(RIGHT_PWM_TO_ANGLE, float(pwm))
    return 0.0


def twist_to_right_wheel_angle(
        linear_mps: float, angular_radps: float,
        *, minimum_radius_m: float = MIN_TURNING_RADIUS_M) -> float:
    """Port of the STM32 ``Vz_to_Akm_Angle`` conversion."""
    if abs(linear_mps) <= 1.0e-9 or abs(angular_radps) <= 1.0e-9:
        return 0.0
    yaw_rate = angular_radps
    if abs(linear_mps / yaw_rate) <= minimum_radius_m:
        yaw_rate = math.copysign(abs(linear_mps) / minimum_radius_m, yaw_rate)
    center_radius = linear_mps / yaw_rate
    angle = math.atan(WHEELBASE_M / (center_radius + 0.5 * TRACK_WIDTH_M))
    return max(STEERING_RIGHT_MIN_RAD, min(STEERING_LEFT_MAX_RAD, angle))


def center_yaw_rate_from_right_wheel_angle(
        linear_mps: float, right_wheel_angle_rad: float) -> float:
    """Physical center yaw rate implied by the front-right wheel angle."""
    if abs(linear_mps) <= 1.0e-9 or abs(right_wheel_angle_rad) <= 1.0e-9:
        return 0.0
    center_radius = (
        WHEELBASE_M / math.tan(right_wheel_angle_rad) -
        0.5 * TRACK_WIDTH_M)
    if abs(center_radius) <= 1.0e-9:
        return 0.0
    return linear_mps / center_radius


def effective_right_wheel_angle(linear_mps: float, yaw_rate_radps: float) -> float:
    """Infer an equivalent angle from speed and independently measured yaw.

    This is an effective vehicle response, not steering-sensor feedback.
    """
    if abs(linear_mps) <= 1.0e-4 or abs(yaw_rate_radps) <= 1.0e-5:
        return 0.0
    center_radius = linear_mps / yaw_rate_radps
    return math.atan(WHEELBASE_M / (center_radius + 0.5 * TRACK_WIDTH_M))
