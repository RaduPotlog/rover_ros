# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Pure skid-steer odometry calibration maths. No ROS imports - unit-tested directly."""

from dataclasses import dataclass
from statistics import median
from typing import List, Sequence


@dataclass(frozen=True)
class SpinSegment:
    commanded_yaw_rate: float   # rad/s sent to the drive controller
    left_rim_speed: float       # m/s, mean measured wheel speed * radius, left side
    right_rim_speed: float      # m/s, same for the right side
    imu_yaw_rate: float         # rad/s, gyro z (the ground truth for rotation)


def separation_multiplier(left_rim_speed: float, right_rim_speed: float,
                          imu_yaw_rate: float, wheel_separation: float) -> float:
    """Effective-track / physical-track ratio that makes wheel odometry match the gyro.

    diff_drive: yaw_rate = (v_right - v_left) / (wheel_separation * multiplier).
    Skid-steer wheels slip sideways while turning, so the track that explains the measured
    rotation is wider than the physical one - the multiplier is that ratio.
    """
    if wheel_separation <= 0:
        raise ValueError('wheel_separation must be > 0')
    if abs(imu_yaw_rate) < 1e-6:
        raise ValueError('imu_yaw_rate is ~0; the rover did not turn')
    return (right_rim_speed - left_rim_speed) / (wheel_separation * imu_yaw_rate)


@dataclass(frozen=True)
class SeparationResult:
    multiplier: float            # median over accepted segments
    per_segment: List[float]     # same order as the accepted segments
    rejected: int                # segments that barely turned or gave a non-physical value


def calibrate_separation(segments: Sequence[SpinSegment], wheel_separation: float,
                         min_yaw_rate: float = 0.05) -> SeparationResult:
    values = []
    rejected = 0
    for seg in segments:
        if abs(seg.imu_yaw_rate) < min_yaw_rate:
            rejected += 1
            continue
        value = separation_multiplier(seg.left_rim_speed, seg.right_rim_speed,
                                      seg.imu_yaw_rate, wheel_separation)
        # <= 0 means the gyro and wheels disagree on the turn direction (IMU axis or wheel
        # direction wiring), which no multiplier can fix.
        if value <= 0:
            rejected += 1
            continue
        values.append(value)
    if not values:
        raise ValueError('no usable spin segment; check the IMU sign and that the rover turned')
    return SeparationResult(multiplier=median(values), per_segment=values, rejected=rejected)


def radius_multiplier(measured_distance: float, odometry_distance: float) -> float:
    """Scale for wheel_radius so wheel odometry reports the distance actually driven."""
    if measured_distance <= 0 or odometry_distance <= 0:
        raise ValueError('distances must be > 0')
    return measured_distance / odometry_distance
