# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import pytest

from rover_controller.calibration_analysis import (
    calibrate_separation, radius_multiplier, separation_multiplier, SpinSegment)

SEPARATION = 0.62602


def spin(yaw_rate, true_multiplier):
    """Rim speeds a rover with this effective track needs to turn at yaw_rate."""
    half = yaw_rate * SEPARATION * true_multiplier / 2
    return SpinSegment(yaw_rate, -half, half, yaw_rate)


def test_recovers_effective_track():
    seg = spin(0.6, 1.7)
    assert separation_multiplier(seg.left_rim_speed, seg.right_rim_speed,
                                 seg.imu_yaw_rate, SEPARATION) == pytest.approx(1.7)


def test_median_over_both_directions_and_rates():
    segments = [spin(w, m) for w, m in [(0.3, 1.9), (0.6, 1.7), (1.0, 1.6),
                                         (-0.3, 1.9), (-0.6, 1.7), (-1.0, 1.6)]]
    result = calibrate_separation(segments, SEPARATION)
    assert result.multiplier == pytest.approx(1.7)
    assert result.rejected == 0


def test_rejects_stalled_and_wrong_sign_segments():
    good = spin(0.6, 1.5)
    stalled = SpinSegment(0.6, -0.2, 0.2, 0.01)
    flipped = SpinSegment(0.6, good.left_rim_speed, good.right_rim_speed, -0.6)
    result = calibrate_separation([good, stalled, flipped], SEPARATION)
    assert result.multiplier == pytest.approx(1.5)
    assert result.rejected == 2


def test_all_rejected_raises():
    with pytest.raises(ValueError):
        calibrate_separation([SpinSegment(0.6, -0.2, 0.2, -0.6)], SEPARATION)


def test_radius_multiplier():
    assert radius_multiplier(4.9, 5.0) == pytest.approx(0.98)
    with pytest.raises(ValueError):
        radius_multiplier(0.0, 5.0)
