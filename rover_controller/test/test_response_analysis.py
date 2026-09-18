# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

import math

import pytest

from rover_controller.response_analysis import (
    analyze_step, derive_acceleration_limits, max_acceleration, Sample)


def first_order(target, tau, delay, step_time=1.0, end=4.0, dt=0.01, initial=0.0):
    samples = []
    t = 0.0
    while t < end:
        if t < step_time + delay:
            fb = initial
        else:
            fb = target + (initial - target) * math.exp(-(t - step_time - delay) / tau)
        samples.append(Sample(t, initial if t < step_time else target, fb))
        t += dt
    return samples


def test_first_order_step():
    m = analyze_step(first_order(5.0, tau=0.2, delay=0.05), step_time=1.0)
    assert m.initial == pytest.approx(0.0)
    assert m.target == pytest.approx(5.0)
    assert m.dead_time == pytest.approx(0.05 + 0.2 * -math.log(0.95), abs=0.015)
    assert m.rise_time == pytest.approx(0.2 * math.log(9), abs=0.02)   # 10->90 % of 1st order
    assert m.max_acceleration == pytest.approx(5.0 / 0.2, rel=0.15)    # initial slope A/tau
    assert m.overshoot == pytest.approx(0.0)
    assert m.steady_state_error == pytest.approx(0.0, abs=1e-3)


def test_negative_step_reports_signed_error_in_step_direction():
    samples = [Sample(s.t, s.reference, s.feedback * 0.9)
               for s in first_order(-4.0, tau=0.1, delay=0.0)]
    m = analyze_step(samples, step_time=1.0)
    assert m.steady_state_error == pytest.approx(-0.1, abs=1e-3)  # 10 % too slow


def test_overshoot():
    samples = [Sample(0.0, 0, 0), Sample(0.8, 0, 0), Sample(1.0, 2, 0), Sample(1.2, 2, 2.4),
               Sample(1.4, 2, 2.0), Sample(2.0, 2, 2.0)]
    assert analyze_step(samples, 1.0, settle_window=0.3).overshoot == pytest.approx(0.2)


def test_never_reached_gives_none():
    samples = [Sample(t / 100, 0 if t < 100 else 5, 0.0) for t in range(300)]
    m = analyze_step(samples, step_time=1.0)
    assert m.dead_time is None and m.rise_time is None


def test_record_without_pre_step_samples_is_rejected():
    with pytest.raises(ValueError):
        analyze_step([Sample(1.0, 1, 0)], step_time=1.0)


def test_max_acceleration_ignores_single_sample_noise():
    samples = [Sample(t / 100, 0, 1.0 * t / 100) for t in range(100)]
    samples[50] = Sample(0.5, 0, 5.0)  # one noisy spike
    assert max_acceleration(samples, span=0.1) < 50


def test_acceleration_limits():
    limits = derive_acceleration_limits(10.0, 0.1651, 0.62602, 1.5, margin=0.8)
    assert limits.linear == pytest.approx(0.8 * 0.1651 * 10.0)
    assert limits.angular == pytest.approx(2 * limits.linear / (0.62602 * 1.5))


@pytest.mark.parametrize('bad', [dict(margin=0.0), dict(margin=1.5), dict(wheel_radius=0.0)])
def test_acceleration_limits_rejects_bad_input(bad):
    args = dict(wheel_acceleration=10.0, wheel_radius=0.1651, wheel_separation=0.62602,
                separation_multiplier=1.5, margin=0.8)
    args.update(bad)
    with pytest.raises(ValueError):
        derive_acceleration_limits(**args)
