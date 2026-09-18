# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Pure analysis of a wheel-speed step response. No ROS imports - unit-tested directly."""

from dataclasses import dataclass
import math
from statistics import mean, median
from typing import Optional, Sequence


@dataclass(frozen=True)
class Sample:
    t: float          # s
    reference: float  # rad/s, what the wheel was asked for
    feedback: float   # rad/s, what the encoder measured


@dataclass(frozen=True)
class StepMetrics:
    initial: float                       # rad/s before the step
    target: float                        # rad/s commanded after the step
    dead_time: Optional[float]           # s until feedback moved 5 % of the step
    rise_time: Optional[float]           # s from 10 % to 90 % of the step
    max_acceleration: float              # rad/s^2, largest |d feedback / dt|
    overshoot: float                     # fraction of the step, >= 0
    steady_state_error: Optional[float]  # fraction of the step, signed (+ = too fast)


def _crossing(samples: Sequence[Sample], start: float, initial: float, amplitude: float,
              fraction: float) -> Optional[float]:
    for s in samples:
        if s.t >= start and (s.feedback - initial) / amplitude >= fraction:
            return s.t
    return None


def max_acceleration(samples: Sequence[Sample], span: float = 0.05) -> float:
    """Largest |slope| of feedback over windows of at least `span` s (tames encoder noise)."""
    best = 0.0
    j = 0
    for i, s in enumerate(samples):
        j = max(j, i)
        while j < len(samples) and samples[j].t - s.t < span:
            j += 1
        if j == len(samples):
            break
        dt = samples[j].t - s.t
        best = max(best, abs(samples[j].feedback - s.feedback) / dt)
    return best


def analyze_step(samples: Sequence[Sample], step_time: float,
                 settle_window: float = 0.5, pre_window: float = 0.3) -> StepMetrics:
    """Metrics for one step that happens at `step_time` in a record of `samples`.

    `samples` must be sorted by time and cover some time before the step and enough time
    after it to settle; the last `settle_window` seconds are taken as steady state.
    """
    before = [s.feedback for s in samples if step_time - pre_window <= s.t < step_time]
    after = [s for s in samples if s.t >= step_time]
    if not before or not after:
        raise ValueError('record must contain samples before and after the step')
    initial = mean(before)
    target = median(s.reference for s in after)
    amplitude = target - initial
    accel = max_acceleration(after)
    if abs(amplitude) < 1e-6:
        return StepMetrics(initial, target, None, None, accel, 0.0, None)

    moved = _crossing(after, step_time, initial, amplitude, 0.05)
    t10 = _crossing(after, step_time, initial, amplitude, 0.1)
    t90 = _crossing(after, step_time, initial, amplitude, 0.9)
    peak = max((s.feedback - initial) / amplitude for s in after)
    end = after[-1].t
    settled = [s.feedback for s in after if s.t >= end - settle_window]
    error = (mean(settled) - target) / abs(amplitude) if end - step_time > settle_window else None
    if error is not None and amplitude < 0:
        error = -error  # + always means "faster than asked" in the direction of the step
    return StepMetrics(
        initial=initial,
        target=target,
        dead_time=None if moved is None else moved - step_time,
        rise_time=None if t10 is None or t90 is None else t90 - t10,
        max_acceleration=accel,
        overshoot=max(0.0, peak - 1.0),
        steady_state_error=error,
    )


@dataclass(frozen=True)
class AccelerationLimits:
    linear: float   # m/s^2
    angular: float  # rad/s^2


def derive_acceleration_limits(wheel_acceleration: float, wheel_radius: float,
                               wheel_separation: float, separation_multiplier: float,
                               margin: float = 0.8) -> AccelerationLimits:
    """Body acceleration limits a wheel that reaches `wheel_acceleration` rad/s^2 can follow.

    Straight line: both sides accelerate together, a = r * alpha.
    In-place turn: sides accelerate in opposite directions,
    alpha_z = 2 * r * alpha / (wheel_separation * separation_multiplier).
    `margin` keeps the commanded ramp below what the wheel can just barely do.
    """
    if min(wheel_acceleration, wheel_radius, wheel_separation, separation_multiplier) <= 0:
        raise ValueError('all inputs must be > 0')
    if not 0 < margin <= 1:
        raise ValueError('margin must be in (0, 1]')
    rim = margin * wheel_radius * wheel_acceleration
    return AccelerationLimits(
        linear=rim,
        angular=2.0 * rim / (wheel_separation * separation_multiplier),
    )


def finite(value: Optional[float]) -> bool:
    return value is not None and math.isfinite(value)
