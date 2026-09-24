# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""The tuning tools' command schedule. No ROS imports - unit-tested directly."""

from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass(frozen=True)
class Segment:
    start: float    # s since the session started
    end: float
    linear: float   # m/s
    angular: float  # rad/s
    label: str


def build_schedule(commands: List[Tuple[float, float, str]], hold_time: float,
                   rest_time: float) -> List[Segment]:
    """Rest (zero command) before every command, then hold it; a final rest at the end."""
    segments = []
    t = 0.0
    for linear, angular, label in commands:
        segments.append(Segment(t, t + rest_time, 0.0, 0.0, 'rest'))
        t += rest_time
        segments.append(Segment(t, t + hold_time, linear, angular, label))
        t += hold_time
    segments.append(Segment(t, t + rest_time, 0.0, 0.0, 'rest'))
    return segments


def active_segment(schedule: List[Segment], t: float) -> Optional[Segment]:
    for segment in schedule:
        if segment.start <= t < segment.end:
            return segment
    return None
