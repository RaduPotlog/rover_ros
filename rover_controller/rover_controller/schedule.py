# Copyright 2026 Mechatronics Academy
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
