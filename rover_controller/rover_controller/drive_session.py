# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Shared plumbing for the tuning tools: a timed schedule of body-velocity commands."""

from dataclasses import dataclass
import datetime
import os
from typing import List, Optional, Tuple

from geometry_msgs.msg import TwistStamped
from rclpy.node import Node


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


class DriveSession(Node):
    """Publishes a schedule on the twist_mux teleop input; subclasses record and analyse.

    Commands go through twist_mux (priority 100 Foxglove input by default), so the E-Stop
    motion lock still stops the rover. Nothing moves unless `enable_motion` is true.
    """

    def __init__(self, name: str, hold_time: float = 3.0):
        super().__init__(name)
        self.declare_parameter('cmd_topic', 'teleop_foxglove_cmd_vel_stamped')
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('hold_time', hold_time)
        self.declare_parameter('rest_time', 2.0)
        self.declare_parameter('enable_motion', False)
        self.declare_parameter('output_dir', '')
        self._publisher = self.create_publisher(
            TwistStamped, self.get_parameter('cmd_topic').value, 10)
        self.schedule: List[Segment] = []
        self._start = None
        self._timer = None
        self.finished = False

    def now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def elapsed(self) -> float:
        return 0.0 if self._start is None else self.now_s() - self._start

    def output_dir(self) -> str:
        path = self.get_parameter('output_dir').value or os.path.join(
            os.path.expanduser('~'), 'rover_calibration',
            f'{self.get_name()}_{datetime.datetime.now():%Y%m%d_%H%M%S}')
        os.makedirs(path, exist_ok=True)
        return path

    def run(self, schedule: List[Segment]) -> bool:
        """Start publishing; returns False (and moves nothing) unless enable_motion is set."""
        self.schedule = schedule
        for segment in schedule:
            if segment.label != 'rest':
                self.get_logger().info(
                    f'{segment.start:6.1f}-{segment.end:6.1f} s  {segment.label}: '
                    f'v={segment.linear:+.2f} m/s  w={segment.angular:+.2f} rad/s')
        if not self.get_parameter('enable_motion').value:
            self.get_logger().warn(
                'Dry run: the rover would drive the schedule above. Clear the area (or lift the '
                'wheels for a first run) and re-run with -p enable_motion:=true.')
            return False
        self._start = self.now_s()
        self._timer = self.create_timer(
            1.0 / self.get_parameter('publish_rate').value, self._tick)
        return True

    def command(self, linear: float, angular: float):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = linear
        msg.twist.angular.z = angular
        self._publisher.publish(msg)

    def stop(self):
        if self._timer is not None:
            self._timer.cancel()
        for _ in range(5):
            self.command(0.0, 0.0)

    def _tick(self):
        segment = active_segment(self.schedule, self.elapsed())
        override = self.command_override(self.elapsed())
        if override is not None:
            self.command(*override)
        elif segment is None:
            self.stop()
            self.finished = True
        else:
            self.command(segment.linear, segment.angular)

    def command_override(self, t: float) -> Optional[Tuple[float, float]]:
        """Subclasses may take over the command (e.g. drive until a distance); None = schedule."""
        return None
