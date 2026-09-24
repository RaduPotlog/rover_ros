#!/usr/bin/env python3

# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Calibrate the skid-steer wheel_separation_multiplier (spin) or wheel radius (straight).

Spin in place at several rates and compare the rotation the wheels explain against the gyro:
    ros2 run rover_controller wheel_odom_calibration --ros-args -r __ns:=/<ns> \\
        -p mode:=spin -p enable_motion:=true

Drive straight until wheel odometry reads `distance`, then tape-measure the real distance:
    ... -p mode:=straight -p distance:=5.0 -p enable_motion:=true
    radius_multiplier = measured / distance
"""

import os
from statistics import mean

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, JointState
import yaml

from rover_controller.calibration_analysis import calibrate_separation, SpinSegment
from rover_controller.drive_session import DriveSession
from rover_controller.schedule import build_schedule

LEFT = ['fl_wheel_base_to_fl_wheel_joint', 'rl_wheel_base_to_rl_wheel_joint']
RIGHT = ['fr_wheel_base_to_fr_wheel_joint', 'rr_wheel_base_to_rr_wheel_joint']


class WheelOdomCalibration(DriveSession):

    def __init__(self):
        super().__init__('wheel_odom_calibration', hold_time=6.0)
        self.declare_parameter('mode', 'spin')
        self.declare_parameter('yaw_rates', [0.3, 0.6, 1.0, -0.3, -0.6, -1.0])
        self.declare_parameter('discard_time', 2.0)  # s of each spin left out (transient)
        self.declare_parameter('speed', 0.3)
        self.declare_parameter('distance', 5.0)
        # Joint names are matched by suffix, so a namespace prefix doesn't matter.
        self.declare_parameter('left_joints', LEFT)
        self.declare_parameter('right_joints', RIGHT)
        self.declare_parameter('wheel_radius', 0.1651)
        self.declare_parameter('wheel_separation', 0.62602)
        self.wheels = []   # (t, left rim m/s, right rim m/s)
        self.gyro = []     # (t, yaw rate)
        self.distance = 0.0
        self._last_wheel_t = None
        self.create_subscription(JointState, 'joint_states', self._on_joints, 50)
        self.create_subscription(Imu, 'imu/data', self._on_imu, qos_profile_sensor_data)

    def _side(self, msg, suffixes):
        radius = self.get_parameter('wheel_radius').value
        values = [v for n, v in zip(msg.name, msg.velocity)
                  if any(n.endswith(s) for s in suffixes)]
        return mean(values) * radius if values else None

    def _on_joints(self, msg):
        if self._start is None:
            return
        left = self._side(msg, self.get_parameter('left_joints').value)
        right = self._side(msg, self.get_parameter('right_joints').value)
        if left is None or right is None:
            return
        t = self.elapsed()
        if self._last_wheel_t is not None:
            self.distance += (left + right) / 2.0 * (t - self._last_wheel_t)
        self._last_wheel_t = t
        self.wheels.append((t, left, right))

    def _on_imu(self, msg):
        if self._start is not None:
            self.gyro.append((self.elapsed(), msg.angular_velocity.z))

    def start(self) -> bool:
        mode = self.get_parameter('mode').value
        hold = self.get_parameter('hold_time').value
        rest = self.get_parameter('rest_time').value
        if mode == 'spin':
            commands = [(0.0, w, f'spin {w:+.2f}')
                        for w in self.get_parameter('yaw_rates').value]
            return self.run(build_schedule(commands, hold, rest))
        if mode == 'straight':
            speed = self.get_parameter('speed').value
            # Generous hold; command_override() stops at the target distance.
            timeout = 3.0 * self.get_parameter('distance').value / abs(speed)
            return self.run(build_schedule([(speed, 0.0, 'straight')], timeout, rest))
        raise ValueError(f"mode must be 'spin' or 'straight', got {mode!r}")

    def command_override(self, t):
        if (self.get_parameter('mode').value == 'straight' and
                abs(self.distance) >= self.get_parameter('distance').value):
            self.finished = True
            self.stop()
            return (0.0, 0.0)
        return None

    def report(self):
        out = self.output_dir()
        if self.get_parameter('mode').value == 'straight':
            self.get_logger().info(
                f'Wheel odometry distance: {self.distance:.3f} m. Measure the real distance d; '
                f'right/left_wheel_radius_multiplier = d / {abs(self.distance):.3f}.')
            with open(os.path.join(out, 'summary.yaml'), 'w') as f:
                yaml.safe_dump({'odometry_distance_m': self.distance}, f)
            return

        discard = self.get_parameter('discard_time').value
        segments = []
        for seg in self.schedule:
            if seg.label == 'rest':
                continue
            window = (seg.start + discard, seg.end)
            wheels = [w for w in self.wheels if window[0] <= w[0] < window[1]]
            gyro = [g for g in self.gyro if window[0] <= g[0] < window[1]]
            if not wheels or not gyro:
                self.get_logger().warning(f'{seg.label}: no joint_states or imu/data samples')
                continue
            segments.append(SpinSegment(
                commanded_yaw_rate=seg.angular,
                left_rim_speed=mean(w[1] for w in wheels),
                right_rim_speed=mean(w[2] for w in wheels),
                imu_yaw_rate=mean(g[1] for g in gyro)))
        separation = self.get_parameter('wheel_separation').value
        result = calibrate_separation(segments, separation)
        rows = []
        for seg in segments:
            rows.append({'commanded': seg.commanded_yaw_rate, 'imu': round(seg.imu_yaw_rate, 4),
                         'left_m_s': round(seg.left_rim_speed, 4),
                         'right_m_s': round(seg.right_rim_speed, 4)})
            self.get_logger().info(f'{rows[-1]}')
        self.get_logger().info(
            f'wheel_separation_multiplier = {result.multiplier:.3f} '
            f'(per segment {[round(v, 3) for v in result.per_segment]}, '
            f'{result.rejected} rejected). It depends on the surface - calibrate on the one '
            'the rover mostly drives on.')
        with open(os.path.join(out, 'summary.yaml'), 'w') as f:
            yaml.safe_dump({'wheel_separation_multiplier': round(result.multiplier, 4),
                            'per_segment': [round(v, 4) for v in result.per_segment],
                            'rejected': result.rejected, 'segments': rows}, f, sort_keys=False)
        self.get_logger().info(f'Wrote {out}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomCalibration()
    try:
        if node.start():
            while rclpy.ok() and not node.finished:
                rclpy.spin_once(node, timeout_sec=0.1)
            node.report()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.stop()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
