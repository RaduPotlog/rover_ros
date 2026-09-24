#!/usr/bin/env python3

# Copyright 2026 Rover A1 contributors
# Licensed under the Apache License, Version 2.0.

"""Step the body velocity, record each wheel PID's reference/feedback, report the response.

    ros2 run rover_controller wheel_step_response --ros-args -r __ns:=/<ns> \\
        -p enable_motion:=true

Writes <output_dir>/samples.csv and summary.yaml and logs recommended acceleration limits.
"""

import csv
import os
from statistics import median

from control_msgs.msg import MultiDOFStateStamped
import rclpy
from rclpy.executors import ExternalShutdownException
import yaml

from rover_controller.drive_session import build_schedule, DriveSession
from rover_controller.response_analysis import (
    analyze_step, derive_acceleration_limits, finite, Sample)

WHEEL_PIDS = [
    'pid_controller_fl_wheel_base_to_fl_wheel_joint',
    'pid_controller_fr_wheel_base_to_fr_wheel_joint',
    'pid_controller_rl_wheel_base_to_rl_wheel_joint',
    'pid_controller_rr_wheel_base_to_rr_wheel_joint',
]


class WheelStepResponse(DriveSession):

    def __init__(self):
        super().__init__('wheel_step_response')
        self.declare_parameter('pid_controllers', WHEEL_PIDS)
        self.declare_parameter('linear_steps', [0.2, 0.4, 0.6, 0.8, -0.4])
        self.declare_parameter('angular_steps', [0.5, 1.0, -1.0])
        # Must match rover_drive_controller; only used to turn wheel accel into body limits.
        self.declare_parameter('wheel_radius', 0.1651)
        self.declare_parameter('wheel_separation', 0.62602)
        self.declare_parameter('wheel_separation_multiplier', 1.5)
        self.declare_parameter('margin', 0.8)
        self.samples = {}  # wheel -> list of (t, reference, feedback, output)
        for pid in self.get_parameter('pid_controllers').value:
            self.create_subscription(
                MultiDOFStateStamped, f'{pid}/controller_state',
                lambda msg, pid=pid: self._on_state(pid, msg), 50)

    def _on_state(self, pid, msg):
        if self._start is None or not msg.dof_states:
            return
        dof = msg.dof_states[0]
        self.samples.setdefault(pid, []).append(
            (self.elapsed(), dof.reference, dof.feedback, dof.output))

    def start(self) -> bool:
        commands = [(v, 0.0, f'linear {v:+.2f}')
                    for v in self.get_parameter('linear_steps').value]
        commands += [(0.0, w, f'angular {w:+.2f}')
                     for w in self.get_parameter('angular_steps').value]
        return self.run(build_schedule(commands, self.get_parameter('hold_time').value,
                                       self.get_parameter('rest_time').value))

    def report(self):
        if not self.samples:
            self.get_logger().error(
                'No controller_state received - are the wheel PIDs active in this namespace?')
            return
        out = self.output_dir()
        with open(os.path.join(out, 'samples.csv'), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['wheel', 't', 'reference', 'feedback', 'output'])
            for wheel, rows in self.samples.items():
                writer.writerows([wheel, *row] for row in rows)

        rest = self.get_parameter('rest_time').value
        steps = []
        for segment in self.schedule:
            if segment.label == 'rest':
                continue
            per_wheel = {}
            for wheel, rows in self.samples.items():
                window = [Sample(t, ref, fb) for t, ref, fb, _ in rows
                          if segment.start - rest / 2 <= t < segment.end]
                try:
                    per_wheel[wheel] = analyze_step(window, segment.start)
                except ValueError:
                    continue
            if not per_wheel:
                continue
            metric = list(per_wheel.values())

            def med(values):
                values = [v for v in values if finite(v)]
                return round(median(values), 4) if values else None

            steps.append({
                'step': segment.label,
                'wheel_accel_rad_s2': med([m.max_acceleration for m in metric]),
                'dead_time_s': med([m.dead_time for m in metric]),
                'rise_time_s': med([m.rise_time for m in metric]),
                'overshoot': med([m.overshoot for m in metric]),
                'steady_state_error': med([m.steady_state_error for m in metric]),
            })
            self.get_logger().info(f'{steps[-1]}')

        accels = [s['wheel_accel_rad_s2'] for s in steps if s['wheel_accel_rad_s2']]
        summary = {'steps': steps}
        if accels:
            # Conservative: the weakest step bounds what every step can rely on.
            alpha = min(accels)
            limits = derive_acceleration_limits(
                alpha, self.get_parameter('wheel_radius').value,
                self.get_parameter('wheel_separation').value,
                self.get_parameter('wheel_separation_multiplier').value,
                self.get_parameter('margin').value)
            summary['wheel_accel_rad_s2'] = alpha
            summary['recommended'] = {
                'linear_accel_m_s2': round(limits.linear, 3),
                'angular_accel_rad_s2': round(limits.angular, 3),
            }
            self.get_logger().info(
                f'Wheel accel {alpha:.2f} rad/s^2 -> linear.x {limits.linear:.2f} m/s^2, '
                f'angular.z {limits.angular:.2f} rad/s^2 (margin '
                f'{self.get_parameter("margin").value}). If these sit at the drive controller '
                'limits, the controller ramp was the bottleneck - relax it and re-measure.')
        with open(os.path.join(out, 'summary.yaml'), 'w') as f:
            yaml.safe_dump(summary, f, sort_keys=False)
        self.get_logger().info(f'Wrote {out}')


def main(args=None):
    rclpy.init(args=args)
    node = WheelStepResponse()
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
