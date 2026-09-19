# rover_controller

Configuration and launch composition for the Rover A1 ros2_control controllers.
Controller lifecycle and hardware interfaces are managed by controller_manager.

## Configuration

`config/wheel_01_controller.yaml` configures the 4WD Rover with 13-inch-diameter
wheels. Physical wheel geometry matches `rover_description`; calibration
multipliers are separate tuning parameters.

The active topology closes the wheel-speed loop with chained controllers:

```
cmd_vel -> rover_drive_controller (diff_drive) -> pid_controller_<wheel> x4 -> hardware
```

`rover_drive_controller`'s wheel names are `<pid controller>/<joint>`, so it writes
each PID's reference interface and reads back the PID's exported state (the measured
wheel velocity) for odometry. Each PID uses `feedforward_gain: 1.0` - the hardware
velocity command is an open-loop, rad/s-scaled duty cycle - and PI trims the error
that load, battery voltage and slip leave. With `p = i = 0` it is exactly the old
open-loop drive, which is the quickest A/B comparison (gains are live parameters):

```bash
ros2 param set <ns>/pid_controller_fl_wheel_base_to_fl_wheel_joint \
  gains.fl_wheel_base_to_fl_wheel_joint.p 0.0     # likewise .i, and for each wheel
```

Each PID publishes `<pid>/controller_state` (reference, feedback, error, output).
`save_i_term: false` clears each PID's integral whenever it is (re)activated; pid_controller
has no reset service, so to reset on demand deactivate and reactivate
`rover_drive_controller` together with its PIDs.
Listing plain joint names as wheel names instead drives the wheels open loop; the
launch file then spawns no PIDs (see below).

The joint-state and IMU broadcasters are spawned as before.

## Drive-train tuning

Do these in order; each step relies on the one before. Both tools publish on the
twist_mux Foxglove input (priority 100, still gated by the E-Stop motion lock),
print their plan and only move the rover with `-p enable_motion:=true`. Results go
to `~/rover_calibration/<tool>_<timestamp>/` (`summary.yaml`, raw samples).

1. **Wheel-speed loop.** Run the step-response tool with the wheels off the ground
   first, then on the ground. Tune the PID gains until overshoot stays under 10 %
   and steady-state error under 3 %. The DCC1000 encoders report at most every
   50 ms (20 Hz), while the controllers run at 100 Hz, so each PID gets a new
   measurement only every ~5 cycles. Keep the PI gains modest, and read measured
   dead times as accurate to about 50 ms. The driver logs each channel's actual
   encoder interval at startup.

   ```bash
   ros2 run rover_controller wheel_step_response --ros-args -r __ns:=/<ns> -p enable_motion:=true
   ```

2. **Acceleration limits.** The same run reports the largest wheel acceleration it
   saw and the body limits that follow from it (`margin` 0.8):
   `a = margin * r * alpha`, `alpha_z = 2 * margin * r * alpha / (wheel_separation * multiplier)`.
   If the result equals the current `linear.x/angular.z` limits in this file, the
   controller ramp was the bottleneck, not the wheels. Relax those limits for one
   measurement run and repeat. The DCC1000 on-board ramp (`motor_acceleration` in
   the URDF) adds lag of its own, so raise it once the PID owns the speed dynamics.
   Then set the limits from the inside out:
   `rover_drive_controller` >= Nav2 `velocity_smoother` `max_accel`/`max_decel`
   = MPPI `ax_max`/`ax_min`/`az_max` >= `behavior_server.rotational_acc_lim`.

3. **Skid-steer calibration.** Spin in place at several rates in both directions.
   The tool compares the rotation the measured wheel speeds explain with the IMU
   gyro and reports `wheel_separation_multiplier`. Run it on the surface the rover
   mostly drives on, because the value depends on the surface.

   ```bash
   ros2 run rover_controller wheel_odom_calibration --ros-args -r __ns:=/<ns> -p mode:=spin -p enable_motion:=true
   ros2 run rover_controller wheel_odom_calibration --ros-args -r __ns:=/<ns> -p mode:=straight -p distance:=5.0 -p enable_motion:=true
   ```

   `straight` stops when wheel odometry reads `distance`. Tape-measure the real
   distance `d`; then `left/right_wheel_radius_multiplier = d / distance`.
   `test/test_wheel_geometry.py` keeps both multipliers in plausible ranges.

## Launch

`rover_controller.launch.py` loads the robot description and starts the controllers.
On hardware it launches `ros2_control_node`; with `use_sim:=True`, the simulation's
`gz_ros2_control` plugin owns the controller manager.

Configuration selection, in descending precedence:

1. Explicit `controller_config_path`.
2. `<common_dir_path>/rover_controller/config/<wheel_type>_controller.yaml`.
3. The package's bundled `config/<wheel_type>_controller.yaml`.

The selected file is used by the manager, every spawner, and the simulation
robot description. `<namespace>/` placeholders in the file are replaced with
the requested namespace prefix (or an empty string for the root namespace).

Startup activates the joint-state broadcaster, drive controller, then IMU
broadcaster. The drive controller's spawner also takes every controller named as
`<controller>/<joint>` in its wheel names (the wheel PIDs) and activates them with
it as one group (`--activate-as-group`), so controller_manager orders the chain. Each step requires the previous spawner to succeed. A mandatory
spawner failure reports its controller name and exit code and shuts down the
launch; subsequent controllers are not started.

## Topic remapping

Controller topics are remapped with each controller's `node_options_args` under
`controller_manager` in the config file, **not** with launch `remappings=` on
`ros2_control_node` or `<remapping>` tags in the Gazebo plugin. controller_manager
creates every controller as its own node with `use_global_arguments(false)`, so
process-wide remaps only affect the controller manager itself.

```yaml
controller_manager:
  ros__parameters:
    rover_drive_controller:
      type: diff_drive_controller/DiffDriveController
      node_options_args: ["-r", "~/odom:=odometry/wheels"]
```

`--ros-args` is prepended automatically; remap targets are relative, so they follow
the namespace. The spawner's `--controller-ros-args` sets the same parameter.
Resulting topics (in the rover namespace): `cmd_vel` (drive controller input),
`odometry/wheels`, `imu/data`, `joint_states`; lifecycle `transition_event` topics
are hidden under `_<controller>/`.
