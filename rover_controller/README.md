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
velocity command is an open-loop, rad/s-scaled duty cycle - and PID trims the error
that load, battery voltage and slip leave. With `p = i = d = 0` it is exactly the old
open-loop drive, which is the quickest A/B comparison (gains are live parameters):

```bash
ros2 param set <ns>/pid_controller_fl_wheel_base_to_fl_wheel_joint \
  gains.fl_wheel_base_to_fl_wheel_joint.p 0.0     # likewise .i and .d, and for each wheel
```

Each PID is a `rover_controller/SeededPidController`: `pid_controller/PidController` with its
exported state seeded on activation (so diff_drive never reads NaN feedback on its first
cycle), and each wheel's command computed by `WheelSpeedLoop`. With every option below off,
`WheelSpeedLoop` gives exactly `control_toolbox::Pid`'s output (`test/test_wheel_speed_loop.cpp`).
The options are live parameters, like the gains:

| Parameter | Config | Effect |
|---|---|---|
| `stop_at_zero_reference` | `true` | At a zero reference (below `zero_reference_tolerance`, default 0.001 rad/s), send exactly 0 and clear the integral. The DCC1000 brakes only at a target of exactly 0. |
| `integral_reference_delay` / `integral_reference_time_constant` | 0.15 s / 0.08 s | The integral works on (the reference delayed, then lagged) - measurement, a model of the plant's own response, so only the error the plant won't remove by itself (load, skid, friction) winds it up. 0 / 0 = the plain integral. The delay is limited to 1.0 s. |
| `scale_integral_with_reference` | `true` | The integral fades with \|reference\| below the largest reference since the last stop (cleared on reversal), so turns end on time. |
| `turn_feedforward` | `0.0` (off) | Extra command (rad/s) while the rover turns: `turn_side · sign(ω) · turn_feedforward`, ramped in up to `turn_feedforward_full_rate` (0.3 rad/s) and scaled by the turn's share of the wheel speed (1 in a spin in place, 0 straight). Skid scrub eats a constant ~1.7 rad/s of wheel command in a spin (open loop, 2026-10-10), which the integral otherwise has to build up first. Never past `u_clamp`. Start value for tests: ~1.8. |
| `turn_side` / `turn_track_width` | ±1 / 1.0236 m | Which side the wheel is on (−1 left, +1 right) and diff_drive's effective track (`wheel_separation · wheel_separation_multiplier`); `test/test_wheel_geometry.py` checks both. |
| `turn_command_topic` / `turn_command_timeout` | `rover_drive_controller/cmd_vel_out` / 0.2 s | The body command (diff_drive's `publish_limited_velocity`), read at configure; an older command gives no turn feed-forward. |

All four wheels use the same gains, with `i_clamp` ±2.0: with the model-reference integral the
clamp only has to cover the persistent load error (in-place turns need ~2 rad/s of trim). The
comment above the gains in `config/wheel_01_controller.yaml` records why, with the
2026-10-02 ground-tune results. The reference model depends on the DCC1000 ramp
(`motor_acceleration` in the URDF): re-fit it whenever that changes. Keep
`stop_at_zero_reference` on for the real rover: with it off, a frozen integral (up to 2.0
rad/s) sits above the hardware interface's E-Stop reset deadband
(`velocity_command_zero_tolerance` 0.4 rad/s) and the E-Stop can't be reset.
`rover_gazebo/config/sim_wheel_pid.yaml` sets the delay and time constant to 0 in
simulation, which has no motor dead time.

Each PID publishes `<pid>/controller_state` (reference, feedback, error, output).
`save_i_term: false` clears each PID's integral whenever it is (re)activated; pid_controller
has no reset service, so to reset on demand deactivate and reactivate
`rover_drive_controller` together with its PIDs.
Listing plain joint names as wheel names instead drives the wheels open loop; the
launch file then spawns no PIDs (see below).

The joint-state and IMU broadcasters are spawned as before.

controller_manager runs at 25 Hz, and each controller at its own `update_rate`: the
wheel PIDs, `rover_drive_controller`, `rover_imu_broadcaster` and
`rover_joint_state_broadcaster` all at the manager's 25 Hz (the EKF runs at 50 Hz). The
encoders report at most every 100 ms (10 Hz). Every rate must divide the manager's
(`test/test_wheel_geometry.py`). The manager used to run at 100 Hz, which only re-ran the PIDs
on stale feedback, then at 50 Hz, where 22-24 % of the velocity commands were dropped because
the driver's asynchronous call takes ~17 ms (median) to complete (the `command path`
diagnostic; see the comment above the rates in `config/wheel_01_controller.yaml`).

## Drive-train tuning

Do these in order; each step relies on the one before. Both tools publish on the
twist_mux Foxglove input (priority 100, still gated by the E-Stop motion lock),
print their plan and only move the rover with `-p enable_motion:=true`. Results go
to `~/rover_calibration/<tool>_<timestamp>/` (`summary.yaml`, raw samples).

1. **Wheel-speed loop.** Run the step-response tool with the wheels off the ground
   first, then on the ground. Tune the PID gains until overshoot stays under 10 %
   and steady-state error under 3 %. The DCC1000 encoders report at most every
   100 ms (10 Hz), while the PIDs run at 25 Hz, so each PID gets a new
   measurement only every ~2.5 cycles. Keep the PID gains modest, and read measured
   dead times as accurate to about 50 ms. The driver logs each channel's actual
   encoder interval at startup.

   `docs/wheel_pid_tuning_notes.md` records the lifted-wheel measurements behind
   `p`, `i` and `d` and what each knob does on this plant (`p` above ~0.1 rings the
   loop; `d` above 0.04 amplifies encoder noise; with the plain integral, `i_clamp`
   bounded overshoot). The on-ground tune of the clamps and the wheel loop options
   is in the comment above the gains in `config/wheel_01_controller.yaml`. Read both
   before changing a gain.

   ```bash
   ros2 run rover_controller wheel_step_response --ros-args -r __ns:=/<ns> -p enable_motion:=true
   ```

2. **Acceleration limits.** The same run reports the largest wheel acceleration it
   saw and the body limits that follow from it (`margin` 0.8):
   `a = margin * r * alpha`, `alpha_z = 2 * margin * r * alpha / (wheel_separation * multiplier)`.
   If the result equals the current `linear.x/angular.z` limits in this file, the
   controller ramp was the bottleneck, not the wheels. Relax those limits for one
   measurement run and repeat. The DCC1000 on-board ramp (`motor_acceleration` in
   the URDF) adds lag of its own and sets the plant response the wheel PIDs'
   integral reference models: after changing it, repeat step 1 on the ground and
   re-fit `integral_reference_delay` / `_time_constant`.
   Then set the limits from the inside out:
   `rover_drive_controller` >= Nav2 `velocity_smoother` `max_accel`/`max_decel`
   = MPPI `ax_max`/`ax_min`/`az_max` >= `behavior_server.rotational_acc_lim`.

3. **Skid-steer calibration.** Spin in place at several rates in both directions.
   The tool compares the rotation the measured wheel speeds explain with the IMU
   gyro and reports `wheel_separation_multiplier`. Run it on the surface the rover
   mostly drives on, because the value depends on the surface. The IMU is mounted
   upside down, so the tool negates the gyro yaw rate (`imu_yaw_sign` -1); if every
   spin segment is rejected although the rover turned, re-run with `-p imu_yaw_sign:=1`.

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
