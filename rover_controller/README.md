# rover_controller

Configuration and launch composition for the Rover A1 ros2_control controllers.
Controller lifecycle and hardware interfaces are managed by controller_manager.

## Configuration

`config/wheel_01_controller.yaml` configures the 4WD Rover with 13-inch-diameter
wheels. Physical wheel geometry matches `rover_description`; calibration
multipliers are separate tuning parameters.

The active topology uses `diff_drive_controller/DiffDriveController` directly
with the wheel velocity interfaces, plus joint-state and IMU broadcasters.
The wheel PID definitions are **experimental and inactive**. Activating them
requires a deliberate controller chain using reference interfaces and appropriate
activation ordering; spawning them alongside direct wheel control would compete
for the same physical command interfaces. Their dependency and gains are retained
for future experimentation.

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
broadcaster. Each step requires the previous spawner to succeed. A mandatory
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
    drive_controller:
      type: diff_drive_controller/DiffDriveController
      node_options_args: ["-r", "~/odom:=odometry/wheels"]
```

`--ros-args` is prepended automatically; remap targets are relative, so they follow
the namespace. The spawner's `--controller-ros-args` sets the same parameter.
Resulting topics (in the rover namespace): `cmd_vel` (drive controller input),
`odometry/wheels`, `imu/data`, `joint_states`; lifecycle `transition_event` topics
are hidden under `_<controller>/`.
