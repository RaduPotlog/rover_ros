# rover_description

URDF/xacro model of Rover A1: links, joints, meshes, sensors (IMU, lidar), the simulated
battery, and the `<ros2_control>` hardware components the driver stack runs on.

## Model files

```
urdf/rover_a1.urdf.xacro              entry point (xacro args below)
urdf/rover_a1/rover_a1_macro.urdf.xacro  robot macro: body, wheels, sensors, <ros2_control> system
urdf/rover_a1/base.urdf.xacro         chassis (base_link / body_link / base_footprint)
urdf/common/wheel.urdf.xacro          wheel link + continuous joint (fl, fr, rl, rr)
urdf/common/imu.urdf.xacro            imu_link, Gazebo IMU sensor, <ros2_control> sensor component
urdf/common/lidar.urdf.xacro          lidar_link and the Gazebo lidar
urdf/common/battery.urdf.xacro        Gazebo LinearBatteryPlugin
urdf/common/gazebo_system.urdf.xacro  gz_ros2_control plugin (simulation only)
meshes/rover_a1/, meshes/wheel_01/    STL meshes
rviz/rover.rviz                       RViz layout
```

`urdf/rover_a1.urdf.xacro` arguments: `use_sim`, `namespace`, `wheel_config_file`
(`config/wheel_01.yaml`), `battery_config_file` (`config/battery.yaml`),
`controller_config_file` (required with `use_sim`), `imu_xyz`/`imu_rpy`, `lidar_xyz`/`lidar_rpy`.

## ros2_control components

With `use_sim:=false` the URDF declares two hardware components. Each one runs its own node
inside `controller_manager`, named after the component:

| Component (`<ros2_control name>`) | Type | Plugin | Exports |
|-----------------------------------|------|--------|---------|
| `rover_system_node` | system | `rover_hardware_interface/RoverA1System` | velocity command + position/velocity/effort state per wheel joint; drives the Phidget DCC1000 motor controllers and talks Modbus TCP to the safety controller (E-Stop, GPIO) at `192.168.88.11:502` |
| `rover_imu` | sensor | `rover_hardware_interface/PhidgetImuSensor` (Phidgets Spatial + Madgwick filter) | `imu/orientation.*`, `imu/angular_velocity.*`, `imu/linear_acceleration.*` |

The hardware parameters (Modbus address, timeouts, gear ratio, encoder resolution, …) are the
`<param>` tags in `urdf/rover_a1/rover_a1_macro.urdf.xacro` and `urdf/common/imu.urdf.xacro`. With
`use_sim:=true`, the system uses `gz_ros2_control/GazeboSimSystem` and carries the IMU interfaces
(fed by the Gazebo IMU sensor), and `rover_imu` has no hardware.

## Frame Conventions

Standard ROS conventions ([REP-103](https://ros.org/reps/rep-0103.html)) throughout.
For `base_link` / `body_link`:

| Axis | Direction |
|------|-----------|
| `+x` / `-x` | forward / backward |
| `+y` / `-y` | left / right |
| `+z` / `-z` | up / down |
| `+yaw` (`angular.z`) | counter-clockwise seen from above, i.e. turning left |

This puts the four wheels at:

| Prefix | x | y | Corner |
|--------|---|---|--------|
| `fl` | `+wheelbase/2` | `+wheel_separation/2` | front-left |
| `fr` | `+wheelbase/2` | `-wheel_separation/2` | front-right |
| `rl` | `-wheelbase/2` | `+wheel_separation/2` | rear-left |
| `rr` | `-wheelbase/2` | `-wheel_separation/2` | rear-right |

All four wheel joints spin about **`+y`** (`<axis xyz="0 1 0"/>`). That sign is
deliberate and the same for every wheel: with the contact patch below the hub,
a positive joint velocity drives the rover **forward** (`+x`). No wheel needs a
negated axis. `rover_drive_controller`'s `left_wheel_names` are correspondingly the
`+y` pair (`rl`, `fl`) and `right_wheel_names` the `-y` pair (`rr`, `fr`).

`base_link` is the root and coincides with `body_link` (the chassis centre).
`base_footprint` is its ground projection, derived in the URDF as
`wheel axis height - wheel_radius`.

## Config Files

- `wheel_01.yaml` - Rover 4WD 13-inch wheel (`wheel_radius` 0.1651 m, 0.33 m diameter) configuration, including `wheelbase` and
  `wheel_separation`. Both are **full centre-to-centre** distances (front-to-rear and
  left-to-right respectively); the URDF halves them to get each mount point. This is the
  single source of truth for wheel geometry; `rover_controller/config/wheel_01_controller.yaml`
  must match it (checked by `rover_controller/test/test_wheel_geometry.py`).

  Note that `wheel_separation_multiplier: 1.5` in that controller config is **not** geometry -
  it is the usual empirical skid-steer correction (a 4-wheel skid-steer's effective track
  exceeds its physical track), so it is not expected to match anything here. The 1.5 was
  set by hand; measure it with `rover_controller`'s `wheel_odom_calibration` tool (see
  "Drive-train tuning" in `rover_controller/README.md`) and record the date and surface.

- `battery.yaml` - Simulated battery (Gazebo `LinearBatteryPlugin`) configuration.

## Launch Files

- `rover_load_urdf.launch.py` - generates the URDF with xacro and runs `robot_state_publisher`
  (node `rover_state_publisher_node`, publishing `robot_description` and TF with the
  `<namespace>/` frame prefix). `controller_config_path` is required: it is embedded in the URDF
  for `gz_ros2_control`, and `rover_controller.launch.py` supplies it.

  | Argument | Default | Description |
  |----------|---------|-------------|
  | `controller_config_path` | (required) | Controller config embedded in the URDF. |
  | `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace and TF frame prefix. |
  | `robot_model` | `$ROBOT_MODEL_NAME`, else `rover_a1` | Robot model. |
  | `use_sim` | `False` | Build the simulation variant (`gz_ros2_control`). |
  | `wheel_type` | `wheel_01` | `wheel_01` or `custom`; selects `config/<wheel_type>.yaml`. |
  | `wheel_config_path` | `config/<wheel_type>.yaml` | Wheel geometry file. |
  | `publish_robot_state` | `True` | Start `robot_state_publisher`. |

- `rover_rviz.launch.py` - RViz with `rviz/rover.rviz`. `<namespace>` in the config is
  substituted.

  | Argument | Default | Description |
  |----------|---------|-------------|
  | `rviz_config` | `rviz/rover.rviz` | RViz config file. |
  | `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace used in the config. |
  | `use_sim` | `False` | Use simulation time. |
