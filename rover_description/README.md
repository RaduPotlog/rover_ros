# rover_description

Package containing URDF files responsible for creating a representation of the Rover.

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
negated axis. `drive_controller`'s `left_wheel_names` are correspondingly the
`+y` pair (`rl`, `fl`) and `right_wheel_names` the `-y` pair (`rr`, `fr`).

`base_link` is the root and coincides with `body_link` (the chassis centre).
`base_footprint` is its ground projection, derived in the URDF as
`wheel axis height - wheel_radius`.

## Config Files

- `wheel_01.yaml` - Rover 4WD (33 inch) wheel's configuration, including `wheelbase` and
  `wheel_separation`. Both are **full centre-to-centre** distances (front-to-rear and
  left-to-right respectively); the URDF halves them to get each mount point. This is the
  single source of truth for wheel geometry; `rover_controller/config/wheel_01_controller.yaml`
  must match it (checked by `rover_controller/test/test_wheel_geometry.py`).

  Note that `wheel_separation_multiplier: 1.5` in that controller config is **not** geometry -
  it is the usual empirical skid-steer correction (a 4-wheel skid-steer's effective track
  exceeds its physical track), so it is not expected to match anything here.

- `battery.yaml` - Simulated battery (Gazebo `LinearBatteryPlugin`) configuration.

## Launch Files

- `rover_load_urdf.launch.py` - Generates the Rover's URDF and runs `robot_state_publisher`.
  `controller_config_path` is required (it is embedded in the URDF for `gz_ros2_control`) and is
  supplied by `rover_controller.launch.py`.