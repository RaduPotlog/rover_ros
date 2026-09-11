# rover_description

Package containing URDF files responsible for creating a representation of the Rover.

## Config Files

- `wheel_01.yaml` - Rover 4WD (33 inch) wheel's configuration, including `wheelbase` and
  `wheel_separation`. This is the single source of truth for wheel geometry;
  `rover_controller/config/wheel_01_controller.yaml` must match it (checked by
  `rover_controller/test/test_wheel_geometry.py`).

- `battery.yaml` - Simulated battery (Gazebo `LinearBatteryPlugin`) configuration.

## Launch Files

- `rover_load_urdf.launch.py` - Generates the Rover's URDF and runs `robot_state_publisher`.
  `controller_config_path` is required (it is embedded in the URDF for `gz_ros2_control`) and is
  supplied by `rover_controller.launch.py`.