# rover_localization

This package is responsible for Rover's sensors fussion.

## Config Files

- `rel_localization.yaml` - Ekf_filter's configuration for data fusion using the IMU (`imu/data`) and wheel odometry (`odometry/wheels`) published by rover_controller; the filtered output is published on `odom`.
  `print_diagnostics` must stay `true`: robot_localization always reports the
  `odometry/filtered topic status` frequency diagnostic, but only counts publishes when it is enabled,
  so with `false` that status is a permanent false ERROR ("No events recorded").

## Known limitations

- EKF parameters are not visible in Foxglove's Parameters panel. robot_localization declares
  `odom1`, `imu1`, `pose0` and `twist0` without values while discovering sensors, so the
  bridge's bulk parameter request is rejected as a whole. The resulting rclcpp warning
  (`Failed to get parameters: parameter 'imu1' is not initialized`) is silenced in the launch
  file with `--log-level rclcpp:=ERROR`.

## Launch Files

- `rover_localization.launch.py` - Activates EKF filter.