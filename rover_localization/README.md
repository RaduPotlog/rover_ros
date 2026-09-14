# rover_localization

This package is responsible for Rover's sensors fussion.

## Config Files

- `rel_localization.yaml` - Ekf_filter's configuration for data fusion using the IMU (`imu/data`) and wheel odometry (`odometry/wheels`) published by rover_controller; the filtered output is published on `odom`.
  `print_diagnostics` must stay `true`: robot_localization always reports the
  `odometry/filtered topic status` frequency diagnostic, but only counts publishes when it is enabled,
  so with `false` that status is a permanent false ERROR ("No events recorded").

## Launch Files

- `rover_localization.launch.py` - Activates EKF filter.