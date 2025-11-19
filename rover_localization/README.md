# rover_localization

This package is responsible for Rover's sensors fussion.

## Config Files

- `rel_localization.yaml` - Ekf_filter's configuration for data fusion using the IMU and odometry published by the rover_controller.

## Launch Files

- `rover_localization.launch.py` - Activates EKF filter.