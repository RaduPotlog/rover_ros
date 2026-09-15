# rover_localization

Fuses the rover's wheel odometry and IMU with a `robot_localization` EKF. The result is the
`odom` odometry and the `<namespace>/odom → <namespace>/base_link` transform that Nav 2 uses.

## Interfaces (`rover_ekf_node`)

| Direction | Name | Type |
|-----------|------|------|
| sub | `odometry/wheels` | `nav_msgs/Odometry` (from `rover_controller`'s `rover_drive_controller`) |
| sub | `imu/data` | `sensor_msgs/Imu` (from `rover_controller`'s `rover_imu_broadcaster`) |
| pub | `odom` | `nav_msgs/Odometry` (the EKF's `odometry/filtered`, remapped) |
| pub | `/tf` | `<namespace>/odom → <namespace>/base_link` (`tf_prefix` = namespace) |
| srv | `localization/set_pose`, `localization/enable`, `localization/toggle` | robot_localization services |
| pub | `diagnostics` | robot_localization status |

The filter runs at 50 Hz in 2D mode (`two_d_mode: true`).

## Config Files

- `config/rel_localization.yaml` - EKF configuration for data fusion using the IMU (`imu/data`)
  and wheel odometry (`odometry/wheels`); the filtered output is published on `odom`.
  `print_diagnostics` must stay `true`. robot_localization always reports the
  `odometry/filtered topic status` frequency diagnostic, but only counts publishes when it is
  enabled, so with `false` that status is a permanent false ERROR ("No events recorded").

## Launch Files

- `rover_localization.launch.py` - starts `rover_ekf_node` (and optionally
  `navsat_transform_node`).

| Argument | Default | Description |
|----------|---------|-------------|
| `use_ekf` | `False` | Start the EKF. `rover_bringup` and `rover_gazebo` pass `True`. |
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace and TF prefix. |
| `localization_mode` | `rel` | `rel`: relative to the start pose; `enu`: East-North-Up orientation. Selects `config/<mode>_localization[_with_gps].yaml`. |
| `fuse_gps` | `False` | Also start `navsat_transform_node` and use the `_with_gps` config. |
| `launch_nmea_gps` | `False` | Start an NMEA GPS driver. |
| `localization_config_path` | `config/<mode>_localization[_with_gps].yaml` | Explicit EKF config. |
| `common_dir_path` | empty | If set, the default config is read from `<common_dir_path>/rover_localization/config/`. |
| `use_sim` | `False` | Use simulation time. |
| `log_level` | `INFO` | Logging level. |

```bash
ros2 launch rover_localization rover_localization.launch.py use_ekf:=True
ros2 topic echo /rover/odom
```

## Known limitations

- EKF parameters are not visible in Foxglove's Parameters panel. robot_localization declares
  `odom1`, `imu1`, `pose0` and `twist0` without values while discovering sensors, so the
  bridge's bulk parameter request is rejected as a whole. The resulting rclcpp warning
  (`Failed to get parameters: parameter 'imu1' is not initialized`) is silenced in the launch
  file with `--log-level rclcpp:=ERROR`.
- Only `rel_localization.yaml` exists. `localization_mode:=enu` and `fuse_gps:=True` select
  config files that are not in the package, so pass `localization_config_path` with them.
- `launch_nmea_gps:=True` includes `rover_localization.launch.py` itself instead of a GPS driver
  launch file.
