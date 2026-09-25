# rover_gps_heading

The absolute (ENU) heading that `rover_localization` needs to fuse GPS. `navsat_transform_node`
needs the rover's heading to place GPS fixes in the map frame. The IMU's magnetometer heading is
not reliable near the drive motors, so this package derives the heading from GNSS course instead.

This is localization logic, not a sensor driver. The GPS driver and the fix-health diagnostics
live in the sensor payload (`rover_sensors/rover_gps`, container `rover-a1-sensors`). This
package only consumes `gps/fix`, so any GNSS receiver that publishes a reliable `NavSatFix`
works; `navsat_transform_node` needs it reliable too. A receiver with its own true heading (dual
antenna) can skip this node and feed its heading to `navsat_transform_node` directly.

`rover_localization` starts `rover_gps_heading_node` in GPS mode (`ROVER_USE_GPS=true`) and
loads its parameters from `config/rel_localization_with_gps.yaml`.

## Interfaces (`rover_gps_heading_node`)

| Direction | Name | Type |
|-----------|------|------|
| sub | `gps/fix` | `sensor_msgs/NavSatFix` (reliable, volatile, depth 10, from the sensor payload) |
| sub | `odom` | `nav_msgs/Odometry` (local EKF output from `rover_localization`) |
| pub | `gps/heading_imu` | `sensor_msgs/Imu`: orientation only, ENU yaw of `<namespace>/base_link`. Published only when `publish_heading` is true and the alignment has finished. Input of `rover_navsat_transform_node`. |
| srv | `gps/reset_heading_alignment` | `std_srvs/Trigger`: discard the alignment and collect it again. |
| pub | `diagnostics` | hardware id `RoverGpsHeading`, task `Heading alignment` (`/Rover/GPS` in `diagnostics_agg`) |

## Heading alignment

1. While the rover drives straight (|vx| ≥ `min_speed_m_s`, |yaw rate| ≤ `max_yaw_rate_rad_s`,
   yaw change ≤ `max_yaw_change_rad`), every `min_segment_length_m` of GNSS displacement gives
   one measurement: `offset = course_ENU − mean odom yaw`. When reversing (vx < 0), π is added.
   The course is measured between antenna positions, so the antenna offset does not bias it.
2. Once `required_segments` measurements agree within `max_offset_std_rad` (circular std), the
   offset is latched and the diagnostic becomes OK.
3. From then on every `odom` message is republished as `gps/heading_imu` with
   `yaw = odom yaw + offset`. The yaw std is the offset std, at least `min_heading_std_rad`.

**On the rover:** after start, drive straight for about 10 m (for example 3 × 3 m at ≥ 0.3 m/s)
with a good fix. Until then, the global EKF runs on wheels + IMU only.
Call `gps/reset_heading_alignment` to realign, and restart `rover_navsat_transform_node`
afterwards: it computes its transform only once.

## Parameters

| Name | Default | Description |
|------|---------|-------------|
| `publish_heading` | `true` | When false, the alignment is only estimated and reported (OK while unaligned). |
| `heading_frame_id` | `base_link` | Set by `rover_localization` to `<namespace>/base_link`. |
| `min_heading_std_rad` | `0.05` | Lower bound of the published yaw std. |
| `alignment.min_segment_length_m` | `3.0` | Straight distance per measurement [m]. |
| `alignment.min_speed_m_s` | `0.3` | Minimum \|vx\| [m/s]. |
| `alignment.max_yaw_rate_rad_s` | `0.1` | Maximum \|yaw rate\| [rad/s]. |
| `alignment.max_yaw_change_rad` | `0.1` | Maximum yaw change within a segment [rad]. |
| `alignment.max_horizontal_std_m` | `5.0` | Fixes with a larger error are ignored [m]. |
| `alignment.max_fix_gap_s` | `2.5` | A longer gap between fixes restarts the segment [s]. |
| `alignment.max_odometry_age_s` | `0.5` | A fix with older odometry is ignored [s]. |
| `alignment.required_segments` | `3` | Measurements needed. |
| `alignment.max_offset_std_rad` | `0.1` | Agreement needed [rad]. |

All parameters are read-only; invalid values stop the node at startup.

## Layout

```
domain/          GnssFix/OdometrySample value types, geo math (ENU offset, angle wrap, circular
                 stats, quaternion yaw), HeadingAlignmentEstimator, HeadingPublisherPort — no ROS
application/     AlignHeadingUseCase (fix + odom → alignment → ENU heading)
infrastructure/  RoverGpsHeadingNode (composition root, parameters, subscriptions, reset service),
                 Ros2HeadingPublisher, msg conversions
```

`test/unit/` covers the geo math, the estimator, the use case and the message mapping without a
ROS graph. `test/integration/test_rover_gps_heading_node.cpp` runs the real node in-process and
checks the alignment diagnostic, the heading output after alignment, the reset service,
parameter validation and the reliable `gps/fix` subscription. Run them with
`colcon test --packages-select rover_gps_heading`.
