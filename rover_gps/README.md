# rover_gps

GNSS integration for Rover A1. The Teltonika RUTX11 router has the GNSS receiver and forwards
its NMEA sentences over UDP. This package starts the NMEA driver for that stream, reports GPS
health on `diagnostics`, and provides the absolute (ENU) heading that `rover_localization` needs
to fuse GPS.

## Nodes

| Node | Package / executable | Role |
|------|----------------------|------|
| `rover_gps_driver` | `nmea_navsat_driver` / `nmea_socket_driver` ([rover_nmea_navsat_driver](https://github.com/RaduPotlog/rover_nmea_navsat_driver), branch `master`) | Listens for NMEA on UDP `0.0.0.0:10110` and publishes `gps/fix`. |
| `rover_gps_node` | `rover_gps` / `rover_gps_node` | GPS health diagnostics and the odom → ENU heading alignment. |

## Interfaces (`rover_gps_node`)

| Direction | Name | Type |
|-----------|------|------|
| sub | `gps/fix` | `sensor_msgs/NavSatFix` (sensor-data QoS, from `rover_gps_driver`) |
| sub | `odom` | `nav_msgs/Odometry` (local EKF output from `rover_localization`) |
| pub | `gps/heading_imu` | `sensor_msgs/Imu`: orientation only, ENU yaw of `<namespace>/base_link`. Published only when `publish_heading` is true and the alignment has finished. Input of `rover_navsat_transform_node`. |
| srv | `gps/reset_heading_alignment` | `std_srvs/Trigger`: discard the alignment and collect it again. |
| pub | `diagnostics` | hardware id `RoverGps`, tasks `GPS fix` and `Heading alignment` (`/Rover/GPS` in `diagnostics_agg`) |

The driver also publishes `gps/vel`, `gps/heading` (only if the receiver sends HDT) and
`gps/time_reference`.

### `GPS fix` diagnostic

| Level | When |
|-------|------|
| STALE | No fix message received yet. |
| ERROR | No fix message for `fix_timeout_s` (NMEA link lost), or horizontal std > `error_horizontal_std_m`. |
| WARN | Receiver reports no fix, horizontal std > `warn_horizontal_std_m`, or rate < `expected_rate_hz * min_rate_ratio`. |
| OK | Otherwise. |

Values shown: fix status, latitude/longitude/altitude, horizontal std (from the `NavSatFix`
covariance, which the driver computes from HDOP), rate, age and fix count.

### Heading alignment

`navsat_transform_node` needs the rover's absolute heading to place GPS fixes in the map frame.
The IMU's magnetometer heading is not reliable near the drive motors, so the heading comes from
GNSS course instead:

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

## Parameters (`config/rover_gps.yaml`)

| Name | Default | Description |
|------|---------|-------------|
| `expected_rate_hz` | `1.0` | GGA rate configured on the RUTX11. |
| `min_rate_ratio` | `0.5` | WARN below `expected_rate_hz * min_rate_ratio`. |
| `fix_timeout_s` | `3.0` | ERROR when no fix arrives for this long. |
| `warn_horizontal_std_m` / `error_horizontal_std_m` | `5.0` / `20.0` | Accuracy thresholds [m]. |
| `publish_heading` | `false` | Set by the launch file from `ROVER_USE_GPS`. |
| `heading_frame_id` | `base_link` | Set by the launch file to `<namespace>/base_link`. |
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

All parameters are read-only; invalid values or `warn > error` stop the node at startup.

Driver parameters (`rover_gps_driver`): `ip` `0.0.0.0`, `port` `10110`, `frame_id` `gps_link`
(prefixed with the namespace through `tf_prefix`), `useRMC` `false`. `gps_link` is defined in
`rover_description`. Its offset comes from `ROVER_GPS_LOCALIZATION_{X,Y,Z}` and
`ROVER_GPS_ORIENTATION_{R,P,Y}` and defaults to the body origin.

## Launch

```bash
ros2 launch rover_gps rover_gps.launch.py namespace:=rover publish_heading:=false
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace and TF prefix. |
| `publish_heading` | `$ROVER_USE_GPS`, else `false` | Accepts `true`/`1`/`yes`/`on` (any case). |
| `rover_gps_config_path` | `config/rover_gps.yaml` | Parameter file for both nodes. |
| `common_dir_path` | empty | If set, the default config is read from `<common_dir_path>/rover_gps/config/`. |
| `log_level` | `INFO` | Logging level. |

`rover_bringup` starts it with `publish_heading:=<use_gps>`.

## RUTX11 configuration

The RUTX11 (`192.168.1.1`) must forward NMEA over **UDP** to the rover (`192.168.1.201:10110`)
at 1 s intervals. `rover_bringup/scripts/rutx11_gps_nmea_forwarding.sh` does this through `uci`:

```bash
cd src/rover_ros/rover_bringup/scripts
./rutx11_gps_nmea_forwarding.sh show                 # read-only; prompts for the router password
RUTX11_PASSWORD=... ./rutx11_gps_nmea_forwarding.sh apply
```

`apply` sets these values, commits them and restarts `gpsd`:

- `gps.gpsd.enabled=1`
- `gps.nmea_forwarding.{enabled=1, proto=udp, hostname=$ROVER_HOST, port=$NMEA_PORT}`
- `gps.{GPGGA,GPRMC,GPVTG}.{forwarding_enabled=1, forwarding_interval=$NMEA_INTERVAL_S}`

It can be overridden with `RUTX11_HOST`, `RUTX11_USER`, `ROVER_HOST`, `NMEA_PORT` and
`NMEA_INTERVAL_S`. In the WebUI the same settings are under
**Services → GPS → NMEA → NMEA forwarding**. The antenna must see the sky: without a fix the
router still forwards GGA, and `GPS fix` reports WARN "No GNSS fix.".

Check that NMEA arrives on the rover (stop `rover_gps_driver` first, since it holds the port):

```bash
nc -ul 10110
```

## Layout

```
domain/          GnssFix/OdometrySample value types, geo math (ENU offset, angle wrap, circular
                 stats), GpsHealthEvaluator, HeadingAlignmentEstimator, output ports — no ROS
application/     MonitorGpsUseCase (fix → health report), AlignHeadingUseCase (fix + odom →
                 alignment → ENU heading)
infrastructure/  RoverGpsNode (composition root, parameters, subscriptions, reset service),
                 Ros2GpsHealthPublisher / Ros2HeadingPublisher (topics, diagnostics), msg conversions
```

The unit tests in `test/unit/` cover the domain, the use cases and the message mapping without
a ROS graph. `test/integration/test_rover_gps_node.cpp` runs the real node in-process and
checks the STALE → OK diagnostics, the heading output after alignment, the reset service and
threshold validation. Run them with `colcon test --packages-select rover_gps`.
