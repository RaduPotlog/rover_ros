# rover_rs16_lidar

RoboSense **RS16** LiDAR for Rover A1, in one package and one process: the driver, the
`scan` topic Nav 2 costmaps consume, and the health task `rover_diag_manager` aggregates.

This replaces the old `rover_lidar` + `rover_rslidar_sdk` pair. The point cloud, the scan and
the diagnostics are published on exactly the same topics, in the same frame, with the same
message layout, so `rover_navigation`, `pointcloud_crop_box` and `rover_gazebo` are unaffected.

This is the hardware counterpart of the simulated lidar: `rover_gazebo` bridges a `gpu_lidar`
onto `<namespace>/scan`, and this package publishes the same topic from the real sensor, so
`rover_navigation` runs unchanged in both.

## Node

| Node | Package / executable | Role |
|------|----------------------|------|
| `rover_rs16_lidar_node` | `rover_rs16_lidar` / `rover_rs16_lidar_node` | Receives MSOP/DIFOP over UDP, publishes `rslidar_points`, flattens it into `scan`, and reports stream health on `diagnostics`. |

One node, with a real `name=`. The old stack needed three processes and could not name the
driver at all, because `rslidar_sdk` created its ROS nodes internally with hardcoded names.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| pub | `rslidar_points` | `sensor_msgs/PointCloud2` in `<namespace>/lidar_link`, ~10 Hz, sensor-data QoS |
| pub | `scan` | `sensor_msgs/LaserScan` in `<namespace>/lidar_link` (only with `publish_scan`) |
| pub | `diagnostics` | hardware id `RoverLidar`, task `Lidar status` (`/Rover/Lidar` in `diagnostics_agg`) |

The `PointCloud2` layout is deliberately byte-for-byte what `rslidar_sdk` produced with
`POINT_TYPE_XYZI`: four `FLOAT32` fields (`x`, `y`, `z`, `intensity`), a 16-byte stride, and
`width`/`height` swapped relative to the driver's own convention so `pcl::PointCloud`
consumers see the cloud the way they always have.

### `Lidar status` diagnostic

| Level | When |
|-------|------|
| STALE | No point cloud received yet. |
| ERROR | No point cloud for `cloud_timeout_s` — lidar unpowered, unplugged, or the rover is not on its subnet. |
| WARN | Cloud has fewer than `min_points_warn` points (blocked or blinded sensor), or rate < `expected_rate_hz * min_rate_ratio`. |
| OK | Otherwise. |

Values shown: cloud count, points per cloud, rate and age. The age is measured from frame
**arrival**, not the cloud stamp: with `use_lidar_clock` the stamp comes from the sensor's own
clock and is not comparable to the rover's.

The status string downstream consumers match is **`rover_rs16_lidar_node: Lidar status`**
(`rover_navigation`'s `IsLidarHealthy`, `rover_mission_manager`'s `lidar_status_name`, and
`rover_diag_manager`'s `find_and_remove_prefix`).

## Configuration — one file

`config/rover_rs16_lidar.yaml` is an ordinary ROS parameter file, read by the one node. There
is no second yaml-cpp schema to keep in step with upstream, and no `config_path` parameter.

`<namespace>/` is substituted by the launch file (`nav2_common.launch.ReplaceString`) with the
rover namespace plus a slash, or with nothing when unnamespaced — the same idiom
`rover_controller` uses. That is what makes `frame_id` match the `frame_prefix`
`robot_state_publisher` applies to the URDF frames.

Parameters are declared **read-only with ranges**, and inconsistent combinations are rejected
at construction rather than at the first cloud (`min_distance >= max_distance`, `input_type:
pcap` with no `pcap_path`, an `angle_increment` wider than the sweep, …).

| Group | Keys |
|-------|------|
| topics / frame | `frame_id`, `point_cloud_topic`, `scan_topic`, `publish_scan`, `publisher_queue_size` |
| sensor | `input_type`, `msop_port`, `difop_port`, `host_address`, `group_address`, `min_distance`, `max_distance`, `use_lidar_clock`, `dense_points`, `ts_first_point`, `wait_for_difop`, `start_angle`, `end_angle` |
| replay | `pcap_path`, `pcap_repeat`, `pcap_rate` |
| scan | `scan.min_height`, `scan.max_height`, `scan.angle_min`, `scan.angle_max`, `scan.angle_increment`, `scan.scan_time`, `scan.range_min`, `scan.range_max`, `scan.use_inf` |
| health | `expected_rate_hz`, `min_rate_ratio`, `cloud_timeout_s`, `min_points_warn`, `publish_frequency` |

### Differences from the stack this replaces

| | Before | Now |
|---|---|---|
| Processes | 3 (`rover_rslidar_sdk_node`, `pointcloud_to_laserscan`, `rover_lidar_node`) | 1 |
| Config files | 2, one of them not a ROS parameter file | 1 |
| Point cloud QoS | RELIABLE (`rclcpp::QoS(1000)` upstream) | sensor-data (best-effort), depth `publisher_queue_size` |
| Scan | upstream `pointcloud_to_laserscan` + TF round-trip to the frame it was already in | in-package projection, no TF |
| Scan `target_frame` | a parameter | gone — the scan is published in the sensor frame, which is what the old value resolved to anyway |
| Diagnostic status name | `rover_lidar_node: Lidar status` | `rover_rs16_lidar_node: Lidar status` |

## Architecture

Clean Architecture, three layers, the same shape as `rover_gps`:

```
domain/         PointCloudFrame, LaserScanFrame, LidarSettings, ScanProjector,
                LidarHealthEvaluator  +  ports/ (LidarSourcePort, the three publisher ports)
application/    StreamLidarUseCase (frame -> cloud + scan + health), MonitorLidarUseCase
infrastructure/ RsDriverLidarSource (the only file that includes rs_driver),
                Ros2PointCloudPublisher, Ros2LaserScanPublisher, Ros2LidarHealthPublisher,
                RoverRs16LidarNode (composition root)
```

`rover_rs16_lidar_core` links neither ROS nor `rs_driver`, which is what keeps the projection
and the health rules unit-testable on their own. `LidarSourcePort` is what lets the whole
pipeline be exercised with no sensor on the network — the integration test injects a stub.

`rs_driver` itself is vendored, header-only, under `3rdparty/rs_driver/`; see
`3rdparty/rs_driver/README.rover.md` for the version and the re-vendoring procedure.

### Threading

`rs_driver` decodes on its own threads and hands finished revolutions back through a
get/put callback pair, so buffers are recycled through a free queue and, after the first few
revolutions, nothing is allocated per frame. A single worker thread drains the filled queue,
converts, and does the publishing — the ROS executor is never blocked by a 30 000-point copy.
`MonitorLidarUseCase` is therefore touched from two threads and locks internally.

## Hardware setup

The RS16 streams UDP to the rover: **MSOP 6699** (points) and **DIFOP 7788** (device info).
The lidar ships on a fixed IP (RoboSense default `192.168.1.200`, destination `192.168.1.102`),
so the rover needs an address on that subnet and must not firewall those ports. Verify with
`sudo tcpdump -i <iface> udp port 6699` before blaming the driver.

Mount pose is **not** set here — `rover_description` places `lidar_link` relative to
`body_link` from the `ROVER_LIDAR_LOCALIZATION_X/Y/Z` and `ROVER_LIDAR_ORIENTATION_R/P/Y`
environment variables (see `rover_docker/docker-compose.yml`).

## Usage

Started by `rover_bringup` when `ROVER_USE_LIDAR` is true (default `false`, so rovers with no
lidar fitted are unaffected). Standalone:

```bash
ros2 launch rover_rs16_lidar rover_rs16_lidar.launch.py
ros2 launch rover_rs16_lidar rover_rs16_lidar.launch.py publish_scan:=False   # cloud only

# no hardware: replay a capture taken with
#   sudo tcpdump -i <iface> -w rs16.pcap udp port 6699 or udp port 7788
ros2 launch rover_rs16_lidar rover_rs16_lidar.launch.py \
    input_type:=pcap pcap_path:=/path/rs16.pcap
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of the node. |
| `log_level` | `INFO` | Logging level. |
| `common_dir_path` | empty | Directory with config overrides (`<dir>/rover_rs16_lidar/config/...`). |
| `publish_scan` | `True` | `False` publishes the point cloud only. |
| `input_type` | `lidar` | `pcap` replays a capture instead of reading the sensor. |
| `pcap_path` | empty | Capture to replay; required with `input_type:=pcap`. |
| `rover_rs16_lidar_config_path` | `config/rover_rs16_lidar.yaml` | Parameter file. |

Checks:

```bash
ros2 topic hz /<ns>/rslidar_points                              # ~10 Hz
ros2 topic echo /<ns>/rslidar_points --field header --once      # frame_id == <ns>/lidar_link
ros2 topic hz /<ns>/scan
ros2 topic echo /<ns>/diagnostics --once                        # rover_rs16_lidar_node: Lidar status
ros2 run tf2_ros tf2_echo <ns>/base_link <ns>/lidar_link
```

## Tests

```bash
colcon build --packages-select rover_rs16_lidar --cmake-args -DBUILD_TESTING=ON
colcon test --packages-select rover_rs16_lidar && colcon test-result --all
```

Unit tests cover the health rules, the settings validation and the scan projection against
`rover_rs16_lidar_core` alone (no ROS, no hardware). The integration test builds the real node
with a stub `LidarSourcePort` and asserts the published cloud layout, the scan, the
`publish_scan` switch, the diagnostic name and level, and that the source is stopped on
shutdown.

## Known limitations

- **No `ring` or `time` fields.** `rs_driver` is compiled with `POINT_TYPE_XYZI`, as
  `rover_rslidar_sdk` was. That is enough for costmaps and for the scan projection, but LiDAR
  odometry / SLAM stacks that deskew per point need `XYZIRT`, which means changing
  `POINT_TYPE_XYZI` in `CMakeLists.txt`, widening `domain::LidarPoint`, and adding the two
  fields in `Ros2PointCloudPublisher`.
- **No IMU from the lidar.** Built without `ENABLE_IMU_DATA_PARSE`; the rover's IMU is the
  Phidget Spatial in `rover_hardware_interface`.
- **`use_sim_time` does not apply to the cloud stamps** — they come from the lidar or the
  system clock, never `/clock`. Simulation keeps using the `rover_gazebo` bridge instead.
- **The scan is a single horizontal slice** (`scan.min_height` / `scan.max_height` around the
  lidar origin), so obstacles outside that band are invisible to a 2D costmap. Feeding the
  full cloud into a 3D layer (STVL) is the alternative, and is not wired up yet.
- **The scan is published in the sensor frame only.** The old `target_frame` parameter is
  gone; it always resolved to `lidar_link`, the frame the cloud was already in.
