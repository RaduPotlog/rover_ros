# rover_lidar

LiDAR integration for Rover A1. Starts the RoboSense **RS16** driver
([`rover_rslidar_sdk`](https://github.com/RaduPotlog/rover_rslidar_sdk)), flattens its point
cloud into the `scan` topic Nav 2 costmaps consume, and reports lidar health on `diagnostics`.

This is the hardware counterpart of the simulated lidar: `rover_gazebo` bridges a `gpu_lidar`
onto `<namespace>/scan`, and this package publishes the same topic from the real sensor, so
`rover_navigation` runs unchanged in both.

## Nodes

| Node | Package / executable | Role |
|------|----------------------|------|
| *(driver, see below)* | `rover_rslidar_sdk` / `rover_rslidar_sdk_node` | Receives MSOP/DIFOP over UDP and publishes `rslidar_points`. |
| `rover_pointcloud_to_laserscan` | `pointcloud_to_laserscan` / `pointcloud_to_laserscan_node` | Flattens `rslidar_points` into `scan`. Skipped with `publish_scan:=False`. |
| `rover_lidar_node` | `rover_lidar` / `rover_lidar_node` | Lidar stream health diagnostics. |

The driver process gets **no** `name=`. It creates its ROS nodes internally with hardcoded
names (`param_handle`, `rover_rslidar_points_destination_0`), so a `__node` remap would collapse
them onto a single colliding name. The launch `namespace` is passed as a global ROS argument and
does reach all of them, so the nodes appear as `/<namespace>/rover_rslidar_points_destination_0`.

## Interfaces

| Direction | Name | Type |
|-----------|------|------|
| pub | `rslidar_points` | `sensor_msgs/PointCloud2` in `<namespace>/lidar_link`, ~10 Hz |
| pub | `scan` | `sensor_msgs/LaserScan` in `<namespace>/lidar_link` (only with `publish_scan`) |
| sub | `rslidar_points` | `rover_lidar_node`, sensor-data QoS — only the arrival time and point count are used |
| pub | `diagnostics` | hardware id `RoverLidar`, task `Lidar status` (`/Rover/Lidar` in `diagnostics_agg`) |

### `Lidar status` diagnostic

| Level | When |
|-------|------|
| STALE | No point cloud received yet. |
| ERROR | No point cloud for `cloud_timeout_s` — lidar unpowered, unplugged, or the rover is not on its subnet. |
| WARN | Cloud has fewer than `min_points_warn` points (blocked or blinded sensor), or rate < `expected_rate_hz * min_rate_ratio`. |
| OK | Otherwise. |

Values shown: cloud count, points per cloud, rate and age. The age is measured from message
**arrival**, not `header.stamp`: with `use_lidar_clock` the stamp comes from the sensor's own
clock and is not comparable to the rover's.

## Configuration — two files, on purpose

| File | Read by | Format |
|------|---------|--------|
| `config/rslidar.yaml` | `rover_rslidar_sdk_node` | **Not** a ROS parameter file. The driver takes a single ROS parameter, `config_path`, and parses this whole file itself with yaml-cpp. Keep its schema identical to upstream `rover_rslidar_sdk/config/config.yaml` or the driver aborts at startup. |
| `config/rover_lidar.yaml` | `rover_pointcloud_to_laserscan`, `rover_lidar_node` | Ordinary ROS parameters. |

In both, `<namespace>/` is substituted by the launch file (`nav2_common.launch.ReplaceString`)
with the rover namespace plus a slash, or with nothing when unnamespaced — the same idiom
`rover_controller` uses. That is what makes `ros_frame_id` match the `frame_prefix`
`robot_state_publisher` applies to the URDF frames.

Passing `config_path` is **mandatory**, not an optimization: with no override the driver falls
back to the config installed inside `rover_rslidar_sdk`, which publishes on the absolute topic
`/rslidar_points` (escaping the namespace) with `ros_frame_id: rover_rslidar` (a frame in no TF
tree). Three values therefore differ deliberately from upstream:

| Key | Upstream | Here | Why |
|-----|----------|------|-----|
| `ros_frame_id` | `rover_rslidar` | `<namespace>/lidar_link` | The frame `rover_description` actually publishes. |
| `ros_send_point_cloud_topic` | `/rslidar_points` | `rslidar_points` | Relative, so the launch namespace applies. |
| `ros_queue_length` | `1000` | `10` | 1000 buffered `PointCloud2` messages is hundreds of MB. |

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
ros2 launch rover_lidar rover_lidar.launch.py
ros2 launch rover_lidar rover_lidar.launch.py publish_scan:=False   # point cloud only
```

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of every node. |
| `log_level` | `INFO` | Logging level. |
| `common_dir_path` | empty | Directory with config overrides (`<dir>/rover_lidar/config/...`). |
| `publish_scan` | `True` | `False` skips `pointcloud_to_laserscan`. |
| `rover_lidar_config_path` | `config/rover_lidar.yaml` | ROS parameters for the two rover nodes. |
| `rslidar_config_path` | `config/rslidar.yaml` | Driver configuration. |

Checks:

```bash
ros2 topic hz /<ns>/rslidar_points                              # ~10 Hz
ros2 topic echo /<ns>/rslidar_points --field header --once      # frame_id == <ns>/lidar_link
ros2 topic hz /<ns>/scan
ros2 run tf2_ros tf2_echo <ns>/base_link <ns>/lidar_link
```

## Known limitations

- **No `ring` or `time` fields.** `rover_rslidar_sdk` is built with the compile-time
  `POINT_TYPE=XYZI`. That is enough for costmaps and for `pointcloud_to_laserscan`, but LiDAR
  odometry / SLAM stacks that deskew per point need `XYZIRT`, which means changing
  `POINT_TYPE` in the SDK's `CMakeLists.txt` and rebuilding.
- **No IMU from the lidar.** The SDK is built with `ENABLE_IMU_DATA_PARSE=OFF` and
  `imu_port: 0`; the rover's IMU is the Phidget Spatial in `rover_hardware_interface`.
- **`use_sim_time` is ignored** by the driver — its stamps come from the lidar or the system
  clock, never `/clock`. Simulation keeps using the `rover_gazebo` bridge instead.
- `scan` is a single horizontal slice (`min_height` / `max_height` around the lidar origin),
  so obstacles outside that band are invisible to a 2D costmap. Feeding the full cloud into a
  3D layer (STVL) is the alternative, and is not wired up yet.
