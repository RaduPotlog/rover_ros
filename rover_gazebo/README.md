# rover_gazebo

Runs Rover A1 in Gazebo Sim (`gz sim`): the world, the spawned robot with `gz_ros2_control`, the
same controllers, EKFs and twist_mux as the real rover, simulated stand-ins for the sensor
payload (RS16 lidar, RUTX11 GNSS), a ROS–Gazebo bridge and RViz.

The ROS side mirrors what the platform and the sensor containers publish on the rover, so the
orchestrator (`rover_navigation`, `rover_mission_manager`) runs against the simulation unchanged
apart from `use_sim_time`.

## Building

```bash
export ROVER_ROS_BUILD_TYPE=simulation
sudo apt install ros-$ROS_DISTRO-gz-ros2-control ros-$ROS_DISTRO-ros-gz-bridge   # or rosdep
colcon build --symlink-install --packages-up-to rover_metapackage rover_autonomy \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

Run the simulation on a local middleware: an inherited `ZENOH_CONFIG_OVERRIDE` pointing at the
rover's router would join the simulated nodes to the real rover.

## Launch Files

- `simulation.launch.py` - entry point. It starts:
  - `rover_world`'s `rover_world.launch.py` (Gazebo with `config/teleop.config` as the GUI layout)
  - `rover_description`'s `rover_rviz.launch.py` (unless `use_rviz:=False`)
  - `include/simulate_robot.launch.py`

  Everything runs with `use_sim_time`.
- `include/simulate_robot.launch.py` - per-robot part:
  - `include/spawn_robot.launch.py` spawns the URDF at `x`/`y`/`z`/`roll`/`pitch`/`yaw`
    (default `0, -2.0, 0.2`). The model is named after the namespace, else `rover_a1`.
  - `rover_controller` with `use_sim:=True`, so `gz_ros2_control` owns the controller manager.
  - `rover_localization` with `use_sim:=True use_ekf:=True`; `fuse_gps` follows `use_gps`, which
    adds `rover_gps_heading_node`, `rover_navsat_transform_node` and `rover_ekf_global_node`.
  - `rover_twist_mux` (mux + `rover_motion_lock_node`), as on the rover:
    `nav_cmd_vel_stamped` / teleop → `cmd_vel`, gated by `motion_lock`.
  - `sim_gpio_state_publisher` (`scripts/sim_gpio_state.py`): an all-clear
    `hardware_interface/gpio_state`, which `rover_motion_lock_node` needs to open the lock.
    `ros2 param set <ns>/sim_gpio_state_publisher e_stop true` simulates a software E-Stop.
  - `gz_bridge` (`ros_gz_bridge/parameter_bridge`) configured by `config/gz_bridge.yaml`.
  - `rover_rs16_lidar_scan` (`pointcloud_to_laserscan`): `scan` sliced from `rslidar_points`
    with the real driver's settings (±0.25 m, 360° at 0.5°, 0.2–20 m).
  - `static_tf_publisher`: an optional `world → <namespace>/odom` transform at the spawn pose
    (`add_world_transform:=True`).

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of the robot's nodes and topics. |
| `use_gps` | `$ROVER_USE_GPS`, else `false` | Fuse the simulated GNSS (dual EKF + navsat_transform). |
| `publish_global_tf` | `$ROVER_GPS_PUBLISH_MAP_TF`, else `false` | `rover_ekf_global_node` broadcasts `map → odom`. |
| `use_rviz` | `True` | Start RViz (`simulation.launch.py`). |
| `gz_gui` | `config/teleop.config` | Gazebo GUI layout; `{namespace}` in the file is replaced with `namespace`. |
| `gz_headless_mode` | `False` | Run Gazebo server-only with headless rendering (`rover_world`). |
| `log_level` | `INFO` | Logging level. |

`rover_safety`, `rover_led`, `rover_battery`, `rover_crsf_teleop` and `rover_diag_manager` are
not started in simulation.

## Simulated sensors

Defined in `rover_description` (`urdf/common/lidar.urdf.xacro`, `gps.urdf.xacro`), sim only.
Frames carry the namespace prefix, like `robot_state_publisher`'s TF.

| Sensor | Stand-in for | ROS topic | Frame |
|--------|--------------|-----------|-------|
| `gpu_lidar`, 16 rings ±15°, 360° at 0.5°, 0.2–20 m, 10 Hz | `rover_rs16_lidar` | `<ns>/rslidar_points` (`PointCloud2`), `<ns>/scan` (`LaserScan`) | `<ns>/lidar_link` |
| `navsat`, 5 Hz, σ 5 cm | `rover_gps` (RUTX11) | `<ns>/gps/fix` (`NavSatFix`) | `<ns>/gps_link` |
| `imu`, 50 Hz | Phidget IMU | `<ns>/imu/data` (via `rover_imu_broadcaster`) | `<ns>/imu_link` |

In simulation only, `body_link` gets a box collision over the `base.stl` bounds, so the body
collides with obstacles. The hardware URDF keeps visual-only geometry.

The lidar uses the `ROVER_LIDAR_*` mount pose; with none set it sits 0.45 m above `body_link`,
high enough that its lowest ring (-15°) clears the body. A custom mount lower than about
0.37 m makes the rover see itself. The GNSS uses `ROVER_GPS_*`. The world's `<spherical_coordinates>` sets
the datum (50.088384 N, 19.939128 E).

## Config Files

- `gz_bridge.yaml` - bridged topics. `<namespace>` is substituted at launch.

  | ROS topic | Gazebo topic | Direction |
  |-----------|--------------|-----------|
  | `/clock` | `/clock` | Gazebo → ROS |
  | `/<namespace>/teleop_foxglove_cmd_vel_stamped` (`TwistStamped`) | `/<namespace>/teleop_gz_cmd_vel` (`gz.msgs.Twist`) | Gazebo → ROS (GUI teleop into twist_mux) |
  | `/<namespace>/rslidar_points` (`PointCloud2`) | `/<namespace>/lidar/points` | Gazebo → ROS |
  | `/<namespace>/gps/fix` (`NavSatFix`) | `/<namespace>/gps/fix` (`gz.msgs.NavSat`) | Gazebo → ROS |

- `teleop.config` - Gazebo GUI layout with a teleop panel that publishes to
  `{namespace}/teleop_gz_cmd_vel`.

## Running

`scripts/rover_sim.sh` (installed as `lib/rover_gazebo/rover_sim.sh`) runs the simulation from a
fresh terminal, whatever `~/.bashrc` set up for the rover. It sets
`ROVER_ROS_BUILD_TYPE=simulation` and `ROVER_NAMESPACE` (default `rover`), drops
`ZENOH_CONFIG_OVERRIDE`, and starts a zenoh router bound to `127.0.0.1:7447`, which it stops on
exit. It also checks that the apt packages are installed. Run as-is, it starts the simulation;
sourced, it only prepares the current shell, e.g. for a second terminal:

```bash
# terminal 1: simulation (--build builds the workspace first; extra args go to the launch file)
~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh --build
ROVER_USE_GPS=true ~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh use_rviz:=False

# terminal 2: orchestrator / ros2 CLI on the same local middleware
source ~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh
ros2 launch rover_navigation bringup.launch.py use_sim_time:=True localization_source:=slam
```

It uses rmw_zenoh because FastDDS inter-process discovery does not work on the WSL host.

Manually, with the environment already set up:

```bash
ros2 launch rover_gazebo simulation.launch.py
ROVER_NAMESPACE=rover ROVER_USE_GPS=true ros2 launch rover_gazebo simulation.launch.py use_rviz:=False

# then the orchestrator, e.g.
ros2 launch rover_navigation bringup.launch.py use_sim_time:=True localization_source:=slam
```

With `ROVER_USE_GPS=true`, `gps/heading_imu` (and so `odometry/gps` / `odometry/global`) only
appears after the rover has driven straight for a few metres, exactly as on the rover.
