# Simulation

`rover_gazebo` runs Rover A1 in Gazebo Sim with the same controllers, EKF and command
arbitration as the real rover. Simulated sensors stand in for the sensor payload, so the ROS
graph looks like the rover's apart from `use_sim_time`.

## What the simulation provides

| Part | What runs | Source |
|------|-----------|--------|
| World | `rover_world/world/rover_world.sdf`: 25 m x 25 m ground plane, perimeter walls at ±11 m, four boxes and three pillars around the spawn point. Physics step 0.001 s, real-time factor 1.0. | `rover_world/world/rover_world.sdf` |
| Robot | URDF built with `use_sim:=true`, spawned at x `0.0`, y `-2.0`, z `0.2` (drop height). The model is named after the namespace. | `rover_gazebo/launch/include/simulate_robot.launch.py` |
| ros2_control | `gz_ros2_control/GazeboSimSystem` owns the controller manager. Same controller config as hardware (`wheel_01_controller.yaml`), then `rover_gazebo/config/sim_wheel_pid.yaml` on top. | `rover_description/urdf/common/gazebo_system.urdf.xacro`, `rover_controller/launch/rover_controller.launch.py` |
| Controllers | `rover_drive_controller`, the four wheel PIDs, `rover_joint_state_broadcaster`, `rover_imu_broadcaster`. | `rover_controller/config/wheel_01_controller.yaml` |
| Localization | `rover_ekf_node`; with `use_gps` also `rover_gps_heading_node`, `rover_navsat_transform_node`, `rover_ekf_global_node`. | `rover_localization/launch/rover_localization.launch.py` |
| Command path | `rover_twist_mux_node`, `rover_motion_lock_node`, `rover_command_freshness_node`, as on the rover. | `rover_twist_mux/launch/rover_twist_mux.launch.py` |
| Safety PLC stand-in | `sim_safety_plc`: the E-Stop latch (HW button, SW E-Stop, latch reset), `hardware_interface/safety_status` and `hardware_interface/safety_command_echo` at 5.0 Hz, and the `hardware_interface/sw_*` E-Stop services. Driven by the Gazebo **Rover Safety** panel. | `rover_gazebo/scripts/sim_safety_plc.py`, `rover_gazebo_plugins` |
| Bridge | `gz_bridge` (`ros_gz_bridge/parameter_bridge`). | `rover_gazebo/config/gz_bridge.yaml` |
| Laser scan | `rover_rs16_lidar_scan` (`pointcloud_to_laserscan`): `scan` from `rslidar_points`, height slice ±0.25 m, 360° at 0.008727 rad, 0.2 to 20.0 m. | `rover_gazebo/launch/include/simulate_robot.launch.py` |
| Visualization | RViz (`use_rviz`), Gazebo GUI with a teleop panel and the Rover Safety panel. | `rover_gazebo/launch/simulation.launch.py` |
| World frame | Optional static `world` → `rover/odom` at the spawn pose (`add_world_transform:=True`). | `rover_gazebo/launch/include/simulate_robot.launch.py` |

### Simulated sensors

| Sensor | Stands in for | Gazebo settings | ROS topic | Frame |
|--------|---------------|-----------------|-----------|-------|
| `gpu_lidar` | RoboSense RS16 (sensor payload) | 16 rings over ±0.261799 rad, 720 samples over 360°, 0.2 to 20.0 m, 10 Hz, Gaussian noise σ 0.01 m | `rslidar_points` (`sensor_msgs/PointCloud2`), `scan` (`sensor_msgs/LaserScan`) | `rover/lidar_link` |
| `navsat` | RUTX11 GNSS (sensor payload) | 5 Hz, horizontal noise σ 4.5e-7 deg (about 5 cm), vertical σ 0.1 m | `gps/fix` (`sensor_msgs/NavSatFix`) | `rover/gps_link` |
| `imu` | Phidget IMU | 50 Hz, ENU orientation reference | `imu/data` (`sensor_msgs/Imu`, through `rover_imu_broadcaster`) | `rover/imu_link` |

The world's `<spherical_coordinates>` set the GNSS datum: latitude 50.088384°, longitude
19.939128°, elevation 0 m. The world loads the NavSat, IMU, Contact, Physics, Sensors,
SceneBroadcaster and UserCommands systems.

Source: `rover_description/urdf/common/lidar.urdf.xacro`, `rover_description/urdf/common/gps.urdf.xacro`,
`rover_description/urdf/common/imu.urdf.xacro`, `rover_world/world/rover_world.sdf`.

### Bridged topics

| ROS topic | Gazebo topic | Direction | Notes |
|-----------|--------------|-----------|-------|
| `/clock` | `/clock` | Gazebo → ROS | Simulation time. |
| `teleop_foxglove_cmd_vel_stamped` (`geometry_msgs/TwistStamped`) | `/rover/teleop_gz_cmd_vel` (`gz.msgs.Twist`) | Gazebo → ROS | Gazebo GUI teleop panel into twist_mux (priority 100). |
| `rslidar_points` (`sensor_msgs/PointCloud2`) | `/rover/lidar/points` | Gazebo → ROS | Lidar cloud. |
| `gps/fix` (`sensor_msgs/NavSatFix`) | `/rover/gps/fix` (`gz.msgs.NavSat`) | Gazebo → ROS | GNSS fix. |

Source: `rover_gazebo/config/gz_bridge.yaml`, `rover_gazebo/config/teleop.config`.

## Start the simulation

Install the Gazebo packages and build with `ROVER_ROS_BUILD_TYPE=simulation` (see
[Getting started](overview.md#getting-started)). `rover_sim.sh` checks for `gz_ros2_control`,
`ros_gz_bridge` and `ros_gz_sim` and prints the `apt` command if one is missing.

```bash
# Build the workspace, then start the simulation
~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh --build

# Start with GPS fusion and without RViz
ROVER_SYSTEM_USE_GPS=true ~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh use_rviz:=False

# Second terminal: a shell on the same local middleware, for ros2 CLI or the orchestrator
source ~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh
ros2 topic echo /rover/odom
```

`rover_sim.sh` does this before it launches `rover_gazebo simulation.launch.py`:

1. Sets `ROVER_ROS_BUILD_TYPE=simulation` and `ROVER_SYSTEM_NAMESPACE` (default `rover`).
2. Sets `RMW_IMPLEMENTATION=rmw_zenoh_cpp` and removes `ZENOH_CONFIG_OVERRIDE`,
   `ROS_AUTOMATIC_DISCOVERY_RANGE` and `ROS_STATIC_PEERS`.
3. Sources `/opt/ros/<distro>` and the workspace overlay, and stops the `ros2` CLI daemon.
4. Starts a Zenoh router bound to `127.0.0.1:7447` (or reuses one already listening), and stops
   it on exit.

Extra arguments go to the launch file. Run as a script it starts the simulation; sourced, it
only prepares the current shell.

!!! danger "Keep the simulation off the real rover"
    A shell with the rover's `ZENOH_CONFIG_OVERRIDE` joins every simulated node to the real
    rover's graph, and both use the `/rover` namespace. Start the simulation with `rover_sim.sh`,
    or clear the variable and run a local router before `ros2 launch rover_gazebo simulation.launch.py`.

Source: `rover_gazebo/scripts/rover_sim.sh`, `rover_scripts/README.md`.

### Launch arguments

`ros2 launch rover_gazebo simulation.launch.py [arg:=value ...]`

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_SYSTEM_NAMESPACE`, else empty | Namespace of the robot's nodes and topics. |
| `use_rviz` | `True` | Start RViz. |
| `gz_gui` | `rover_gazebo/config/teleop.config` | Gazebo GUI layout; `{namespace}` in it is replaced. |
| `log_level` | `INFO` | Logging level. |
| `use_gps` | `$ROVER_SYSTEM_USE_GPS`, else `false` | Fuse the simulated GNSS (dual EKF, navsat_transform, GPS heading). |
| `publish_global_tf` | `$ROVER_PLATFORM_GPS_MAP_TF`, else `false` | `rover_ekf_global_node` broadcasts `map → odom`. |
| `x`, `y`, `z`, `roll`, `pitch`, `yaw` | `0.0`, `-2.0`, `0.2`, `0.0`, `0.0`, `0.0` | Spawn pose. |
| `add_world_transform` | `False` | Publish a static `world` → `odom` at the spawn pose. |
| `gz_bridge_config_path` | `rover_gazebo/config/gz_bridge.yaml` | Bridge configuration. |
| `gz_headless_mode` | `False` | Gazebo server only, headless rendering (`rover_world`). |
| `gz_world` | `rover_world/world/rover_world.sdf` | SDF world (`rover_world`). |

`simulation.launch.py` declares the first four. The others are declared by the included
`simulate_robot.launch.py` and `rover_world.launch.py` and are set from the same command line.
`simulation.launch.py` fixes the Gazebo console level at `1`.

Source: `rover_gazebo/launch/simulation.launch.py`, `rover_gazebo/launch/include/simulate_robot.launch.py`,
`rover_world/launch/rover_world.launch.py`.

### E-Stop and latch

The Gazebo GUI docks a **Rover Safety** panel (`rover_gazebo_plugins/RoverSafetyPanel`) above
Teleop. It drives `sim_safety_plc`, which models the safety PLC's set-dominant latch
(`rover_arch/SAFETY_CHAIN.md`):

| Control | Real rover counterpart | Effect in simulation |
|---------|------------------------|----------------------|
| **HW E-STOP** | Physical mushroom button (maintained) and the HW reset button | Click to press, click again to release. Pressing sets the latch; reported as `hw_e_stop_user_button`. Releasing it resets the latch, like the HW reset button, unless the SW coil is still set |
| **SW E-STOP** | `hardware_interface/sw_user_e_stop_set` | Sets the SW user E-Stop coil and the latch |
| **SW RESET** | `hardware_interface/sw_user_e_stop_reset` | Releases the SW coil. Refused while any wheel turns faster than 0.05 rad/s. The latch stays set |
| **RESET LATCH** | `hardware_interface/sw_e_stop_latch_reset` | Clears the latch, unless the HW button is pressed or the SW coil is set |

To drive again after **SW E-STOP**, press SW RESET then RESET LATCH. After **HW E-STOP**,
release it; that also resets the latch. The panel lamps show the SW coil, the latch, the contactor
and `motion_lock`. They turn grey when `sim_safety_plc` stops answering. The line below the lamps
shows the outcome of the last request.

While the latch is set, the contactor reads open. `rover_motion_lock_node` then closes
`motion_lock`, and twist_mux stops every input. Standing in for the unpowered motors,
`sim_safety_plc` also publishes zero commands on `cmd_vel` at 5.0 Hz.

The same three `std_srvs/Trigger` services as on the rover are served, so the drive UI and the CLI
share the panel's state:

```bash
ros2 service call /rover/hardware_interface/sw_user_e_stop_set std_srvs/srv/Trigger
ros2 service call /rover/hardware_interface/sw_user_e_stop_reset std_srvs/srv/Trigger
ros2 service call /rover/hardware_interface/sw_e_stop_latch_reset std_srvs/srv/Trigger
```

The panel talks gz-transport and `gz_bridge` maps it to ROS (`/rover/sim_safety/*`). The latch
starts clear (`latch_set_at_startup` `false`). The real PLC starts latched.

Source: `rover_gazebo/scripts/sim_safety_plc.py`, `rover_gazebo/scripts/sim_safety_plc_model.py`,
`rover_gazebo_plugins/src/rover_safety_panel/`, `rover_gazebo/config/gz_bridge.yaml`,
`rover_gazebo/config/teleop.config`.

## Differences from the real rover

| Area | Real rover | Simulation |
|------|------------|------------|
| Hardware plugin | `rover_hardware_interface/RoverA1System` and `PhidgetImuSensor` | `gz_ros2_control/GazeboSimSystem`; the IMU interfaces are part of it |
| Safety PLC and E-Stop | Modbus TCP to the PLC; `hardware_interface/*` services; hardware E-Stop button and relay | `sim_safety_plc` models the latch, HW button and SW E-Stop, and serves the three `hardware_interface/sw_*` E-Stop services. The latch starts clear. There is no watchdog, motor-driver-fault coil or welded-contactor check. `aux_output_*/set`, `aux_io_state` and `rover_driver_state` do not exist |
| Nodes not started | | `rover_safety`, `rover_led`, `rover_battery`, `rover_crsf_teleop`, `rover_diag_manager`. The web bridges are not part of the simulation launch |
| Battery | BMS over UDP to `rover_battery`, `battery/battery_status` | Gazebo `LinearBatteryPlugin` (below). Not bridged to ROS: no battery topic |
| Wheel PID gains | `d` `0.04` | `d` `0.0` (`sim_wheel_pid.yaml`): Gazebo wheels have no encoder or motor lag, and the D term made the loop oscillate |
| Wheel PID integral reference | `integral_reference_delay` `0.15` s, `integral_reference_time_constant` `0.08` s | Both `0.0` (`sim_wheel_pid.yaml`): the model stands for the real motors' dead time, which Gazebo doesn't have, so the integral works on the plain error |
| Drive `wheel_radius` | `0.1651` m (tuned rolling radius) | `0.1699` m (CAD tyre radius of the simulated wheel cylinders) |
| Body collision | Visual mesh only | Two boxes over the body and front arch, so the body hits obstacles |
| IMU mount | `ROVER_SYSTEM_MOUNT_IMU_*` variables, default `-0.09 0.0 0.2` m, roll `3.14159` rad | Fixed at `-0.09 0.0 0.2` m, rpy `0 0 0` |
| Lidar mount | `ROVER_SYSTEM_MOUNT_LIDAR_*` variables (lidar driver in `rover_sensors`) | `ROVER_SYSTEM_MOUNT_LIDAR_*` if set, else `0.68` m above `body_link` |
| GNSS and lidar drivers | Sensor payload container (`rover_sensors`) | Gazebo sensors through the bridge |
| Time | System clock | `use_sim_time`, `/clock` from Gazebo |
| Middleware | Zenoh router in the rover's router container | Local Zenoh router on `127.0.0.1:7447` |

Source: `rover_gazebo/launch/include/simulate_robot.launch.py`, `rover_gazebo/config/sim_wheel_pid.yaml`,
`rover_controller/config/wheel_01_controller.yaml`, `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro`,
`rover_description/urdf/rover_a1/base.urdf.xacro`, `rover_description/launch/rover_load_urdf.launch.py`,
`rover_gazebo/config/gz_bridge.yaml`.

### Simulated battery

The URDF adds Gazebo's `LinearBatteryPlugin` in simulation only. It runs inside Gazebo; nothing
bridges it to ROS, and `rover_battery` is not started.

| Setting | Value |
|---------|-------|
| `capacity` | 40.0 Ah |
| `initial_charge_percentage` | 0.7 |
| `voltage` | 41.4 V |
| `charging_time` | 8.0 h |
| `power_load` | 160.0 W (the xacro divides it by 100 as a workaround for gz-sim issue 225) |
| `simulate_discharging` | `false` |

Source: `rover_description/config/battery.yaml`, `rover_description/urdf/common/battery.urdf.xacro`.

## Running autonomy against the simulation

The orchestrator (Nav 2, mission manager) lives in `rover_orchestrator` and runs unchanged
against the simulation with `use_sim_time:=True`. Its documentation covers how to start it.

More: [rover_gazebo README](https://github.com/RaduPotlog/rover_ros/blob/master/rover_gazebo/README.md),
[rover_world README](https://github.com/RaduPotlog/rover_ros/blob/master/rover_world/README.md),
[rover_description README](https://github.com/RaduPotlog/rover_ros/blob/master/rover_description/README.md).
