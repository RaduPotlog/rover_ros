# rover_gazebo

Runs Rover A1 in Gazebo Sim (`gz sim`): the world, the spawned robot with `gz_ros2_control`, the
same controllers and EKF as the real rover, a ROS–Gazebo bridge and RViz.

## Launch Files

- `simulation.launch.py` - entry point. It starts:
  - `rover_world`'s `rover_world.launch.py` (Gazebo with `config/teleop.config` as the GUI layout)
  - `rover_description`'s `rover_rviz.launch.py` (unless `use_rviz:=False`)
  - `include/simulate_robot.launch.py`

  Everything runs with `use_sim_time`.
- `include/simulate_robot.launch.py` - per-robot part:
  - `include/spawn_robot.launch.py` spawns the URDF at `x`/`y`/`z`/`roll`/`pitch`/`yaw`
    (default `0, -2.0, 0.2`).
  - `rover_controller` with `use_sim:=True`, so `gz_ros2_control` owns the controller manager.
  - `rover_localization` with `use_sim:=True use_ekf:=True`.
  - `gz_bridge` (`ros_gz_bridge/parameter_bridge`) configured by `config/gz_bridge.yaml`.
  - `static_tf_publisher`: a static `world → <namespace>/odom` transform at the spawn pose.

| Argument (`simulation.launch.py`) | Default | Description |
|-----------------------------------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of the robot's nodes and topics. |
| `use_rviz` | `True` | Start RViz. |
| `gz_gui` | `config/teleop.config` | Gazebo GUI layout; `{namespace}` in the file is replaced with `namespace`. |
| `log_level` | `INFO` | Logging level. |

`rover_safety`, `rover_led`, `rover_twist_mux`, `rover_battery`, `rover_crfs_teleop` and
`rover_diag_manager` are not started in simulation. Velocity commands go straight to `cmd_vel`.

## Config Files

- `gz_bridge.yaml` - bridged topics. `<namespace>` is substituted at launch.

  | ROS topic | Gazebo topic | Direction |
  |-----------|--------------|-----------|
  | `/clock` | `/clock` | Gazebo → ROS |
  | `/<namespace>/cmd_vel` (`TwistStamped`) | same (`gz.msgs.Twist`) | Gazebo → ROS (the GUI teleop plugin drives the controller) |
  | `/<namespace>/scan` (`LaserScan`) | `/lidar` | Gazebo → ROS |

- `teleop.config` - Gazebo GUI layout with a teleop panel that publishes to `{namespace}/cmd_vel`.

## Running

```bash
ros2 launch rover_gazebo simulation.launch.py
ROVER_NAMESPACE=rover ros2 launch rover_gazebo simulation.launch.py use_rviz:=False
```

The package hook adds its `lib/` to `GZ_GUI_PLUGIN_PATH` and `GZ_GAZEBO_SYSTEM_PLUGIN_PATH`.
