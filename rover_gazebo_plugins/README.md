# rover_gazebo_plugins

Gazebo Sim (Jetty, gz-gui 10 / Qt 6) plugins for the Rover A1 simulation. They are built only
for simulation: `rover_gazebo` depends on this package when `ROVER_ROS_BUILD_TYPE=simulation`.

## RoverSafetyPanel

The **Rover Safety** panel is docked in `rover_gazebo/config/teleop.config`. It is the operator
side of `rover_gazebo`'s `sim_safety_plc`, which stands in for the safety PLC and models its
set-dominant E-Stop latch.

| Button | Real rover counterpart |
|--------|------------------------|
| HW E-STOP | Physical E-Stop button. Maintained: click to press, click again to release. Releasing it also resets the latch, standing in for the HW reset button |
| SW E-STOP | `hardware_interface/sw_user_e_stop_set` |
| SW RESET | `hardware_interface/sw_user_e_stop_reset`; refused while the wheels turn |
| RESET LATCH | `hardware_interface/sw_e_stop_latch_reset`; no effect while a stop is still asserted |

The lamps show the SW E-Stop coil, the latch, the contactor and `motion_lock`. They go grey when
no state has arrived for 2 s. The line under the lamps is the outcome of the last request.

The plugin uses gz-transport only, with no rclcpp in the Gazebo GUI process. `rover_gazebo/config/gz_bridge.yaml`
maps these topics to ROS:

| gz topic | Type | Direction |
|----------|------|-----------|
| `/<ns>/sim_safety/hw_e_stop_button` | `gz.msgs.Boolean` | panel → ROS. Published on change and every second |
| `/<ns>/sim_safety/sw_e_stop_set`, `sw_e_stop_reset`, `latch_reset` | `gz.msgs.Empty` | panel → ROS |
| `/<ns>/sim_safety/sw_e_stop`, `latch_active`, `contactor_engaged` | `gz.msgs.Boolean` | ROS → panel |
| `/<ns>/motion_lock` | `gz.msgs.Boolean` | ROS → panel |
| `/<ns>/sim_safety/result` | `gz.msgs.StringMsg` | ROS → panel |

Configuration in the GUI config:

```xml
<plugin filename="RoverSafetyPanel" name="Rover Safety">
  <namespace>{namespace}</namespace>
</plugin>
```

An environment hook adds this package's `lib/` to `GZ_GUI_PLUGIN_PATH`, so `gz sim` finds the
plugin after `source install/setup.bash`.

## RosContextShutdown

A gz-sim **system** plugin with no behaviour of its own: when the server tears its systems down,
it calls `rclcpp::shutdown()`.

`gz_ros2_control` calls `rclcpp::init()` in the Gazebo process but never `rclcpp::shutdown()`, so
the default context is otherwise shut down by its static destructor inside `exit()`. Under
`rmw_zenoh_cpp` that runs after Zenoh's Tokio thread-local storage is destroyed, and the process
aborts on every Ctrl+C:

```
thread '<unnamed>' panicked at .../zenoh-runtime/src/lib.rs:154:21:
The Thread Local Storage inside Tokio is destroyed. ...
[gazebo-1] Aborted
[ERROR] [launch]: Caught exception in launch (see debug for traceback): Cannot shutdown a ROS adapter that is not running
```

rmw_zenoh lists this under "Known issues": whoever initialises the context has to shut it down
before the process exits.

`rover_description/urdf/common/gazebo_system.urdf.xacro` loads it in the same `<gazebo>` block,
**after** `gz_ros2_control`, so the controller manager is stopped first:

```xml
<plugin filename="RosContextShutdown" name="rover_gazebo_plugins::RosContextShutdown" />
```

With several models each instance holds a reference and the last one destroyed shuts the context
down. A second hook adds this package's `lib/` to `GZ_SIM_SYSTEM_PLUGIN_PATH`.
