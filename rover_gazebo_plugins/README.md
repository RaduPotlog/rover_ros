# rover_gazebo_plugins

Gazebo Sim (Jetty, gz-gui 10 / Qt 6) GUI plugins for the Rover A1 simulation. They are built only
for simulation: `rover_gazebo` depends on this package when `ROVER_ROS_BUILD_TYPE=simulation`.

## RoverSafetyPanel

The **Rover Safety** panel is docked in `rover_gazebo/config/teleop.config`. It is the operator
side of `rover_gazebo`'s `sim_safety_plc`, which stands in for the safety PLC and models its
set-dominant E-Stop latch.

| Button | Real rover counterpart |
|--------|------------------------|
| HW E-STOP | Physical E-Stop button. Maintained: click to press, click again to release |
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
