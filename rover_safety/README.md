# rover_safety

Behavior-tree driven safety supervision for the rover. Two lifecycle nodes feed rover state
into BehaviorTree.CPP trees and act on it:

- `rover_safety_node` trips the software E-Stop when the battery is unsafe.
- `rover_led_safety_node` chooses the LED animations (E-Stop, ready, manual drive, battery,
  error) and requests them from `rover_led`.

Both are `nav2::LifecycleNode`s that configure and activate themselves (`autostart_node`).

## rover_safety_node

| Direction | Name | Type |
|-----------|------|------|
| sub | `rover_battery/battery_status` | `sensor_msgs/BatteryState` (latest-state QoS) |
| sub | `hardware_interface/rover_driver_state` | `rover_msgs/RoverDriverState` |
| sub | `hardware_interface/gpio_state` | `rover_msgs/GpioState` |
| sub | `system_status` | `rover_msgs/SystemStatus` (from `rover_diag_manager`, CPU temperature) |
| client | `hardware_interface/sw_user_e_stop_set` | `std_srvs/Trigger` (called from the tree) |
| pub | `diagnostics` | hardware id `Rover Safety`: `Safety inputs`, `Battery safety verdict`, `Safety behavior tree` |

The battery reaction is decided outside the tree by the pure
`domain::evaluateBatterySafety()` (`include/rover_safety/domain/battery_safety_policy.hpp`):

| Battery reading | Verdict |
|-----------------|---------|
| health `WATCHDOG_TIMER_EXPIRE`, `DEAD` or `OVERVOLTAGE` | `TRIP_E_STOP` |
| health `OVERHEAT` and temperature above `battery.temp.fatal` | `SHUTDOWN` |
| health `OVERHEAT` and temperature above `battery.temp.critical` | `TRIP_E_STOP` |
| anything else | none |

The verdict is written to the blackboard (`battery_verdict`, `battery_verdict_reason`) and the
`RoverSafety` tree (`behavior_trees/rover_safety.xml`) acts on it at `timer_frequency`. It calls
`hardware_interface/sw_user_e_stop_set` for `TRIP_E_STOP` (unless the E-Stop is already set)
and runs `SignalShutdown` for `SHUTDOWN`.

The node is not started with `use_sim:=True`.

## rover_led_safety_node

| Direction | Name | Type |
|-----------|------|------|
| sub | `rover_battery/battery_status` | `sensor_msgs/BatteryState` |
| sub | `hardware_interface/gpio_state` | `rover_msgs/GpioState` (`gpio_pin_hw_e_stop_user_button` → `e_stop_state`) |
| sub | `joy` | `sensor_msgs/Joy` (dead-man button → `drive_state`) |
| client | `led/set_animation` | `rover_msgs/SetLedAnimation` (called from the tree) |
| pub | `diagnostics` | hardware id `Bumper Led`: `LED safety inputs`, `LED safety behavior tree` |

The `RoverLedSafety` tree (`behavior_trees/rover_led_safety.xml`) runs three subtrees in parallel:

- `ErrorAnimationSubtree` shows `ERROR` while the battery status is unknown or it is overheating
  while charging, and `NO_ERROR` otherwise.
- `BatteryAnimationSubtree`:
  - While charging, it shows `CHARGING_BATTERY`, updated every `battery.charging_anim_step`,
    or `BATTERY_CHARGED` at 100 %.
  - While discharging, it repeats `LOW_BATTERY` every `battery.anim_period.low` seconds below
    `battery.percent.threshold.low`, shows `CRITICAL_BATTERY` below
    `battery.percent.threshold.critical`, and `BATTERY_NOMINAL` otherwise.
- `StateAnimationSubtree` shows `E_STOP` while the hardware E-Stop button is pressed, otherwise
  `MANUAL_ACTION` while the dead-man button is held and `READY` when idle.

Animation ids are the `rover_msgs/LedAnimation` constants.

The node also runs in simulation.

## Behavior trees

| File | Tree | Used by | Groot2 port (`bt_server_port`) |
|------|------|---------|--------------------------------|
| `behavior_trees/RoverSafetyBT.btproj` → `rover_safety.xml` | `RoverSafety` | `rover_safety_node` | 6666 |
| `behavior_trees/RoverLedSafetyBT.btproj` → `rover_led_safety.xml` | `RoverLedSafety` | `rover_led_safety_node` | 5555 |

Open the `.btproj` files in Groot2 to edit the trees, or connect Groot2 to the port above to
watch a running tree. The trees load these BT plugin libraries (`plugin_libs` /
`ros_plugin_libs`):

| Library | Node | Kind |
|---------|------|------|
| `call_trigger_service_bt_node` | `CallTriggerService` | action, `std_srvs/Trigger` client |
| `call_set_bool_service_bt_node` | `CallSetBoolService` | action, `std_srvs/SetBool` client |
| `call_set_led_animation_service_bt_node` | `CallSetLedAnimationService` | action, `rover_msgs/SetLedAnimation` client |
| `execute_command_bt_node` | `ExecuteCommand` | action, runs a shell command |
| `signal_shutdown_bt_node` | `SignalShutdown` | action, sets the `signal_shutdown` blackboard entry |
| `tick_after_timeout_bt_node` | `SafetyBtTickAfterTimeout` | decorator, ticks its child at most once per `timeout` s |

Service names in the trees are relative (e.g. `led/set_animation`), so they follow the node's
namespace.

## Config Files

- `config/rover_safety.yaml` - `rover_safety_node` parameters (battery temperature limits, tick
  rate, Groot port, plugin list).
- `config/led_safety.yaml` - `rover_led_safety_node` parameters (battery thresholds and animation
  timing, tick rate, Groot port, plugin list).

Every parameter, with its default and validation, is declared in `src/safety_parameters.yaml`
and `src/led_safety_parameters.yaml` (`generate_parameter_library`). `input_timeout` sets when a
periodic input is reported stale on diagnostics.

## Launch Files

- `rover_safety.launch.py` - starts `rover_led_safety_node` and, unless `use_sim:=True`,
  `rover_safety_node`.

| Argument | Default | Description |
|----------|---------|-------------|
| `namespace` | `$ROVER_NAMESPACE`, else empty | Namespace of both nodes. |
| `use_sim` | `False` | Skip `rover_safety_node` in simulation. |
| `log_level` | `INFO` | Logging level. |
| `led_bt_project_path` | `behavior_trees/RoverLedSafetyBT.btproj` | LED safety tree project. |
| `safety_bt_project_path` | `behavior_trees/RoverSafetyBT.btproj` | Safety tree project. |
| `common_dir_path` | empty | If set, the default `led_bt_project_path` is taken from `<common_dir_path>/rover_safety/behavior_trees/`. |

```bash
ros2 launch rover_safety rover_safety.launch.py
ros2 lifecycle get /rover/rover_safety_node
```

## Layout

```
domain/          battery_safety_policy (verdict from battery health + temperature),
                 safety_health (input freshness) - no ROS dependencies
infrastructure/  safety_diagnostics (diagnostic_updater status mapping)
safety_node / led_safety_node   lifecycle ROS adapters: subscriptions -> blackboard, tree timer
behavior_tree.hpp               tree loading, plugin registration, Groot2 publisher
plugins/         BT action and decorator nodes listed above
```

## Tests

```bash
colcon test --packages-select rover_safety && colcon test-result --all --verbose
```

- `test/unit/` covers the battery safety policy, input health and diagnostics mapping.
- `test/integration/test_call_trigger_service.cpp` runs the `CallTriggerService` BT node
  against a real service.

## Known limitations

- `SHUTDOWN` only sets the `signal_shutdown` blackboard entry. Nothing reads it yet, so an
  over-temperature battery beyond `battery.temp.fatal` has no effect beyond the verdict shown on
  diagnostics.
- `cpu.temp.*`, `driver.temp.*`, `fan_turn_off_timeout` and `shutdown_hosts_path` are declared
  but not used: fan control and driver temperature handling are not implemented (see the TODO in
  `src/safety_node.cpp`).
- Nothing in this workspace publishes `joy`, so `MANUAL_ACTION` is never shown unless a joystick
  driver is added.
