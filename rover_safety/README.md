# rover_safety

Behavior-tree driven safety supervision for the rover. Two lifecycle nodes feed rover state
into BehaviorTree.CPP trees and act on it:

- `rover_safety_node` trips the software E-Stop when the battery is unsafe, and shuts down the
  **ROS controller** (the computer running the rover ROS 2 stack) when asked to.
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
| client | `hardware_interface/sw_user_e_stop_set` | `std_srvs/Trigger` (called from the trees) |
| service | `~/shutdown` | `std_srvs/Trigger`: shuts down the ROS controller (`shutdown.service_enabled`) |
| pub | `diagnostics` | hardware id `Rover Safety`: `Safety inputs`, `Battery safety verdict`, `Safety behavior tree`, `Shutdown` |

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
and runs `SignalShutdown` for `SHUTDOWN`, which starts the shutdown below.

The node is not started with `use_sim:=True`.

### Shutting down the ROS controller

Mirrors `husarion_ugv_manager`'s safety manager. A shutdown starts when:

- the `RoverSafety` tree runs `SignalShutdown` (battery `SHUTDOWN` verdict), or
- anything calls `~/shutdown` (`std_srvs/Trigger`), e.g.
  `ros2 service call /rover/rover_safety_node/shutdown std_srvs/srv/Trigger {}`.

The node then halts the safety tree and ticks the `RoverShutdown` tree
(`behavior_trees/rover_shutdown.xml`) at `timer_frequency` instead:

1. `CallTriggerService` trips the E-Stop (`hardware_interface/sw_user_e_stop_set`), best effort.
2. `ShutdownHostsFromFile` asks every host in `shutdown_hosts_path` to shut down, best effort
   (see [Remote hosts](#remote-hosts)).
3. `ExecuteCommand` runs `shutdown.command`, by default the installed
   `shutdown_ros_controller.sh --no-e-stop`, with the reason exported as `ROVER_SHUTDOWN_REASON`.
   It must finish within `shutdown.command_timeout`.

The tree runs without blocking the executor. The service replies right away:

| Situation | `success` | `message` |
|-----------|-----------|-----------|
| No shutdown yet | `true` | `Shutdown started.` |
| Shutdown tree running | `false` | `Shutdown already in progress.` |
| Power-off command accepted | `false` | `Shutdown already requested; powering off.` |
| Last attempt failed less than `shutdown.retry_backoff` s ago | `false` | `Previous shutdown failed; ...` |

Once the power-off command succeeds, the node keeps the safety tree halted while the computer goes
down. If it fails, the `Shutdown` diagnostic turns ERROR, the safety tree resumes, and a new request
(service or battery verdict) retries after `shutdown.retry_backoff`. The rules live in
`domain::ShutdownSequence`.

### `shutdown_ros_controller.sh`

Powers off this computer. The shutdown tree runs it, and it also works on its own, e.g. when the
ROS stack is down:

```bash
ros2 run rover_safety shutdown_ros_controller.sh --reason "Maintenance"
ros2 run rover_safety shutdown_ros_controller.sh --dry-run   # print what it would do
```

It trips the E-Stop (`/$ROVER_NAMESPACE/hardware_interface/sw_user_e_stop_set`, skipped with
`--no-e-stop`; a failure does not stop the power-off). Then it uses the first power-off method that
works:

| Method | Used when |
|--------|-----------|
| balena Supervisor API `POST /v1/shutdown` | `BALENA_SUPERVISOR_ADDRESS` and `BALENA_SUPERVISOR_API_KEY` are set (label `io.balena.features.supervisor-api`) |
| systemd-logind `PowerOff` over D-Bus (`dbus-send`) | a system bus is reachable; `/host/run/dbus/system_bus_socket` is used when present (label `io.balena.features.dbus`) |
| `systemctl poweroff` | fallback on a regular Linux host |

Options: `--reason TEXT`, `--namespace NS`, `--no-e-stop`, `--dry-run`, `-h`. It exits 0 once a
power-off request is accepted, 1 when every method failed, and 2 on bad usage.

### Remote hosts

`config/shutdown_hosts.yaml` (launch argument `shutdown_hosts_config_path`) lists hosts to shut
down before the ROS controller. It uses the husarion format:

```yaml
hosts:
  - ip: 192.168.77.201   # required
    port: 3003           # default 3003
    secret: change-me    # HMAC-SHA256 key
    timeout: 10.0        # seconds to wait for the host to stop answering ping, default 5.0
```

Each host must serve `GET /shutdown?ts=<unix seconds>&sig=<hex HMAC-SHA256(secret, "/shutdown|<ts>")>`,
answer 200 and power off. Unreachable hosts are skipped. The file ships with no hosts.

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
| `behavior_trees/RoverSafetyBT.btproj` → `rover_shutdown.xml` | `RoverShutdown` | `rover_safety_node` | 7777 (`shutdown.bt_server_port`) |
| `behavior_trees/RoverLedSafetyBT.btproj` → `rover_led_safety.xml` | `RoverLedSafety` | `rover_led_safety_node` | 5555 |

Open the `.btproj` files in Groot2 to edit the trees, or connect Groot2 to the port above to
watch a running tree. Each node loads the BT plugin libraries named in its `plugin_libs` and
`ros_plugin_libs` parameters (resolved through the library path), then the project:

| Library | Node | Kind |
|---------|------|------|
| `call_trigger_service_bt_node` | `CallTriggerService` | action, `std_srvs/Trigger` client |
| `call_set_bool_service_bt_node` | `CallSetBoolService` | action, `std_srvs/SetBool` client |
| `call_set_led_animation_service_bt_node` | `CallSetLedAnimationService` | action, `rover_msgs/SetLedAnimation` client |
| `execute_command_bt_node` | `ExecuteCommand` | action, runs a bash command (killed with its children on timeout or halt) |
| `shutdown_hosts_from_file_bt_node` | `ShutdownHostsFromFile` | action, shuts down the hosts listed in a YAML file |
| `signal_shutdown_bt_node` | `SignalShutdown` | action, sets the `signal_shutdown` blackboard entry |
| `tick_after_timeout_bt_node` | `SafetyBtTickAfterTimeout` | decorator, ticks its child at most once per `timeout` s |

Service names in the trees are relative (e.g. `led/set_animation`), so they follow the node's
namespace.

## Config Files

- `config/rover_safety.yaml` - `rover_safety_node` parameters (battery temperature limits, tick
  rate, Groot ports, plugin list, shutdown).
- `config/shutdown_hosts.yaml` - remote hosts to shut down before the ROS controller.
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
| `shutdown_hosts_config_path` | `config/shutdown_hosts.yaml` | Hosts to shut down before the ROS controller. |
| `common_dir_path` | empty | If set, the default `led_bt_project_path` and `shutdown_hosts_config_path` are taken from `<common_dir_path>/rover_safety/`. |

```bash
ros2 launch rover_safety rover_safety.launch.py
ros2 lifecycle get /rover/rover_safety_node
```

## Layout

```
domain/          battery_safety_policy (verdict from battery health + temperature),
                 safety_health (input freshness),
                 shutdown_sequence (shutdown state, idempotent requests, retry backoff) - no ROS dependencies
infrastructure/  safety_diagnostics (diagnostic_updater status mapping),
                 shutdown_command (bash command carrying the shutdown reason),
                 command_handler (bash command in its own process group, watched from a thread),
                 shutdown_host (signed HTTP shutdown request, ping until down)
safety_node / led_safety_node   lifecycle ROS adapters: subscriptions -> blackboard, tree timer
behavior_tree.hpp               tree loading, plugin registration, Groot2 publisher
plugins/         BT action and decorator nodes listed above
scripts/         shutdown_ros_controller.sh
```

## Tests

```bash
colcon test --packages-select rover_safety && colcon test-result --all --verbose
```

- `test/unit/` covers the battery safety policy, input health, shutdown sequence, shutdown command
  quoting, diagnostics mapping, and the `ExecuteCommand`, `SignalShutdown`, `ShutdownHosts*` plugins
  (against a local HTTP server; no root needed).
- `test/integration/test_call_trigger_service.cpp` runs the `CallTriggerService` BT node
  against a real service.
- `test/integration/test_shutdown_tree.cpp` loads `RoverSafetyBT.btproj` with the plugin lists of
  `config/rover_safety.yaml` and runs `RoverShutdown`, including a retry.
- `test/integration/test_safety_node_shutdown.cpp` runs `rover_safety_node` in-process: the
  `~/shutdown` service, retry after a failed power-off, and a fatal battery temperature.
- `test/integration/test_shutdown_script.sh` checks `shutdown_ros_controller.sh` against stub
  `ros2`/`curl`/`dbus-send`/`systemctl` binaries.

## Known limitations

- `cpu.temp.*`, `driver.temp.*`, `battery.temp.window_len` and `fan_turn_off_timeout` are
  declared but not used: fan control, temperature smoothing and driver temperature handling are
  not implemented (see the TODO in `src/safety_node.cpp`).
- Both nodes build their trees at configure time, and the nav2 service BT nodes wait up to 3 s for
  their servers there (`hardware_interface/sw_user_e_stop_set`, `led/set_animation`). If a server is
  missing, configure fails and is not retried automatically; run
  `ros2 lifecycle set <node> configure` once it is up.
- Anyone on the ROS graph can call `~/shutdown`. Set `shutdown.service_enabled: false` to remove it.
- Nothing in this workspace publishes `joy`, so `MANUAL_ACTION` is never shown unless a joystick
  driver is added.
