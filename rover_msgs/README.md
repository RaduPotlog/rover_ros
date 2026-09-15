# rover_msgs

Custom messages and services of the Rover A1 stack.

## Messages

| Message | Content | Used by |
|---------|---------|---------|
| `ChargingStatus` | header, charging flag, total and battery current, charger type (`UNKNOWN`/`WIRED`/`WIRELESS`) | `rover_battery` → `rover_battery/charging_status` |
| `GpioState` | safety controller pins: HW/SW E-Stop buttons, motor contactor, CPU watchdog heartbeat, driver fault, latch status/reset | `rover_hardware_interface` → `hardware_interface/gpio_state` |
| `RoverDriverState` | header, `DriverStateNamed[]`, overall `error` | `rover_hardware_interface` → `hardware_interface/rover_driver_state` |
| `DriverStateNamed` | driver name (`default`/`front`/`rear`) + `DriverState` | inside `RoverDriverState` |
| `DriverState` | current, temperature, `FaultFlag`, `RuntimeError`, data timeout flags | inside `DriverStateNamed` |
| `FaultFlag` | driver fault bits `emergency_stop`, `motor_setup_fault` | inside `DriverState` |
| `RuntimeError` | driver runtime error bit `safety_stop_active` | inside `DriverState` |
| `SystemStatus` | CPU usage per core, CPU temperature, load, RAM and disk usage | `rover_diag_manager` → `system_status` |
| `LedAnimation` | animation `id` (named constants `E_STOP` … `BLINKER_RIGHT`) + `param` | `SetLedAnimation` request |
| `LedAnimationCatalog` / `LedAnimationInfo` | loaded animations: id, name, priority layer | `rover_led` → `led/animations` |
| `LedState` / `LedSegmentState` / `LedLayerState` | what every priority layer (`ERROR`/`ALERT`/`INFO`/`STATE`) of every segment plays | `rover_led` → `led/state` |
| `LedImageAnimation` | image, duration, brightness, repeat, colour | `SetLedImageAnimation` request |
| `LedAnimationQueue` | list of queued animation names | not used |

## Services

| Service | Request → Response | Served by |
|---------|--------------------|-----------|
| `SetLedAnimation` | `LedAnimation animation`, `bool repeating` → `success`, `message` | `rover_led` (`led/set_animation`) |
| `SetLedBrightness` | `float32 data` (0–1) → `success`, `message` | `rover_led` (`led/set_brightness`) |
| `SetLedImageAnimation` | front/rear `LedImageAnimation`, `interrupting`, `repeating` → `success`, `message` | not implemented by any node |

```bash
ros2 interface show rover_msgs/msg/GpioState
```
