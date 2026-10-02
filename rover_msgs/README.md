# rover_msgs

Custom messages and services of the Rover A1 stack.

## Messages

| Message | Content | Used by |
|---------|---------|---------|
| `ChargingStatus` | header, charging flag, total and battery current, charger type (`UNKNOWN`/`WIRED`/`WIRELESS`) | `rover_battery` → `rover_battery/charging_status` |
| `SafetyStatus` | plant state of the safety chain: HW E-Stop button, motor contactor aux-contact feedback, latch, latch cause, link health | `rover_hardware_interface` → `hardware_interface/safety_status` |
| `SafetyCommandEcho` | read-backs of the safety coils software drives: SW E-Stop, driver-fault stop, latch-reset pulse, CPU watchdog heartbeat. Diagnostic; may only ever inhibit, never permit | `rover_hardware_interface` → `hardware_interface/safety_command_echo` |
| `AuxIoState` | general-purpose aux IO on the safety PLC (DIO06..11 inputs, read-back of DIO00..05 outputs), with sample time and link health. Not safety - never gate motion on it | `rover_hardware_interface` → `hardware_interface/aux_io_state` |
| `RoverDriverState` | header, `DriverStateNamed[]`, overall `error` | `rover_hardware_interface` → `hardware_interface/rover_driver_state` |
| `DriverStateNamed` | driver name + `DriverState`. The hardware interface fills `rear_left`/`rear_right`/`front_left`/`front_right`; the `NAME_DEFAULT`/`NAME_FRONT`/`NAME_REAR` constants in the message are unused | inside `RoverDriverState` |
| `DriverState` | current, temperature, `FaultFlag`, `RuntimeError`, data timeout flags | inside `DriverStateNamed` |
| `FaultFlag` | driver fault bits `emergency_stop`, `motor_setup_fault` | inside `DriverState` |
| `RuntimeError` | driver runtime error bit `safety_stop_active` | inside `DriverState` |
| `SystemStatus` | CPU usage per core, CPU temperature, load, RAM and disk usage | `rover_diag_manager` → `system_status` |
| `LedAnimation` | animation `id` (named constants `E_STOP` … `BLINKER_RIGHT`) + `param` | `SetLedAnimation` request |
| `LedAnimationCatalog` / `LedAnimationInfo` | loaded animations: id, name, priority layer | `rover_led` → `led/animations` |
| `LedState` / `LedSegmentState` / `LedLayerState` | what every priority layer (`ERROR`/`ALERT`/`INFO`/`STATE`) of every segment plays | `rover_led` → `led/state` |
| `LedImageAnimation` | image, duration, brightness, repeat, colour | `SetLedImageAnimation` request |
| `LedAnimationQueue` | list of queued animation names | not used |
| `RcChannels` | raw 11-bit CRSF channel counts (16 channels, index N-1 = channel N) | `rover_crsf_teleop` → `rc/channels` |
| `RcLinkStatus` | CRSF link statistics: uplink/downlink RSSI, link quality, SNR, RF mode, TX power | `rover_crsf_teleop` → `rc/link` |
| `RcCalibration` | per-channel raw-count min/mid/max and deadband for one transmitter | `SetRcCalibration` request |
| `RcCalibrationState` | RC calibration phase (`IDLE`/`CENTER`/`SWEEP`/`REVIEW`), samples, progress, teleop inhibit, verified E-Stop state | `rover_crsf_teleop` → `rc/calibration/state` (latched) |
| `DriveMode` | header, driving `mode` (`MANUAL`/`ASSISTED`/`AUTOMATIC`), obstacle `guard` state (`GUARD_BYPASSED`/`CLEAR`/`SLOWING`/`STOPPED`/`NO_DATA`), `reason` of the last change | `rover_drive_mode` (rover_orchestrator) → `drive_mode` (latched) |
| `MissionState` | mission id, state (`IDLE`/`RUNNING`/`HELD`/`SUCCEEDED`/`FAILED`/`CANCELLED`), current waypoint index, total, message | `rover_mission_manager` (rover_orchestrator) → `mission_state` (latched) |
| `LocalizationState` | indoor localization mode (`UNAVAILABLE`/`MAPPING`/`LOCALIZATION`/`SWITCHING`), map name, message | `rover_indoor_nav_manager` (rover_orchestrator) → `localization_state` (latched) |
| `MapInfo` / `MapList` | stored maps (name, resolution, size, save time) and the active map | `rover_indoor_nav_manager` → `maps` (latched) |
| `Place` / `PlaceList` | named poses (id, name, map, x, y, theta) on the active map | `rover_indoor_nav_manager` → `places` (latched) |

## Services

| Service | Request → Response | Served by |
|---------|--------------------|-----------|
| `SetLedAnimation` | `LedAnimation animation`, `bool repeating` → `success`, `message` | `rover_led` (`led/set_animation`) |
| `StopLedAnimation` | `uint16 id` → `success`, `message` | `rover_led` (`led/stop_animation`) |
| `SetLedBrightness` | `float32 data` (0–1) → `success`, `message` | `rover_led` (`led/set_brightness`) |
| `SetLedImageAnimation` | front/rear `LedImageAnimation`, `interrupting`, `repeating` → `success`, `message` | not implemented by any node |
| `SetDriveMode` | `uint8 mode` → `success`, `message`, `mode` in force | `rover_drive_mode` (`set_drive_mode`) |
| `StartRcCalibration` | `bool e_stop_confirmed` → `success`, `message` (refused unless the E-Stop is engaged) | `rover_crsf_teleop` (`rc/calibration/start`) |
| `SetRcCalibration` | `RcCalibration calibration`, `bool persist` → `success`, `message` | `rover_crsf_teleop` (`rc/calibration/apply`) |
| `SetMission` | `mission_id`, `PoseStamped[] waypoints` → `success`, `message` | `rover_mission_manager` (`set_mission`) |
| `SaveMap` / `LoadMap` / `DeleteMap` | map `name` (`LoadMap`: optional initial pose) → `success`, `message` | `rover_indoor_nav_manager` (`save_map`, `load_map`, `delete_map`) |
| `SavePlace` / `DeletePlace` | `Place` / place `id` → `success`, `message` (`SavePlace` returns the stored place) | `rover_indoor_nav_manager` (`save_place`, `delete_place`) |

```bash
ros2 interface show rover_msgs/msg/SafetyStatus
```
