# Open items

This page lists what the manual can't state yet, and the places where the repository disagrees
with itself. It was compiled on 2026-10-01 from `master` and updated on 2026-10-02 for the wheel PID merge. When an item is resolved, update the
source file first, then the manual page, then remove the item here.

## Values not yet defined (TBD)

### Physical and environmental

| Item | Where it is needed | Requirement |
|---|---|---|
| Rated payload | [Specification](hardware/specification.md) | SYS-SR-027 |
| Maximum slope and terrain types | Specification | SYS-SR-028 |
| Operating temperature range | Specification | SYS-SR-029 |
| Ingress protection (IP) rating | Specification | SYS-SR-029 |
| Dimension limit (L × W × H) | Specification | SYS-SR-026 |
| Measured mass (scale) | Specification; the 54.35 kg is CAD | SYS-SR-001 |
| Measured stopping distance from 1.0 m/s | [Safety](safety.md) | SYS-SR-014 |
| Photos of the real rover | [Overview](index.md) gallery | |
| Location of the hardware E-Stop and reset buttons on the chassis | Safety | SYS-SR-006 |
| Transport and handling: lifting points, towing, free-wheeling, storage | Safety | |

### Power

| Item | Where it is needed |
|---|---|
| Nominal pack voltage, cell count, energy (Wh) | [Specification](hardware/specification.md), [Power and battery](hardware/power-and-battery.md) |
| Runtime on hardware (SYS-SR-015) | Specification |
| Charger, charging connector, voltage/current, time and procedure (SYS-SR-018) | Power and battery |
| Main power switch: part and location; power-down order | Power and battery |
| How the LED controllers, router and IMU are powered | Power and battery |
| BMS telemetry rate (set by the ESP32 firmware, outside this repo) | Power and battery |

### Components and electrical

| Item | Where it is needed |
|---|---|
| Controller computer model, CPU and OS (the diagram only says "RPi") | [Components](hardware/components.md), [Software overview](software/overview.md) |
| Part numbers: motors (and power rating), BMS, battery pack, VINT hub, ELRS receiver, LED controllers, contactor/relays, E-Stop button, power board, Power Guard | Components |
| Portenta DIO electrical ratings and connector pinout | [IO and network](hardware/io-and-network.md) |
| USB vendor/product IDs and udev rules (the repo has none) | IO and network |
| Datasheets for the Portenta, motors, ELRS receiver, VINT hub and router | Components |
| Bind address of the Foxglove and rosbridge bridges | IO and network |

## Source conflicts

The manual uses the value the code runs with. The other source should be corrected.

| Topic | Value used by the code | Other value | Files |
|---|---|---|---|
| Track width | 0.617 m | 0.615 m "measured" | `rover_description/config/wheel_01.yaml`, `rover_controller/config/wheel_01_controller.yaml` (comment), `rover_platform_mbse/data/*` |
| Effective track (track × multiplier) | 0.617 × 1.659 = 1.0236 m | 1.020 m (controller comment); 1.0204 m (`effective_track_width` in `rover_crsf_teleop/config/rover_crsf_teleop.yaml`) | `rover_controller/config/wheel_01_controller.yaml` |
| `wheel_separation_multiplier` | 1.659 | 1.5 (`rover_description/README.md`); 1.63 (`rover_controller/docs/wheel_pid_tuning_notes.md`) | `rover_controller/config/wheel_01_controller.yaml` |
| `motor_current_limit` | 15.0 A | 10 A in the URDF comment and in the `utils.hpp` comment | `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro` |
| `motor_acceleration` | 2.0 duty/s | The URDF comment describes 10.0; the wheel PID reference model (`integral_reference_delay` 0.15 s / `_time_constant` 0.08 s) was fitted on the ground at 10.0 and not re-fitted | `rover_a1_macro.urdf.xacro`, `rover_controller/config/wheel_01_controller.yaml` |
| `velocity_command_zero_tolerance` | 0.4 rad/s | 0.35 (`rover_arch/SAFETY_CHAIN.md` §6, URDF comment) | `rover_a1_macro.urdf.xacro` |
| Maximum angular speed | 1.5 rad/s | "1.7 m/s2" (SYS-SR-004 answer; also a unit error) | `wheel_01_controller.yaml`, `rover_platform_mbse/system/*.xlsx` |
| Battery voltage | 24 V (`motor_supply_voltage`) | 41.4 V in the simulated battery | `rover_a1_macro.urdf.xacro`, `rover_description/urdf/common/battery.urdf.xacro` |
| BMS link | BLE → ESP32 → UDP | RS485/RS232 serial `battery_monitor` (architecture diagram) | `rover_battery/`, `rover_arch/rover_a1_arch.drawio` |
| Motor power switching | One contactor with an auxiliary contact | Four relays K1–K4 on QX 0.0, no auxiliary contact (schematic) | `rover_arch/SAFETY_CHAIN.md`, `rover_arch/rover_a1_arch.drawio` |
| LED channel 1 UDP port | 3334 | 3333 | `rover_led/config/rover_a1_udp_led_channel_1.yaml`, `rover_led/README.md` |
| Simulated lidar height (no `ROVER_LIDAR_*` set) | 0.68 m | 0.45 m | `rover_a1_macro.urdf.xacro`, `rover_gazebo/README.md` |
| IMU hub port | -1 (any) is used when opening the device | 0 in the URDF, default 2 in the parameters | `rover_hardware_interface/src/phidget_imu_sensor.cpp` |
| SYS-SR-001 mass limit | — | The requirement text says ≤ 60 kg *excluding* payload; the Open Questions sheet answers that payload *is* included | `rover_platform_mbse/system/ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx` |
| MBSE baseline | Current values above | 43.45 kg URDF mass, 100 Hz control, 2.5 kg wheels, 0.615 m track | `rover_platform_mbse/data/platform_parameters.json` (baseline 35aeb3e) |

## Stale documentation and comments

| File | Stale statement | Current behaviour |
|---|---|---|
| `rover_safety/README.md` | Subscribes to `hardware_interface/gpio_state` (`GpioState`) | Subscribes to `hardware_interface/safety_command_echo` (`SafetyCommandEcho`) |
| `rover_safety/README.md` | A failed configure is not retried | Both nodes retry every `configure_retry_period` (5.0 s) |
| `rover_safety/README.md` | `rover_led_safety_node` runs in simulation | The simulation does not launch rover_safety |
| `rover_crsf_teleop/config/rover_crsf_teleop.yaml` | Comments mention `gpio_state` and a 2 Hz IO refresh | Reads `safety_status`; IO poll is 10 Hz |
| `rover_crsf_teleop/README.md` | Bridge node `rover_serial_bridge_node`; no `safety_status` subscription | Node `rover_crsf_serial_bridge`; subscribes to `hardware_interface/safety_status` |
| `rover_a1_macro.urdf.xacro` | `safety_io_poll_period_ms` comment refers to a `gpio_state` staleness timeout | `gpio_state` was removed |
| `rover_controller/docs/wheel_pid_tuning_notes.md` | Multiplier 1.63, `gpio_state`; `i_clamp` 0.25 front / 0.33 rear bounds overshoot | 1.659, `safety_command_echo`; `i_clamp` 2.0 on all wheels with the model-reference integral |
| `rover_controller/README.md` | `i_clamp` per wheel (0.25 / 0.33) is what bounds overshoot; raise `motor_acceleration` once the PID owns the speed dynamics | `i_clamp` 2.0 on all wheels; overshoot is handled by the wheel loop options (`stop_at_zero_reference`, `integral_reference_*`, `scale_integral_with_reference`), which the README doesn't mention |
| `rover_a1_macro.urdf.xacro` | `velocity_command_zero_tolerance` comment: the frozen I-term (`i_clamp` 0.25 / 0.33) keeps the command above zero while inhibited | `stop_at_zero_reference` sends exactly 0 at a zero reference; `i_clamp` is now 2.0 |
| `rover_led/README.md` | Front segment `0-39`, E_STOP duration 6, LED 0 on the right | `39-0`, 4, LED 0 on the left |
| `rover_arch/SAFETY_CHAIN.md` §5 | The latch can't be cleared while the rover moves | That check guards `sw_user_e_stop_reset`; `sw_e_stop_latch_reset` only needs the hardware interface ACTIVE |
| `rover_arch/rover_a1_arch.drawio` | `/hardware_interface/gpio_state`, `GpioController`, serial `battery_monitor` | Removed or replaced |
| `rover_twist_mux/README.md` | Simulation does not launch this package | `simulate_robot.launch.py` launches it |
| `rover_localization/README.md` | TF `odom → base_link`; rover_gazebo passes `fuse_gps` False | `odom → base_footprint`; `use_gps` is passed through |
| `rover_description/README.md` | `base_link` is the root link | The root is `base_footprint` |
| `rover_world/README.md` | An empty world | Walls, 4 boxes and 3 pillars |
| `rover_msgs/README.md` | Driver names default/front/rear; several msgs/srvs missing | rear_left/rear_right/front_left/front_right; see [ROS 2 API](software/ros-api.md) |
| `rover_diag_manager/README.md` | Group list without `Lidar` | `Lidar` group exists; `rover_command_freshness_node` has no analyzer (lands in Other) |
| `rover_bringup/README.md` | Container `rovera1-app` | Top-level README says `rover-a1-platform` |
| `rover_gazebo/README.md` | No mention of the drive `wheel_radius` override | `sim_wheel_pid.yaml` sets 0.1699 m in simulation |

## Behaviour worth reviewing

These come from reading the code while writing the manual. They are not documentation errors.

- The `E_STOP` LED animation follows only the hardware button. A software E-Stop, an RC E-Stop
  or a held latch leaves the LEDs on `READY`.
- The software motor-driver-fault input (`COIL_3`) is only asserted during start-up; nothing
  asserts it at runtime.
- The Foxglove layout (`rover_foxglove/`) uses topics the bridge whitelist in
  `rover_bringup/launch/rover_web_bridges.launch.py` leaves out, among them its joystick topic
  `teleop_foxglove_cmd_vel_stamped`. The whitelist is deliberate: the web joystick goes through
  `teleop_web_cmd_vel_stamped` and `rover_drive_mode`. The layout should follow, or those panels
  stay empty unless `ROVER_FOXGLOVE_TOPIC_WHITELIST` is opened.
