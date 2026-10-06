# Open items

This page lists what the manual can't state yet, and the places where the repository disagrees
with itself. It was compiled on 2026-10-01 from `master` and updated on 2026-10-02 for the wheel PID merge and the stale-documentation fixes. When an item is resolved, update the
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
| `motor_current_limit` | 15.0 A | 10 A in the URDF comment and in the `utils.hpp` comment | `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro` |
| Maximum angular speed | 1.5 rad/s | "1.7 m/s2" (SYS-SR-004 answer; also a unit error) | `wheel_01_controller.yaml`, `rover_platform_mbse/system/*.xlsx` |
| Battery voltage | 24 V (`motor_supply_voltage`) | 41.4 V in the simulated battery | `rover_a1_macro.urdf.xacro`, `rover_description/urdf/common/battery.urdf.xacro` |
| BMS link | BLE → ESP32 → UDP | RS485/RS232 serial link (hardware part of the architecture diagram) | `rover_battery/`, `rover_arch/rover_a1_arch.drawio` |
| Motor power switching | One contactor with an auxiliary contact | Four relays K1–K4 on QX 0.0, no auxiliary contact (schematic) | `rover_arch/SAFETY_CHAIN.md`, `rover_arch/rover_a1_arch.drawio` |
| IMU hub port | -1 (any) is used when opening the device | 0 in the URDF, default 2 in the parameters | `rover_hardware_interface/src/phidget_imu_sensor.cpp` |
| SYS-SR-001 mass limit | — | The requirement text says ≤ 60 kg *excluding* payload; the Open Questions sheet answers that payload *is* included | `rover_platform_mbse/system/ROVER-A1-ROVER_ROS_SYS-SR-V-002.xlsx` |
| MBSE baseline | Current values above | 43.45 kg URDF mass, 100 Hz control, 2.5 kg wheels, 0.615 m track | `rover_platform_mbse/data/platform_parameters.json` (baseline 35aeb3e) |

## Behaviour worth reviewing

These come from reading the code while writing the manual. They are not documentation errors.

- The wheel PIDs' reference model (`integral_reference_delay` 0.40 s / `_time_constant` 0.12 s)
  was fitted on the ground with `motor_acceleration` 1.0 (2026-10-06). The URDF now sets 2.0 and
  the model has not been re-fitted, nor have the acceleration limits (1.4 m/s², 1.3 rad/s²)
  (`rover_controller/config/wheel_01_controller.yaml`).
- `velocity_command_zero_tolerance` is still 0.4 rad/s. The reason for it (a frozen PID
  integral) is gone while `stop_at_zero_reference` is on, so it can come down toward 0.01 once
  that is verified on the rover.
- The `E_STOP` LED animation follows only the hardware button. A software E-Stop, an RC E-Stop
  or a held latch leaves the LEDs on `READY`.
- The software motor-driver-fault input (`COIL_3`) is only asserted during start-up; nothing
  asserts it at runtime.
- The Foxglove layout (`rover_foxglove/`) uses topics the bridge whitelist in
  `rover_bringup/launch/rover_web_bridges.launch.py` leaves out, among them its joystick topic
  `teleop_foxglove_cmd_vel_stamped`. The whitelist is deliberate: the web joystick goes through
  `teleop_web_cmd_vel_stamped` and `rover_drive_mode`. The layout should follow, or those panels
  stay empty unless `ROVER_FOXGLOVE_TOPIC_WHITELIST` is opened.
