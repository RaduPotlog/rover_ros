# rover_hardware_interface

ros2_control hardware plugins for Rover A1. It exports two plugins and has no launch files or
config of its own:

- **`RoverA1System`** drives the four wheel motors and the safety controller.
- **`PhidgetImuSensor`** reads the IMU.

## Plugins

| Plugin (`<ros2_control>` `<plugin>`) | Base class | Hardware | Exports |
|--------------------------------------|------------|----------|---------|
| `rover_hardware_interface/RoverA1System` | `hardware_interface::SystemInterface` | 4× Phidget DCC1000 DC motor controllers (Phidget22 SDK), safety controller over Modbus TCP | per wheel joint: `velocity` command; `position`, `velocity`, `effort` state |
| `rover_hardware_interface/PhidgetImuSensor` | `hardware_interface::SensorInterface` | Phidgets Spatial MOT0110, Madgwick orientation filter | `imu/orientation.{x,y,z,w}`, `imu/angular_velocity.*`, `imu/linear_acceleration.*` |

The URDF that wires them lives in `rover_description`:
- `urdf/rover_a1/rover_a1_macro.urdf.xacro`: component `rover_system_node`, with the Modbus
  endpoint, timeouts and motor/gearbox/encoder parameters as `<param>` tags.
- `urdf/common/imu.urdf.xacro`: component `rover_imu`.

The IMU parameters are declared in `src/phidgets_spatial_parameters.yaml`. `rover_controller`
starts `controller_manager` and the controllers on top of these plugins.

Reference datasheets: `docs/DCC1000_reference.pdf` (motor controller),
`docs/MOT0110_reference.pdf` (IMU).

## ROS interface (`rover_hardware_controller` node)

`RoverA1System` creates a helper node, `rover_hardware_controller`, inside the
`controller_manager` process. It inherits that process's namespace and `diagnostics` remap.

| Direction | Name | Type |
|-----------|------|------|
| pub | `hardware_interface/rover_driver_state` | `rover_msgs/RoverDriverState` |
| pub | `hardware_interface/safety_status` | `rover_msgs/SafetyStatus` - plant state: HW E-Stop button, contactor feedback, latch, link health |
| pub | `hardware_interface/safety_command_echo` | `rover_msgs/SafetyCommandEcho` - read-backs of the coils *we* drive; diagnostic, may only ever inhibit |
| pub | `hardware_interface/aux_io_state` | `rover_msgs/AuxIoState` - general-purpose aux IO (PLC DIO06..11 inputs, DIO00..05 output read-backs). Not safety |
| srv | `hardware_interface/sw_user_e_stop_set` | `std_srvs/Trigger` - set the software E-Stop |
| srv | `hardware_interface/sw_user_e_stop_reset` | `std_srvs/Trigger` - reset the software E-Stop |
| srv | `hardware_interface/sw_e_stop_latch_reset` | `std_srvs/Trigger` - clear the safety relay latch |
| srv | `hardware_interface/aux_output_<0..5>/set` | `std_srvs/SetBool` - switch aux output DIO00..05 ON/OFF; replies after the PLC acknowledges the write |
| pub | `diagnostics` | hardware id `Rover System`: driver and safety controller status |

The aux IO topic and services exist only on real hardware: in simulation `gz_ros2_control`
replaces `RoverA1System`, so nothing talks to the PLC.

Consumers: `rover_safety` (driver state, the SW E-Stop echo, e-stop set), `rover_twist_mux`'s
`rover_motion_lock_node` (both safety topics), `rover_crsf_teleop` (safety status + e-stop
services) and the Foxglove dashboard.

The two safety topics replaced a single `gpio_state` of seven undifferentiated bools. Plant state
and command read-backs are separated because the difference decides what a consumer is allowed to
conclude — see `rover_arch/SAFETY_CHAIN.md` §4 and the message files themselves.

```bash
ros2 topic echo /rover/hardware_interface/safety_status
ros2 service call /rover/hardware_interface/sw_user_e_stop_reset std_srvs/srv/Trigger
ros2 control list_hardware_components
```

## Layout

```
domain/                    emergency stop, velocity command guard, error filter, IMU calibration,
                           driver/GPIO ports - no ROS, hardware_interface or vendor SDK
application/               RoverControlLoopUseCase - the read()/write() cycle decisions
rover_driver/              Phidget motor driver adapters (Phidget22)
rover_safety_controller/   safety controller E-Stop / GPIO adapter over Modbus
rover_sensors/             PhidgetImuSensor
rover_system/              RoverSystem base + RoverA1System (the ros2_control plugin)
system_ros_interface/      rover_hardware_controller node: topics, services, diagnostics
```

The Modbus TCP client itself lives in `rover_transport/rover_modbus_driver` - it was
extracted from this package so it could be depended on and tested on its own.

## Tests

```bash
colcon test --packages-select rover_hardware_interface && colcon test-result --all --verbose
```

- `test/` has gtests per layer: domain, application, driver, safety controller,
  sensors, system and the ROS interface. Fakes live in `test/fakes/`.
- Three architecture checks run as CTest tests:
  - `scripts/check_domain_purity.sh`: domain code includes no ROS, hardware_interface or
    vendor headers.
  - `scripts/check_application_purity.sh`: the same rule for application code.
  - `scripts/check_rt_path_purity.sh`: no known-blocking Modbus or Phidget SDK calls on the
    `read()`/`write()` real-time path.
