# IO and network

This page covers the user aux IO on the safety PLC, the Modbus TCP link that carries it, the platform network, and the USB devices on the ROS controller.

ROS names on this page are relative to the robot namespace `rover`. For example, `hardware_interface/aux_io_state` is `/rover/hardware_interface/aux_io_state`.

## Aux IO for users

The safety PLC (Arduino Portenta Machine Control) has 12 general-purpose digital IO points for payloads: 6 outputs and 6 inputs. They are read and written over the same Modbus TCP link as the safety signals.

!!! danger "Not part of the safety chain"
    Never use the aux IO to gate motion or E-Stop logic. Use `hardware_interface/safety_status` for that. The aux IO is polled at 10 Hz over a link that can fail.

| Signal | PLC pin | Direction | Modbus object | ROS access |
|--------|---------|-----------|---------------|------------|
| `aux_output_0` | DIO00 | output | `COIL_8` | `hardware_interface/aux_output_0/set` |
| `aux_output_1` | DIO01 | output | `COIL_9` | `hardware_interface/aux_output_1/set` |
| `aux_output_2` | DIO02 | output | `COIL_10` | `hardware_interface/aux_output_2/set` |
| `aux_output_3` | DIO03 | output | `COIL_11` | `hardware_interface/aux_output_3/set` |
| `aux_output_4` | DIO04 | output | `COIL_12` | `hardware_interface/aux_output_4/set` |
| `aux_output_5` | DIO05 | output | `COIL_13` | `hardware_interface/aux_output_5/set` |
| `aux_input_0` | DIO06 | input | `COIL_14` | `inputs[0]` of `aux_io_state` |
| `aux_input_1` | DIO07 | input | `COIL_15` | `inputs[1]` |
| `aux_input_2` | DIO08 | input | `COIL_16` | `inputs[2]` |
| `aux_input_3` | DIO09 | input | `COIL_17` | `inputs[3]` |
| `aux_input_4` | DIO10 | input | `COIL_18` | `inputs[4]` |
| `aux_input_5` | DIO11 | input | `COIL_19` | `inputs[5]` |

Source: `rover_arch/SAFETY_CHAIN.md` §2, `rover_hardware_interface/src/rover_safety_controller/rover_safety_controller.cpp`, `rover_msgs/msg/AuxIoState.msg`.

Electrical ratings of the DIO pins (voltage levels, output current, input thresholds) and the connector pinout are **TBD**.

### Setting an output

Each output has a `std_srvs/SetBool` service. The reply comes after the PLC acknowledges the write. A failure returns `success: false` with the reason.

```bash
ros2 service call /rover/hardware_interface/aux_output_0/set std_srvs/srv/SetBool "{data: true}"
ros2 service call /rover/hardware_interface/aux_output_0/set std_srvs/srv/SetBool "{data: false}"
```

- All outputs are driven OFF every time the hardware interface starts.
- Aux writes use the link without priority, so they never delay the watchdog heartbeat or an E-Stop write.
- The PLC refreshes the DIO pins on its 100 ms task.

Source: `rover_hardware_interface/src/rover_system/rover_system.cpp`, `rover_arch/SAFETY_CHAIN.md` §2.

### Reading the state

`hardware_interface/aux_io_state` (`rover_msgs/AuxIoState`) is published at 20 Hz with QoS reliable, volatile, keep last 1.

| Field | Meaning |
|-------|---------|
| `header` | Publish time |
| `io_sample_time` | When the PLC was actually polled. Time out on this field, not on message arrival |
| `inputs[6]` | DIO06–DIO11, `true` = input active |
| `outputs[6]` | Read-back of DIO00–DIO05 as the PLC drives them, `true` = ON |
| `link_healthy` | `false` means the values are the last ones read and may be stale |

```bash
ros2 topic echo /rover/hardware_interface/aux_io_state
```

Source: `rover_msgs/msg/AuxIoState.msg`, `rover_hardware_interface/src/system_ros_interface/system_ros_interface.cpp`, `driver_states_update_frequency` in `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro`.

!!! note
    The aux IO topic and services exist only on real hardware. In simulation `gz_ros2_control` replaces `RoverA1System` and nothing talks to the PLC.

## Modbus TCP link to the safety PLC

The `RoverA1System` hardware plugin owns the link from two background threads: a watchdog thread and an IO poll thread. `read()` and `write()` never do Modbus I/O.

| Item | Value | Source |
|------|-------|--------|
| Address | `192.168.88.11:502` | `rover_a1_macro.urdf.xacro` (`modbus_host`, `modbus_port`) |
| Unit id | 255 | `rover_transport/rover_modbus_driver/include/rover_modbus_driver/application/modbus_discrete_io_client.hpp` |
| Function codes | FC1 read coils, FC2 read discrete inputs, FC5 write single coil | `rover_arch/SAFETY_CHAIN.md` §2 |
| Response timeout | 150 ms | `rover_a1_macro.urdf.xacro` (`modbus_response_timeout_ms`) |
| Connection retry | forever (`0`), 1000 ms apart | `modbus_connection_retry_count`, `modbus_connection_retry_delay_ms` |
| IO poll period | 100 ms (10 Hz), 4 batched transactions | `safety_io_poll_period_ms` |
| Watchdog heartbeat toggle | 200 ms (5 Hz) on `COIL_1` | `safety_wdg_kick_period_ms` |
| PLC watchdog window | about 1 s (measured) | `rover_arch/SAFETY_CHAIN.md` §3 |
| Latch reset pulse | 100 ms | `safety_latch_reset_pulse_ms` |
| Safety and aux topics publish rate | 20 Hz | `driver_states_update_frequency` |

If the heartbeat stops for longer than the PLC watchdog window, the PLC latches the E-Stop. Do not raise `safety_wdg_kick_period_ms` without re-measuring the window. The E-Stop chain is described on the [Safety](../safety.md) page.

??? note "Full Modbus object map"

    | Signal | Modbus object | Direction | Writable | Area |
    |--------|---------------|-----------|----------|------|
    | `hw_e_stop_user_button` | `CONTACT_0` (FC2) | PLC → ROS | no | discrete input |
    | `motor_contactor_engaged` | `COIL_0` | PLC → ROS | no | Digital Outputs |
    | `cpu_wdg_heartbeat` | `COIL_1` | ROS → PLC | yes | Digital Outputs |
    | `sw_e_stop_user_button` | `COIL_2` | ROS → PLC | yes | Digital Outputs |
    | `sw_e_stop_motor_driver_fault` | `COIL_3` | ROS → PLC | yes | Digital Outputs |
    | `sw_e_stop_latch_reset` | `COIL_4` | ROS → PLC (pulse) | yes | Digital Outputs |
    | `sw_e_stop_latch_status` | `COIL_5` | PLC → ROS | no | Digital Outputs |
    | `aux_output_0..5` (DIO00–DIO05) | `COIL_8..COIL_13` | ROS → PLC | yes | Programmable DIO |
    | `aux_input_0..5` (DIO06–DIO11) | `COIL_14..COIL_19` | PLC → ROS | no | Programmable DIO |

    The IO poll reads every mapped object in four transactions: one FC2 read of `CONTACT_0`, and FC1 reads of coils 0–5, 8–13 and 14–19.

    Two limits of the Portenta PLC IDE Modbus server shape these reads:

    - A read must stay inside one memory area. Digital Outputs (0–7) and Programmable DIO (8–19) are separate areas. A read that crosses them returns `false` for the second area.
    - A read must cover at most 8 coils. Longer replies are malformed. A `static_assert` in the code enforces this.

    The PLC IDE labels the DIO points "Modbus Coil 9..20" because it counts from 1. The PDU address counts from 0, so DIO00 is address 8. Check the mapping on the hardware after changing the PLC program: `mbpoll -t 0 -0 -r 0 -c 20`.

    Source: `rover_arch/SAFETY_CHAIN.md` §2, `rover_hardware_interface/src/rover_safety_controller/rover_safety_controller.cpp` (`kCoilReadBlocks`).

More detail: [SAFETY_CHAIN.md](https://github.com/RaduPotlog/rover_ros/blob/master/rover_arch/SAFETY_CHAIN.md), [rover_hardware_interface README](https://github.com/RaduPotlog/rover_ros/blob/master/rover_hardware_interface/README.md), [rover_modbus_driver README](https://github.com/RaduPotlog/rover_ros/blob/master/rover_transport/rover_modbus_driver/README.md).

## Platform network

The platform has two IP networks. The safety PLC sits on a dedicated Ethernet link to the ROS controller. The RUTX11 router provides the platform LAN and a Wi-Fi network for the LED and BMS controllers.

| Device | Interface | IP address | Ports | Source |
|--------|-----------|------------|-------|--------|
| ROS controller | ETH0 (PLC link) | `192.168.88.10/24` | none | `rover_arch/rover_a1_arch.drawio` |
| Safety PLC | ETH0 | `192.168.88.11/24` | 502/TCP Modbus | `rover_a1_macro.urdf.xacro`, drawio |
| ROS controller | ETH1 (router LAN) | `192.168.1.201/24` | 4444/UDP BMS telemetry in | `rover_battery/config/rover_battery.yaml`, drawio |
| ROS controller | same | `192.168.1.201` | 7447/TCP Zenoh router | `rover_scripts/setup_rover_pc.sh` |
| ROS controller | web bridge | bind address not set in rover_ros (upstream default) | 8765/TCP Foxglove bridge (`rover_foxglove_bridge`) | `rover_bringup/README.md` |
| ROS controller | web bridge | bind address not set in rover_ros (upstream default) | 9090/TCP rosbridge (`rover_rosbridge_websocket`) | `rover_bringup/launch/rover_web_bridges.launch.py` |
| ROS controller | WLAN0 | `192.168.77.203/24` | none | drawio |
| RUTX11 router | LAN | `192.168.1.1/24` | none | drawio |
| RUTX11 router | Wi-Fi AP | `192.168.77.1/24` | none | drawio |
| ESP32 BMS bridge / rear LED controller | Wi-Fi | `192.168.77.201` | 3333/UDP LED channel 2 (rear panel) in; 3003 shutdown endpoint (optional, not configured) | `rover_led/config/rover_a1_udp_led_channel_2.yaml`, `rover_safety/config/shutdown_hosts.yaml` |
| Front LED controller | Wi-Fi | `192.168.77.202` | 3334/UDP LED channel 1 (front panel) in | `rover_led/config/rover_a1_udp_led_channel_1.yaml` |

The ROS controller addresses `192.168.88.10` and `192.168.77.203`, and the router addresses, appear only in the architecture diagram, not in a config file. The router's uplink is site-specific and not covered here.

## USB devices

| Device | Connection | Settings | Source |
|--------|------------|----------|--------|
| Phidgets MOT0110 IMU | USB, opened directly (not through the VINT hub) | Data interval 8 ms, Madgwick filter, magnetometer off, ENU world frame | `rover_description/urdf/common/imu.urdf.xacro`, `rover_hardware_interface/src/rover_sensors/phidget_imu_sensor.cpp` |
| Phidgets VINT hub | USB | DCC1000 boards on hub ports 0 (FL), 1 (FR), 4 (RL), 5 (RR); any hub serial number (`-1`) | `rover_hardware_interface/src/rover_driver/rover_a1_driver.cpp` |
| ExpressLRS (CRSF) receiver | USB serial, `/dev/ttyUSB0` | 460800 baud | `rover_crsf_teleop/config/rover_crsf_teleop.yaml` |

!!! warning "ELRS baud rate"
    Keep the receiver at 460800 baud. The serial driver cannot open the CRSF default of 420000. `/dev/ttyUSB0` can move to another device when a second USB serial adapter is plugged in. A stable `/dev/serial/by-id/` path is safer.

USB vendor/product ids and udev rules are **TBD**: the repository has none. The lidar and GNSS receiver are part of the sensor payload, documented in the `rover_sensors` repository.
