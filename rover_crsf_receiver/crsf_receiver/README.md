# crsf_receiver

ROS 2 node that reads CRSF (Crossfire / ExpressLRS) frames from a receiver on a serial port
(UART) and publishes the RC channels and the link statistics. Frame format:
[CRSF message format](https://github.com/crsf-wg/crsf/wiki/Message-Format).

## Node `crsf_receiver_node`

The default node name is `crsf_reader_node`. The rover launches it as `rover_crfs_receiver_node`
from `rover_crfs_teleop`.

| Direction | Topic | Type | QoS |
|-----------|-------|------|-----|
| pub | `rc/channels` | `crsf_receiver_msg/CRSFChannels16` - 16 raw CRSF channel values | best effort, depth 1 |
| pub | `rc/link` | `crsf_receiver_msg/CRSFLinkInfo` - link statistics | best effort, depth 1 |

A timer running at `receiver_rate` publishes the latest decoded frame of each kind. It stops
publishing a kind once no new frame of it has arrived within the parser timeout, so a lost link
shows up as silence rather than stale values.

| Parameter | Default | Description |
|-----------|---------|-------------|
| `device` | `/dev/ttyUSB0` | Serial device. |
| `baudrate` | `460800` | UART baud rate (set as a custom rate after opening the port at 115200). |
| `receiver_rate` | `100` | Publish rate [Hz]. |
| `link_stats` | `true` | Declared but not used: link statistics are always published when received. |

The rover's `rover_crfs_teleop.launch.py` runs it with `device:=/dev/ttyUSB0`,
`baudrate:=460800`, `link_stats:=true` and `receiver_rate:=50`.

```bash
ros2 run crsf_receiver crsf_receiver_node --ros-args -p device:=/dev/ttyUSB0 -p baudrate:=460800
ros2 topic hz /rc/channels
```

## Dependencies

`serial_driver` and `io_context` from `rover_transport`, and `crsf_receiver_msg`.

## Layout

```
src/crsf_receiver_node.cpp   main
src/crsf_receiver.cpp        CrsfReceiverNode: parameters, serial port, publish timer
src/crsf_parser.cpp          byte stream -> CRSF frames (channels, link statistics), CRC8 check
src/baudrate_helper.cpp      custom (non-standard) baud rate via termios2
src/utils.cpp                parser output -> ROS messages
```
