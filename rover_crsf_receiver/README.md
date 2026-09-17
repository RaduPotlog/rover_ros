## Description

**ROS 2** package for receiving CRSF (RC channels values) packets over serial port (UART).

**CRSF protocol** packet format [description](https://github.com/crsf-wg/crsf/wiki/Message-Format).

### Topics

- `rc/channels` - received rc channels values
- `rc/link` - connection statistics information

---

## Installation

### Dependencies:

This package uses [`serial_driver`](https://github.com/ros-drivers/transport_drivers) from `ros-drivers/transport_drivers`.

Let's assume that your ROS 2 workspace is located at `~/ros2_ws/`.

### 1. Clone packages from git:

```bash
cd ~/ros2_ws/src

# Transport drivers (serial_driver dependency):
git clone https://github.com/ros-drivers/transport_drivers.git

# This package:
git clone https://github.com/AndreyTulyakov/ros2_crsf_receiver.git
```

### 2. Install dependencies via rosdep:

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Build

```bash
cd ~/ros2_ws

colcon build --packages-select io_context serial_driver
colcon build --packages-select crsf_receiver_msg
colcon build --packages-select crsf_receiver
```

### 4. Re-source

```bash
source ~/ros2_ws/install/setup.bash
```

---

## Running

### Set up params:

1. Serial device name: `device`, default is `/dev/ttyUSB0`
2. Baud rate: `baudrate`, default is `460800`
3. Link statistics info: `link_stats`, default is `true` (currently not used; link statistics are always published)
4. Receiver rate (hz): `receiver_rate`, default is `100`

### Run ros node:

```bash
# Run Node with default parameters
ros2 run crsf_receiver crsf_receiver_node

# Or setup and run Node with custom parameters values:
ros2 run crsf_receiver crsf_receiver_node --ros-args -p "device:=/dev/ttyUSB0" -p baudrate:=420000 -p link_stats:=true
```

### Check

After correct setup and running without errors you can check topics:

```bash
# Check channels values
ros2 topic echo /rover/rc/channels

# Check link statistics
ros2 topic echo /rover/rc/link

# Check receiver rate
ros2 topic hz /rover/rc/channels
```

### Link statistics message fields:

- `uplink_rssi_ant1` - ( dBm * -1 )
- `uplink_rssi_ant2` - ( dBm * -1 )
- `uplink_status` - Package success rate / Link quality ( % )
- `uplink_snr` - ( db )
- `active_antenna` - Diversity active antenna ( enum ant. 1 = 0, ant. 2 )
- `rf_mode` - ( enum 4fps = 0 , 50fps, 150hz)
- `uplink_tx_power` - ( enum 0mW = 0, 10mW, 25 mW, 100 mW, 500 mW, 1000 mW, 2000mW )
- `downlink_rssi` - ( dBm * -1 )
- `downlink_status` - Downlink package success rate / Link quality ( % )
- `downlink_snr` - ( db )