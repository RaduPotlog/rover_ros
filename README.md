# rover_ros

Mechatronics Academy's Rover A1 ROS2.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b main https://github.com/RaduPotlog/rover_ros.git src/rover_ros
```

### Setup environment variables

#### Real rover:

```bash
export ROVER_ROS_BUILD_TYPE=hardware
```

### Build

```bash
sudo apt install usbutils
sudo apt install plocate

vcs import src < src/rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

cd src/rover_cppuprofile
cmake -Bbuild . -DPROFILE_ENABLED=OFF
cmake --build build
cd build
sudo make install
cd ../../..

cd src/rover_modbus
sudo apt install libnet1-dev
cmake -Bbuild .
cmake --build build
cd build
sudo make install
cd ../../..

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_metapackage --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash

sudo cp src/rover_ros/rover_bringup/scripts/99-elrs.rules /etc/udev/rules.d/
sudo cp src/rover_ros/rover_bringup/scripts/99-libphidget22.rules /etc/udev/rules.d/

sudo udevadm control --reload-rules
sudo udevadm trigger

sudo cp src/rover_ros/rover_bringup/scripts/60-wifi-init.yaml /etc/netplan
sudo chmod 600 /etc/netplan/60-wifi-init.yaml

sudo cp src/rover_ros/rover_bringup/scripts/60-plc-init.yaml /etc/netplan
sudo chmod 600 /etc/netplan/60-plc-init.yaml

sudo cp src/rover_ros/rover_bringup/scripts/60-switch-init.yaml /etc/netplan/
sudo chmod 600 /etc/netplan/60-switch-init.yaml

sudo netplan apply

```

### Running

#### Real rover:

```bash
ros2 launch rover_bringup rover_bringup.launch.py
```
