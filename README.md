# ma_rover_ros

Mechatronics Academy's Rover A1 ROS2.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b main https://github.com/RaduPotlog/rover_ros.git
```

### Setup environment variables

#### Real rover:

```bash
export ROVER_ROS_BUILD_TYPE=hardware
```

### Build

```bash
vcs import src < src/ma_rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos

rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

apt install usbutils

cd src/ma_rover_cppuprofile
cmake -Bbuild . -DPROFILE_ENABLED=OFF
cmake --build build
cd build
make install
cd ../../..

cd src/ma_rover_modbus
apt install libnet1-dev
cmake -Bbuild .
cmake --build build
cd build
make install
cd ../../..

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_metapackage --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

apt install plocate

source install/setup.bash
```

### Running

#### Real rover:

```bash
ros2 launch rover_bringup rover_bringup.launch.py
```
