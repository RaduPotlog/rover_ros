# rover_ros

Mechatronics Academy's Rover A1 ROS2.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b master https://github.com/RaduPotlog/rover_ros.git src/rover_ros
```

### Setup environment variables

#### Real rover:

```bash
export ROVER_ROS_BUILD_TYPE=hardware
```

#### Simulated rover:

```bash
export ROVER_ROS_BUILD_TYPE=simulation
```

### Clone dependency

```bash
vcs import src < src/rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos
```

### Build

```bash
sudo apt install usbutils
sudo apt install plocate

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i
```

#### Only for real rover

`rover_modbus` used to need a manual `sudo make install` here as well. It is now an ament
package that colcon builds in dependency order, so only `rover_cppuprofile` remains.

If this machine ever ran that old step, remove the stale copy first - its headers installed
flat into `/usr/local/include`, which is on a default search path and will shadow the new
`<MB/...>` ones:

```bash
sudo rm -f /usr/local/lib/libModbus_Core.so
sudo rm -f /usr/local/include/{connection,crc,modbusCell,modbusException,modbusRequest,modbusResponse,modbusUtils,server}.hpp
sudo rm -rf /usr/local/lib/cmake/Modbus_Core
```

```bash
cd src/rover_cppuprofile
cmake -Bbuild . -DPROFILE_ENABLED=ON
cmake --build build
cd build
sudo make install
cd ../../..
```

#### For real rover and simulated rover

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_metapackage --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

### Running

#### Real rover:

```bash
ros2 launch rover_bringup rover_bringup.launch.py
```

#### Simulated rover:

```bash
ros2 launch rover_gazebo simulation.launch.py
```
