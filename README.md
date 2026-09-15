# rover_ros

Mechatronics Academy's Rover A1 ROS2.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b lyrical https://github.com/RaduPotlog/rover_ros.git src/rover_ros
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

```bash
cd src/rover_cppuprofile
cmake -Bbuild . -DPROFILE_ENABLED=ON
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
