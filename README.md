# ma_rover_ros

Mechatronics Academy's Rover A1 ROS2.

## Quick start

### Create workspace

```bash
mkdir -p ~/ros2_ws/rover_a1
cd ~/ros2_ws/rover_a1
git clone -b main git@github.com:mechatroacademy/ma_rover_ros.git src/ma_rover_ros
```

### Setup environment variables

#### Real rover:

```bash
export ROVER_ROS_BUILD_TYPE=hardware
```

#### Simulation:

```bash
export ROVER_ROS_BUILD_TYPE=simulation
```

### Build

```bash
vcs import src < src/ma_rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos

sudo rosdep init
rosdep update --rosdistro $ROS_DISTRO
rosdep install --from-paths src -y -i

source /opt/ros/$ROS_DISTRO/setup.bash
colcon build --symlink-install --packages-up-to rover_metapackage --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF

source install/setup.bash
```

### Running

#### Real rover:

```bash
ros2 launch rover_bringup rover_bringup.launch.py
```

```bash
sudo apt install ros-$ROS_DISTRO-rqt*
sudo apt install ros-$ROS_DISTRO-imu-tools

ros2 launch rover_description rover_rviz.launch.py

ros2 run rqt_gui rqt_gui
```

#### Simulation:

```bash
ros2 launch rover_gazebo simulation.launch.py
```

### Rviz:

#### Real rover:

![Robot Model](images/rover_real_imu.png)

### Gazebo

#### Simulation:

![Robot Model](images/rover_simulation_imu.png)
