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

```bash
# Every $ROS_DISTRO below is expanded before ROS is sourced, so set it explicitly.
export ROS_DISTRO=lyrical

# Default namespace of every rover_ros launch file. The rover runs under `rover`, so the
# orchestrator computer must export the same value or the topics never meet.
export ROVER_NAMESPACE=rover
```

#### Real rover:

```bash
export ROVER_ROS_BUILD_TYPE=hardware
```

#### Simulated rover:

```bash
export ROVER_ROS_BUILD_TYPE=simulation
```

Only needed to clone and build by hand: `rover_sim.sh` (see [Running](#running)) sets it for
its own session, and `rover_sim.sh --build` builds the simulation workspace.

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
# Sets up the simulation environment itself, whatever ~/.bashrc exports for the real rover:
# ROVER_ROS_BUILD_TYPE=simulation, ROVER_NAMESPACE (default rover), no ZENOH_CONFIG_OVERRIDE,
# and a local zenoh router on 127.0.0.1:7447. --build builds the workspace first; extra
# arguments go to simulation.launch.py.
~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh
~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh --build use_rviz:=False

# Second terminal (ros2 CLI, orchestrator) on the same local middleware:
source ~/ros2_ws/rover_a1/src/rover_ros/rover_gazebo/scripts/rover_sim.sh
ros2 launch rover_navigation bringup.launch.py use_sim_time:=True localization_source:=slam
```

See [rover_gazebo/README.md](rover_gazebo/README.md#running) for the details and the manual
`ros2 launch rover_gazebo simulation.launch.py` form.

### Testing

```bash
# The build above passes -DBUILD_TESTING=OFF, so rebuild the package under test with it on.
colcon build --symlink-install --packages-select <pkg>
colcon test --packages-select <pkg> --parallel-workers 1
colcon test-result --all
```

Run node tests one at a time: parallel workers make them flaky under the zenoh middleware.

## Related repositories

A complete rover is three repositories, one per container:

- [`rover_ros`](https://github.com/RaduPotlog/rover_ros) - this one, the platform
  (`rover-a1-platform`).
- [`rover_sensors`](https://github.com/RaduPotlog/rover_sensors) - the sensor payload,
  GNSS and lidar drivers (`rover-a1-sensors`).
- [`rover_orchestrator`](https://github.com/RaduPotlog/rover_orchestrator) - the autonomy
  stack, Nav 2 and mission supervision (`rover-a1-orchestrator`).
