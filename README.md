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

One step: write the **rover-pc setup** block into `~/.bashrc`. It covers ROS, the workspace
overlay, `ROVER_*`, and the Zenoh settings to reach the rover at `192.168.1.201`. It's safe to
re-run; see [rover_scripts/README.md](rover_scripts/README.md) for the options.

```bash
src/rover_ros/rover_scripts/setup_rover_pc.sh
source ~/.bashrc
```

Or by hand:

```bash
export ROS_DISTRO=lyrical
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
src/rover_ros/rover_gazebo/scripts/rover_sim.sh
```

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
