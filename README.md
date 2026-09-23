# rover_ros

Mechatronics Academy's Rover A1 ROS2.

**Bringup & simulation**

- [`rover_bringup`](rover_bringup/README.md) - top-level launch files that start the whole
  stack on real hardware.
- [`rover_gazebo`](rover_gazebo/README.md) - Gazebo Sim bringup: the robot with
  `gz_ros2_control`, the same controllers, EKFs and twist_mux as the real rover, simulated
  lidar and GNSS, the ROS–Gazebo bridge and RViz.
- [`rover_world`](rover_world/README.md) - Gazebo Sim worlds for the simulation.
- [`rover_description`](rover_description/README.md) - URDF/xacro model: links, joints,
  meshes, sensors and the `<ros2_control>` hardware components.
- [`rover_metapackage`](rover_metapackage/README.md) - metapackage pulling in the entry point
  for the build type, plus the `vcs` `.repos` dependency lists.

**Control & hardware**

- [`rover_hardware_interface`](rover_hardware_interface/README.md) - ros2_control plugins
  `RoverA1System` (wheel motors and safety controller) and `PhidgetImuSensor`.
- [`rover_controller`](rover_controller/README.md) - ros2_control controller configuration
  and launch.
- [`rover_twist_mux`](rover_twist_mux/README.md) - velocity command arbitration with
  `twist_mux`, plus `rover_motion_lock_node`.
- [`rover_crsf_teleop`](rover_crsf_teleop/README.md) - CRSF (ExpressLRS) RC teleop, software
  E-Stop switches and RC link failsafe.

**Localization**

- [`rover_localization`](rover_localization/README.md) - `robot_localization` EKFs fusing
  wheel odometry and IMU, optionally GPS (`ROVER_USE_GPS`).
- [`rover_gps_heading`](rover_gps_heading/README.md) - ENU heading from GNSS course for
  `navsat_transform_node`.

**Safety, power & status**

- [`rover_safety`](rover_safety/README.md) - behavior-tree safety supervision: software
  E-Stop, shutdown and LED animation selection.
- [`rover_battery`](rover_battery/README.md) - decodes BMS telemetry received over UDP and
  publishes the battery state.
- [`rover_led`](rover_led/README.md) - SK9822 bumper LED panels: animation controller and
  driver.
- [`rover_diag_manager`](rover_diag_manager/README.md) - the rover's diagnostic nodes.

**Interfaces, transport & utilities**

- [`rover_msgs`](rover_msgs/README.md) - custom messages and services.
- [`rover_utils`](rover_utils/README.md) - shared header-only C++ utilities and Python launch
  helpers.
- [`rover_transport`](rover_transport/README.md) - hard fork of `transport_drivers`: serial,
  UDP and Modbus TCP drivers on top of `rover_io_context`.
- [`rover_modbus`](rover_modbus/README.md) - vendored Modbus library for modern C++.

**Not ROS packages**

- [`rover_arch`](rover_arch/README.md) - firmware architecture docs and diagrams, including
  the safety chain.
- [`rover_foxglove`](rover_foxglove/README.md) - Foxglove dashboard layout for the `rover`
  namespace.
- [`rover_scripts`](rover_scripts/README.md) - development PC helper scripts
  (`setup_rover_pc.sh`).

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
