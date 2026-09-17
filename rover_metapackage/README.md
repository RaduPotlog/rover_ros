# rover_metapackage

Metapackage for the Rover A1 ROS stack. Building it pulls in the right entry point for the
build type. The package also holds the `vcs` repository lists for the rover's external
repositories.

## Dependencies

The dependencies are selected by `ROVER_ROS_BUILD_TYPE`:

| `ROVER_ROS_BUILD_TYPE` | Depends on |
|------------------------|------------|
| `hardware` | `rover_bringup` |
| `simulation` | `rover_gazebo` |

Its version (`ros2 pkg xml -t version rover_metapackage`) is shown as the "ROS Driver Version"
in the `rover_bringup` banner.

## Repository lists

| File | Repositories |
|------|--------------|
| `hardware_deps.repos` | `rover_modbus`, `rover_cppuprofile` (both `master`) |
| `simulation_deps.repos` | none (the file has a `repositories:` key with no entries) |

The transport packages (`rover_asio_cmake_module`, `rover_io_context`,
`rover_serial_driver`, `rover_udp_driver`) are **vendored in-tree** under
`rover_transport/`, not pulled by `vcs`. They are a hard fork of
[ros-drivers/transport_drivers](https://github.com/ros-drivers/transport_drivers)
v1.2.0 - see `rover_transport/README.md`.

```bash
cd ~/ros2_ws/rover_a1
export ROVER_ROS_BUILD_TYPE=hardware   # or simulation
vcs import src < src/rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos
rosdep install --from-paths src -y -i
colcon build --symlink-install
```

`rover_docker/rovera1_app/Dockerfile` runs the same `vcs import` with `hardware_deps.repos`.
`rover_orchestrator` (Nav 2) is not listed, so clone it separately on the navigation computer.
