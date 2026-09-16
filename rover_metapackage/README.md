# rover_metapackage

Metapackage for the Rover A1 ROS stack. Building it pulls in the right entry point for the
build type. The package also holds the `vcs` repository lists for the rover's external
repositories.

## Dependencies

The dependencies are selected by `ROVER_ROS_BUILD_TYPE`:

| `ROVER_ROS_BUILD_TYPE` | Depends on |
|------------------------|------------|
| `hardware` | `rover_bringup`, `rover_gps`, `nmea_navsat_driver` |
| `simulation` | `rover_gazebo` |

Its version (`ros2 pkg xml -t version rover_metapackage`) is shown as the "ROS Driver Version"
in the `rover_bringup` banner.

## Repository lists

| File | Repositories |
|------|--------------|
| `hardware_deps.repos` | `rover_modbus`, `rover_cppuprofile`, `rover_crsf_receiver`, `rover_arch`, `rover_foxglove`, `rover_transport`, `rover_network_monitor`, `rover_cockpit_ros2_diagnostics`, `rover_nmea_navsat_driver` (all `master`) |
| `simulation_deps.repos` | none (empty) |

```bash
cd ~/ros2_ws/rover_a1
export ROVER_ROS_BUILD_TYPE=hardware   # or simulation
vcs import src < src/rover_ros/rover_metapackage/${ROVER_ROS_BUILD_TYPE}_deps.repos
rosdep install --from-paths src -y -i
colcon build --symlink-install
```

`rover_docker/rovera1_app/Dockerfile` runs the same `vcs import` with `hardware_deps.repos`.
`rover_orchestrator` (Nav 2) is not listed, so clone it separately on the navigation computer.
