# rover_hardware_interface

Package implements system interfaces and sensor interfaces from ros2_control for Rover.

This package exports `hardware_interface` plugins (`RoverA1System`, `PhidgetImuSensor`) only —
it has no launch files or config of its own. The URDF `<ros2_control>` tag that wires these
plugins to joints, and the `controllers.yaml` + bringup launch file that starts
`controller_manager` on top of them, live in the sibling packages:

## Config Files
- `rover_description` — URDF/xacro `<ros2_control>` tag and hardware parameters
- `rover_controller` — `controllers.yaml` controller configuration

## Launch Files
- `rover_controller` — bringup launch file (robot_state_publisher + controller_manager + spawners)