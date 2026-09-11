# rover_diag_manager

Package contains Rover's diagnostic nodes.

## Architecture

`rover_diag_manager_node` follows this workspace's Clean Architecture layout:

- `domain/` — `SystemSample`, `SystemHealthThresholds`, `HealthReport`, the pure
  `evaluateSystemHealth()` grading function, and the `SystemMetricsSourcePort` /
  `SystemStatusPublisherPort` ports. No ROS or cppuprofile dependency.
- `application/` — `MonitorSystemUseCase`: samples the system once per tick, grades it, and
  publishes both through the output port.
- `infrastructure/` — `LinuxSystemMetricsSource` (cppuprofile + `/sys` + `std::filesystem`),
  `Ros2SystemStatusPublisher` (publishes `system_status`, feeds the `"OS status"`
  `diagnostic_updater` task), and `SystemDiagNode`, the composition root.

Plain (non-lifecycle) node: it owns no resource, only reads ephemeral OS counters each tick.

## Config Files

- `system_diag.yaml` - Rover system diagnostic node's configuration, keyed under
  `rover_diag_manager_node` (the node's runtime name — see `launch/system_diag.launch.py`).
  **This key must match the node's runtime name exactly** (or use a `/**/` wildcard prefix, as
  here) — a mismatch fails silently: `ros2 launch` starts fine and the node falls back to its
  compiled-in defaults with no warning that the file was never applied.

## Launch Files

- `system_diag.launch.py` - Loads the Rover's system diagnostic node.
