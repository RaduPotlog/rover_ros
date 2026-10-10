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

- `system_diag.yaml` - Rover system diagnostic node's configuration (parameters are declared in
  `src/system_diag_params.yaml`, `generate_parameter_library`), keyed under
  `rover_diag_manager_node` (the node's runtime name — see `launch/system_diag.launch.py`).
  **This key must match the node's runtime name exactly** (or use a `/**/` wildcard prefix, as
  here) — a mismatch fails silently: `ros2 launch` starts fine and the node falls back to its
  compiled-in defaults with no warning that the file was never applied.
- `diagnostic_aggregator.yaml` - `diagnostic_aggregator` analyzers. Groups every rover node's
  `diagnostics` into `diagnostics_agg` under
  `/Rover/{Computer,Drive,Battery,Localization,GPS,Lidar,Lighting,Teleop,Motion,Safety}`, matched by
  the `"<node name>: "` prefix of each status. Statuses from nodes not listed land in `/Rover/Other`;
  add an analyzer when a new node publishes diagnostics. `rover_command_freshness_node`
  (rover_twist_mux, `Command freshness`) has no analyzer yet, so it shows up under `Other`.
- `diagnostic_aggregator_gps.yaml`, `diagnostic_aggregator_lidar.yaml` - the `GPS` and `Lidar`
  groups. `system_diag.launch.py` loads each only while that sensor is enabled (launch arguments
  `use_gps` / `use_lidar`, default `ROVER_SYSTEM_USE_GPS` / `ROVER_SYSTEM_USE_LIDAR`): a disabled
  sensor publishes no diagnostics, and its group would otherwise stay STALE.

## Launch Files

- `system_diag.launch.py` - Loads the Rover's system diagnostic node and the
  `diagnostic_aggregator` node (`aggregator_node`, named `rover_diagnostic_aggregator`), which publishes
  `diagnostics_agg` and `diagnostics_toplevel_state` in the launch namespace. `diagnostics_agg` is
  what the drive UI diagnostics page (`rover_drive_interface`) displays.
  Override the analyzers with `diagnostic_aggregator_config_path:=<file>`.
