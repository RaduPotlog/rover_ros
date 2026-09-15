# rover_crfs_teleop

RC teleop for the rover over a CRSF (ExpressLRS) receiver: stick positions become velocity
commands for `twist_mux`, two switches drive the hardware interface's software E-Stop, and an RC
link failsafe stops the rover when the transmitter link is lost.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| sub | `rc/channels` | `crsf_receiver_msg/CRSFChannels16` (best effort) |
| sub | `rc/link` | `crsf_receiver_msg/CRSFLinkInfo` (best effort) |
| pub | `teleop_elrs_cmd_vel_stamped` | `geometry_msgs/TwistStamped`, frame `base_link` |
| client | `hardware_interface/sw_user_e_stop_set` / `_reset` | `std_srvs/Trigger` |
| client | `hardware_interface/sw_e_stop_latch_reset` | `std_srvs/Trigger` |

## Behaviour

- **Lifecycle node.** `configure` reads parameters and subscribes; `activate` starts the 20 ms
  control loop; `deactivate` publishes one zero command and stops. The launch file brings it
  straight to active (`autostart`). A supervisor can deactivate it to take RC off the command
  path without killing the process.
- **Zero once.** A centred stick maps to exactly 0.0 (see `domain/stick_mapping.hpp`) and a zero
  command is published only once, so an idle transmitter doesn't hold `twist_mux` on this source.
- **RC link failsafe.** The link is healthy while `rc/channels` is fresh (`channel_timeout_ms`)
  and - with `require_link_stats` - `rc/link` is fresh (`link_stats_timeout_ms`) and the uplink
  link quality has not dropped below `link_quality_lost_below` (it recovers at
  `link_quality_recovered_at`). When it isn't, the node publishes **one zero command and goes
  silent**: the rover stops at once and `twist_mux` falls through to its next source. The E-Stop
  is not triggered. Switches are ignored while the link is lost.
- **Switches.** Only a change of switch position fires a service call: e-stop channel low = set,
  high = reset; latch-reset channel low = reset latch. Their resting position is learned over
  `switch_settle_frames` ticks at startup and never fires.

All parameters, with the reasoning behind their defaults, are documented in
[`config/rover_crfs.yaml`](config/rover_crfs.yaml). The link failsafe values are initial and need
tuning on the rover (`ros2 topic hz /rover/rc/link`, `ros2 topic echo /rover/rc/link`).

## Layout (Clean Architecture)

```
include/rover_crfs_teleop/
├── domain/          stick_mapping, switch_debouncer, link_monitor, rc_frame, ports  (no ROS)
├── application/     teleop_use_case - the per-tick rules above                     (no ROS)
└── infrastructure/  ROS adapters for the ports + RoverCrfsTeleopNode (lifecycle)
```

## Launch

```bash
ros2 launch rover_crfs_teleop rover_crfs_teleop.launch.py   # receiver + teleop
ros2 lifecycle get /rover/rover_crfs_teleop_node
```

## Tests

```bash
colcon test --packages-select rover_crfs_teleop && colcon test-result --all --verbose
```

`test/unit/` covers domain and application with fake ports, `test/integration/` runs the
lifecycle node against real topics and services, and `test/e2e/` starts the installed node from a
launch description.
