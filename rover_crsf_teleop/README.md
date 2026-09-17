# rover_crsf_teleop

RC teleop for the rover over a CRSF (ExpressLRS) receiver: raw serial bytes are decoded into RC
frames, stick positions become velocity commands for `twist_mux`, two switches drive the hardware
interface's software E-Stop, and an RC link failsafe stops the rover when the transmitter link is
lost.

## How RC input reaches this node

The UART is **not** opened here. `rover_serial_driver`'s `rover_serial_bridge_node` node (from `rover_transport`)
owns the port and publishes raw bytes; this node subscribes to them and decodes CRSF in-process.
The launch file starts both.

```
ELRS receiver ──UART @460800──▶ rover_serial_bridge_node ──rc/raw (UInt8MultiArray)──▶ rover_crsf_teleop_node
                                                                              │
                                        teleop_elrs_cmd_vel_stamped ◀─────────┤
                                        hardware_interface/sw_* (Trigger) ◀───┤
                                        rc/channels, rc/link (echo) ◀─────────┘
```

Decoding in this node rather than in a separate receiver process means the decode runs on the
executor thread — the same thread as the control timer — so `TeleopUseCase` keeps its
single-threaded invariant with no locking, and RC input no longer crosses a DDS hop to reach the
code that acts on it.

**Baud rate.** ASIO supports 460800 natively, which is what the A1's receiver is flashed for.
CRSF's own default of 420000 is *not* in ASIO's table, so a receiver reflashed to 420000 could not
be opened by `rover_serial_bridge_node` at all. Keep it at 460800.

## Interfaces

| Direction | Name | Type |
|---|---|---|
| sub | `rc/raw` | `std_msgs/UInt8MultiArray` (reliable, depth 100) — raw CRSF bytes from `rover_serial_bridge_node` |
| pub | `teleop_elrs_cmd_vel_stamped` | `geometry_msgs/TwistStamped`, frame `base_link` |
| pub | `rc/channels` | `rover_msgs/RcChannels` (best effort) — echo, for tuning only |
| pub | `rc/link` | `rover_msgs/RcLinkStatus` (best effort) — echo, for tuning only |
| client | `hardware_interface/sw_user_e_stop_set` / `_reset` | `std_srvs/Trigger` |
| client | `hardware_interface/sw_e_stop_latch_reset` | `std_srvs/Trigger` |

`rc/channels` and `rc/link` are observability echoes — nothing on the rover consumes them, and
they can be turned off with `publish_rc_topics`. They are plain (not lifecycle) publishers on
purpose, so they keep publishing when the node is deactivated to investigate why it was
deactivated. Channel N is `channels[N-1]`.

## Behaviour

- **Lifecycle node.** `configure` reads parameters, subscribes to the byte stream and resets the
  parser; `activate` starts the 20 ms control loop; `deactivate` publishes one zero command and
  stops. Input is decoded even while inactive, so the link is already known healthy (or not) the
  moment teleop is activated. The launch file brings it straight to active (`autostart`).
- **Zero once.** A centred stick maps to exactly 0.0 (see `domain/stick_mapping.hpp`) and a zero
  command is published only once, so an idle transmitter doesn't hold `twist_mux` on this source.
- **RC link failsafe.** The link is healthy while decoded frames are fresh (`channel_timeout_ms`)
  and — with `require_link_stats` — link statistics are fresh (`link_stats_timeout_ms`) and the
  uplink link quality has not dropped below `link_quality_lost_below` (it recovers at
  `link_quality_recovered_at`). When it isn't, the node publishes **one zero command and goes
  silent**: the rover stops at once and `twist_mux` falls through to its next source. The E-Stop
  is not triggered. Switches are ignored while the link is lost.
- **Switches.** Only a change of switch position fires a service call: e-stop channel low = set,
  high = reset; latch-reset channel low = reset latch. Their resting position is learned over
  `switch_settle_frames` ticks at startup and never fires.
- **Diagnostics** (hardware ID `RC Receiver`) never report ERROR — RC teleop is optional.
  `RC serial link` covers the byte stream (is `rover_serial_bridge_node` alive?), `RC link` the RC signal
  (is the transmitter in range?), and they fail for different reasons.

### Known limitation: the serial bridge does not reconnect

`rover_serial_driver` closes the port on any read error and never reopens it, and its `on_configure`
fails outright if the device is absent. So an unplugged receiver means no RC until the bridge is
cycled:

```bash
ros2 lifecycle set /rover/rover_crsf_serial_bridge cleanup
ros2 lifecycle set /rover/rover_crsf_serial_bridge configure
ros2 lifecycle set /rover/rover_crsf_serial_bridge activate
```

This is not a regression — the receiver package this replaced caught the error and then ran blind
forever, invisibly — and the failsafe handles the consequence safely. The `RC serial link`
diagnostic makes it visible; automatic recovery belongs in a supervisor, not here.

All parameters, with the reasoning behind their defaults, are documented in
[`config/rover_crsf_teleop.yaml`](config/rover_crsf_teleop.yaml). The link failsafe values are
initial and need tuning on the rover (`ros2 topic hz /rover/rc/link`,
`ros2 topic echo /rover/rc/link`).

## Layout (Clean Architecture)

```
include/rover_crsf_teleop/
├── domain/          stick_mapping, switch_debouncer, link_monitor, rc_frame, ports  (no ROS)
│   └── crsf/        crsf_protocol, crc8, crsf_parser - the wire decoder             (no ROS)
├── application/     teleop_use_case - the per-tick rules above                      (no ROS)
└── infrastructure/  ROS adapters for the ports + RoverCrsfTeleopNode (lifecycle)
```

The CRSF decoder sits in `domain/` because it is pure byte math: no clock, no syscalls, no ROS
types. Freshness is `LinkMonitor`'s job, not the parser's.

## Launch

```bash
ros2 launch rover_crsf_teleop rover_crsf_teleop.launch.py   # serial bridge + teleop
ros2 lifecycle get /rover/rover_crsf_serial_bridge
ros2 lifecycle get /rover/rover_crsf_teleop_node
```

## Tests

```bash
colcon test --packages-select rover_crsf_teleop && colcon test-result --all --verbose
```

`test/unit/` covers the domain (including the CRSF decoder, against byte vectors captured from
the implementation it replaces) and the application with fake ports; `test/integration/` drives
the lifecycle node with **encoded CRSF bytes** over real topics and services, so it exercises the
wire format and the parser as well as the teleop rules; `test/e2e/` starts the installed node
from a launch description with no serial bridge, which is also the test that it degrades rather
than failing when the receiver is absent.

## Attribution

The CRSF decoder (`domain/crsf/crc8.*`, `domain/crsf/crsf_parser.*`) is derived from
[`ros2_crsf_receiver`](https://github.com/AndreyTulyakov/ros2_crsf_receiver) by Andrey Tulyakov,
MIT licensed. The CRSF wire constants in `domain/crsf/crsf_protocol.hpp` were restated from the
Team Black Sheep specification (BSD-2-Clause) rather than copied from the upstream
GPL-licensed flight-controller header.
