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
| pub | `rc/calibration/state` | `rover_msgs/RcCalibrationState` (reliable, **transient local**) |
| srv | `rc/calibration/start` | `rover_msgs/StartRcCalibration` |
| srv | `rc/calibration/sweep` / `finish` / `cancel` | `std_srvs/Trigger` |
| srv | `rc/calibration/apply` | `rover_msgs/SetRcCalibration` |
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
- **Zero burst.** A centred stick maps to exactly 0.0 (see `domain/stick_mapping.hpp`). Zeros are
  published for `zero_burst_duration_ms` (300 ms in the config; 0 = a single zero) and then not
  at all, so one lost message cannot leave the rover moving and an idle transmitter doesn't hold
  `twist_mux` on this source.
- **Expo.** `linear_x_expo` / `angular_z_expo` (0.3 / 0.5 in the config, 0 = linear) shape each
  stick as `(1 - e)·m + e·m³` after the deadband: small moves give much less speed, full stick is
  still `out_min`/`out_max`. Same curve as the web joystick. It stacks with any expo in the
  transmitter model, so keep the radio's expo at 0 %.
- **RC link failsafe.** The link is healthy while decoded frames are fresh (`channel_timeout_ms`)
  and — with `require_link_stats` — link statistics are fresh (`link_stats_timeout_ms`) and the
  uplink link quality has not dropped below `link_quality_lost_below` (it recovers at
  `link_quality_recovered_at`). When it isn't, the node publishes **a short zero burst and goes
  silent**: the rover stops at once and `twist_mux` falls through to its next source. The E-Stop
  is not triggered. Switches are ignored while the link is lost.
- **Switches.** Only a change of switch position fires a service call: e-stop channel low = set,
  high = reset; latch-reset channel low = reset latch. Their resting position is learned over
  `switch_settle_frames` ticks at startup and never fires.
- **Diagnostics** (hardware ID `RC Receiver`) never report ERROR — RC teleop is optional.
  `RC serial link` covers the byte stream (is `rover_serial_bridge_node` alive?), `RC link` the RC signal
  (is the transmitter in range?), and they fail for different reasons.

## RC calibration

Stick endpoints and centre are **per channel** (`channel_in_min` / `channel_in_mid` /
`channel_in_max` / `channel_deadband` are 16-element arrays, channel N at index N-1). They used
to be single values shared by both axes, which forced one deadband wide enough for the worse of
the two sticks — on this transmitter ch3 rests at 1004 and ch1 at 987.

Measurement happens **in this node**, not in whatever is driving it: `rc/channels` is best-effort
depth 1 at 50 Hz, so a client tracking min/max over the topic would miss the peaks. The flow is:

```
deactivate ──▶ start(e_stop_confirmed) ──▶ sweep ──▶ finish ──▶ apply ──▶ activate
                      kCenter              kSweep    kReview     kIdle
```

1. **Press the physical E-Stop and deactivate the node.** Three independent interlocks, because
   the sweep drives the sticks to full throw and RC teleop is not the only thing that can command
   this rover:
   - the node reads `hardware_interface/safety_status` itself and refuses unless the **physical
     button is pressed, the PLC has latched, and the motor contactor has opened** — all three.
     A software E-Stop (RC switch, service call) does not count even though it also sets the
     latch: it can be cleared remotely while the operator is standing at the rover, and the
     physical button cannot, because the PLC latch is set-dominant. Not the operator's word for
     it either, and **"cannot verify" refuses too** — no `rover_hardware_interface`, no
     calibration, on a bench or anywhere else;
   - the operator still confirms it, which the node also requires. Evidence and intent are
     separate conditions: the topic can be right while nobody is standing at the rover;
   - `start` refuses while the node is ACTIVE, and `on_activate` refuses while a session runs.

   Releasing the E-Stop mid-session cancels it, after a one-second grace window — a Modbus read
   error surfaces as "clear", so a single sample is not enough to throw away the measurement.
2. **`start`**, with `e_stop_confirmed: true`. The node also inhibits teleop itself for the whole
   session (`TickStatus::kInhibited`): one zero command, then silence, and the switches are not
   evaluated at all.
3. **Release every stick** while the centre is sampled. The mean gives `channel_in_mid`; the
   peak-to-peak spread of the same samples sizes `channel_deadband`, which is the hand procedure
   the config file used to describe, automated.
4. **`sweep`**, then move every stick and switch to both extremes. Peak-held per channel; a
   channel that never travels more than `kMinTravel` keeps its previous values rather than being
   overwritten with a range a few counts wide.
5. **`finish`**, then **`apply`** (with `persist: true` to save it). Applying rebuilds
   `TeleopUseCase` in place — **nothing restarts** — and mirrors the values into the node's own
   parameters so `ros2 param get` agrees with what the rover is using.

Watch it on `rc/calibration/state`, which is transient-local so a client connecting mid-session
immediately learns that teleop is held off. An abandoned session self-cancels after
`calibration_timeout_s`, and the `RC calibration` diagnostic reports one in progress as WARN —
from the outside, a session left running looks exactly like a rover that has stopped responding
to the transmitter.

**Persistence.** `calibration_file` (`/config/rover_crsf_teleop/rc_calibration.yaml`, a balena
named volume) overrides the `channel_in_*` parameters per channel when it loads, because it
describes the transmitter that is actually plugged in. Which source is in force is logged at
startup and shown in the diagnostic, never guessed at. Delete the file to go back to the shipped
values. Leave `calibration_file` empty to turn persistence off entirely.

**Switch channels are not calibrated as axes.** A two-position switch rests at one end of its
travel, so its measured centre *is* its endpoint; checking it the way a stick is checked would
fail every time. `channel_switch_threshold` stays an absolute raw value — but a calibration that
leaves it outside a switch channel's measured range now produces a startup warning, which is how
a switch that would silently read one position forever becomes visible.

**After applying, the E-Stop switch is inert for `switch_settle_frames` ticks** (2 s by default).
That is deliberate: the sweep moved the switches while nothing was watching, so the debouncers
are re-armed and re-learn where they rest, instead of reading the first frame afterwards as a
real edge and firing an E-Stop call the operator never asked for.

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
│   │                rc_calibration - measures this transmitter's endpoints          (no ROS)
│   └── crsf/        crsf_protocol, crc8, crsf_parser - the wire decoder             (no ROS)
├── application/     teleop_use_case - the per-tick rules above                      (no ROS)
│                    calibration_use_case - the session, its gate and its timeout    (no ROS)
└── infrastructure/  ROS adapters for the ports + RoverCrsfTeleopNode (lifecycle)
                     yaml_calibration_store - the persisted calibration
```

The CRSF decoder sits in `domain/` because it is pure byte math: no clock, no syscalls, no ROS
types. Freshness is `LinkMonitor`'s job, not the parser's.

The `(no ROS)` above is checked, not just intended. `rover_crsf_teleop_core` is built from
`domain/` and `application/` alone and links no ROS target, and `test/layer_purity.cmake` (run as
the `test_layer_purity` test) fails the build if either layer ever includes `rclcpp`, a message
package, yaml-cpp or the filesystem - a header-only include would otherwise compile quietly and
take the boundary with it. Infrastructure is split in two: `rover_crsf_teleop_adapters` holds the
port adapters and the conversions, `rover_crsf_teleop_ros` holds the node, so an adapter's unit
test does not link the node.

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
