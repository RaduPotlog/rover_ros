# rover_twist_mux

Arbitrates the rover's velocity command sources with [`twist_mux`](https://github.com/ros-teleop/twist_mux).

It also provides `rover_motion_lock_node`, the adapter that lets the rover's safety IO gate the mux.

## Command arbitration

`twist_mux` is the **sole publisher of `cmd_vel` (`/rover/cmd_vel`)**. Every velocity source feeds it as an input,
highest priority wins, and each input has a 0.5 s timeout:

| Priority | Input | Topic | Source |
|---------:|-------|-------|--------|
| 100 | `joystick` | `teleop_foxglove_cmd_vel_stamped` | Foxglove |
| 10 | `cmd_elrs` | `teleop_elrs_cmd_vel_stamped` | ELRS RC teleop |
| 8 | `driver_interface` | `teleop_driver_interface_cmd_vel_stamped` | Driver UI (rover_drive_interface) Manual mode |
| 5 | `nav` | `nav_cmd_vel_stamped` | Nav 2, on the separate orchestrator computer |

Nav 2 sits lowest so every teleop source preempts autonomy deterministically, rather than
them racing each other on `cmd_vel`. The timeout also makes a LAN partition a defined
transition: the nav input goes stale and the mux falls through, instead of relying on
`diff_drive_controller`'s own `cmd_vel_timeout` as a backstop.

## Motion lock

`rover_motion_lock_node` subscribes `hardware_interface/safety_status`
(`rover_msgs/SafetyStatus`) and `hardware_interface/safety_command_echo`
(`rover_msgs/SafetyCommandEcho`) and
publishes `motion_lock` (`std_msgs/Bool`), which `twist_mux` consumes as a lock. It exists
because `twist_mux` locks are typed `std_msgs/Bool` and nothing in the stack published one, so
the safety IO could not gate the mux at all.

**It is fail-safe closed.** The lock is asserted — motion inhibited — when:

- any enabled E-Stop condition is active (see `config/rover_motion_lock.yaml`),
- either safety topic has not been received yet (startup),
- either has gone stale beyond `gpio_timeout` (the hardware interface died), or
- `SafetyStatus.link_healthy` is false — the hardware interface is alive but its link to the
  safety PLC is not, so the values it is publishing are last-known-good rather than current.
  Staleness alone cannot catch this, because the messages keep arriving.

Both halves are required before any decision is made: acting on one alone would read the missing
half's stop conditions as "not active".

Two former `gpio_state` fields are deliberately *not* lock conditions, and the message split now
makes both exclusions structural rather than a matter of remembering a comment — neither appears
in `SafetyStatus` at all:

- `sw_e_stop_latch_reset` — a command pulse that clears the latch, not a state.
- `cpu_wdg_heartbeat` — an output the safety controller toggles to feed the relay's CPU watchdog.
  It is a liveness square wave, not a fault flag; gating on it made `motion_lock` oscillate at the
  heartbeat rate. A stalled heartbeat is caught by the safety relay, which latches the E-Stop, and
  that *is* a lock condition (`use_sw_e_stop_latch_status`).

The two `sw_*` stop requests this node *does* gate on come from `SafetyCommandEcho`. That is the
safe direction for an echo: "we asked for a stop" is a sound reason to inhibit, and it is visible
a poll or two before the latch it causes. Nothing here reads an echo as evidence of plant state.

`twist_mux` itself treats a *stale lock topic* as locked, so if `rover_motion_lock_node` dies the mux
closes rather than opens. The node therefore republishes at `publish_frequency` (10 Hz) to stay
well inside the lock's 0.5 s timeout, and is launched alongside the mux.

A consequence worth knowing: with no hardware interface running there is no safety state, so the
rover will not accept velocity commands at all. That is intended — no safety IO means no driving.
Simulation is unaffected, as it does not launch this package.

At the default lock priority of 200 an active E-Stop stops the rover outright. To gate autonomy
only and leave all teleop sources free, lower it to 7; see the comment in
`config/rover_twist_mux.yaml`.

## Config Files

- `rover_twist_mux.yaml` - Twist mux's configuration (inputs and the safety lock).
- `rover_motion_lock.yaml` - Which safety-IO pins contribute to the lock, and its timing.

## Launch Files

- `rover_twist_mux.launch.py` - Activates Rover's twist mux and the motion lock node.
