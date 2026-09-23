# rover_twist_mux

Arbitrates the rover's velocity command sources with [`twist_mux`](https://github.com/ros-teleop/twist_mux).

It also provides two adapters, launched alongside the mux:

- `rover_motion_lock_node` lets the rover's safety IO gate the mux.
- `rover_command_freshness_node` passes the Driver UI's commands to the mux only while they are
  fresh, so a Wi-Fi stall cannot make the rover replay old motion.

## Command arbitration

`twist_mux` is the **sole publisher of `cmd_vel` (`/rover/cmd_vel`)**. Every velocity source feeds it as an input,
highest priority wins, and each input times out:

| Priority | Input | Topic | Timeout | Source |
|---------:|-------|-------|--------:|--------|
| 110 | `cmd_elrs` | `teleop_elrs_cmd_vel_stamped` | 0.5 s | ELRS RC teleop |
| 100 | `joystick` | `teleop_foxglove_cmd_vel_stamped` | 0.5 s | Foxglove |
| 8 | `driver_interface` | `teleop_driver_interface_cmd_vel_fresh_stamped` | 0.3 s | Driver UI (rover_drive_interface) Manual mode, via `rover_command_freshness_node` |
| 5 | `nav` | `nav_cmd_vel_stamped` | 0.5 s | Nav 2, on the separate orchestrator computer |

The RC transmitter sits highest, so the operator with line of sight always has the last word;
an idle transmitter publishes a short zero burst and then nothing, so it does not hold the mux.
The Driver UI's input times out after 0.3 s (three periods of its 10 Hz stream), so a dead
browser or Wi-Fi link stops the rover sooner. Nav 2
sits lowest so every teleop source preempts autonomy deterministically, rather than
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

## Command freshness

twist_mux judges an input by when a message *arrives*. The Driver UI's commands cross a websocket
(TCP), which holds them through a Wi-Fi stall and then delivers them in a burst, so the mux would
replay seconds-old motion. `rover_command_freshness_node` sits between the UI's topic
(`teleop_driver_interface_cmd_vel_stamped`) and the mux input
(`teleop_driver_interface_cmd_vel_fresh_stamped`) and drops commands that arrive late.

The browser's clock can be seconds away from the rover's, so the node does not check
`now - stamp`. It tracks the **baseline**, the smallest recent `receive time - stamp` (clock
offset plus best-case transport delay), and drops any command more than `max_delay` (0.3 s) beyond
it. The logic is in `domain/command_freshness_filter.hpp`:

- The baseline follows clock drift at up to `max_clock_drift` (1 ms/s) while commands flow, and
  never loosens during silence, so a burst after a long stall is still judged against the
  pre-stall baseline. A backlog growing faster than that is dropped once it is 0.3 s behind.
- If the browser's clock steps backwards, every command looks late. After `resync_gap` (1 s) of
  silence, i.e. a new Manual session, a steady run of late commands re-baselines the filter
  after `resync_time` (2 s). A stale burst arrives within milliseconds and a mid-stream backlog
  has no preceding silence, so neither can resync. Limitation: a delay that is steady for 2 s
  after silence looks the same as a clock step, and is accepted.
- Unstamped commands are dropped.
- Receive time is the wall clock, not the node clock, so it works in simulation (sim time).
- Diagnostics task `Command freshness`: WARN while it is dropping commands, with counts, the
  baseline and the last excess delay.

## Config Files

- `rover_twist_mux.yaml` - Twist mux's configuration (inputs and the safety lock).
- `rover_motion_lock.yaml` - Which safety-IO pins contribute to the lock, and its timing.
- `rover_command_freshness.yaml` - The Driver UI command freshness filter: topics, `max_delay`,
  drift and resync settings.

## Launch Files

- `rover_twist_mux.launch.py` - Activates Rover's twist mux, the motion lock node and the command
  freshness node.
