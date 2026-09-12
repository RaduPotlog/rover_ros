# rover_twist_mux

This package is responsible for activates twist mux.

It also provides `motion_lock_node`, the adapter that lets the rover's safety IO gate the mux.

## Command arbitration

`twist_mux` is the **sole publisher of `/cmd_vel`**. Every velocity source feeds it as an input,
highest priority wins, and each input has a 0.5 s timeout:

| Priority | Input | Topic | Source |
|---------:|-------|-------|--------|
| 100 | `joystick` | `teleop_foxglove_cmd_vel_stamped` | Foxglove |
| 10 | `cmd_elrs` | `teleop_elrs_cmd_vel_stamped` | ELRS RC teleop |
| 5 | `nav` | `nav_cmd_vel_stamped` | Nav 2, on the separate orchestrator computer |

Nav 2 sits lowest so either teleop source preempts autonomy deterministically, rather than the
two racing each other on `/cmd_vel`. The timeout also makes a LAN partition a defined
transition: the nav input goes stale and the mux falls through, instead of relying on
`diff_drive_controller`'s own `cmd_vel_timeout` as a backstop.

## Motion lock

`motion_lock_node` subscribes `hardware_interface/gpio_state` (`rover_msgs/GpioState`) and
publishes `motion_lock` (`std_msgs/Bool`), which `twist_mux` consumes as a lock. It exists
because `twist_mux` locks are typed `std_msgs/Bool` and nothing in the stack published one, so
the safety IO could not gate the mux at all.

**It is fail-safe closed.** The lock is asserted — motion inhibited — when:

- any enabled E-Stop condition is active (see `config/rover_motion_lock.yaml`),
- no `gpio_state` has been received yet (startup), or
- `gpio_state` has gone stale beyond `gpio_timeout` (the hardware interface died).

`twist_mux` itself treats a *stale lock topic* as locked, so if `motion_lock_node` dies the mux
closes rather than opens. The node therefore republishes at `publish_frequency` (10 Hz) to stay
well inside the lock's 0.5 s timeout, and is launched alongside the mux.

A consequence worth knowing: with no hardware interface running there is no `gpio_state`, so the
rover will not accept velocity commands at all. That is intended — no safety IO means no driving.
Simulation is unaffected, as it does not launch this package.

At the default lock priority of 200 an active E-Stop stops the rover outright. To gate autonomy
only and leave both teleop sources free, lower it to 7; see the comment in
`config/rover_twist_mux.yaml`.

## Config Files

- `rover_twist_mux.yaml` - Twist mux's configuration (inputs and the safety lock).
- `rover_motion_lock.yaml` - Which safety-IO pins contribute to the lock, and its timing.

## Launch Files

- `rover_twist_mux.launch.py` - Activates Rover's twist mux and the motion lock node.
