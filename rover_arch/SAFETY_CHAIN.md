# Rover A1 — E-Stop / safety chain

How the rover stops, who can stop it, and what software can and cannot know about
it. Written because none of this was recorded anywhere: before this document the
string "PLC" appeared exactly once in the entire workspace, in a header comment
in `rover_twist_mux/include/rover_twist_mux/domain/motion_lock_policy.hpp`, and
the Modbus object map existed only as a table literal in
`rover_safety_controller.cpp`.

The authoritative diagram is `rover_a1_arch.drawio`. This file is the prose that
goes with it.

---

## 1. The chain, end to end

```
  SW E-Stop user button ──┐
  SW motor-driver fault ──┤
  CPU watchdog timeout  ──┼──▶  SET (priority)  ┌──────────┐
  HW E-Stop button      ──┘                     │  LATCH   │──▶ contactor coil ──▶ ┤├ motor contactor
                                                │  (SR,    │                        │
  HW reset button       ──┐                     │   set-   │    ┌───────────────────┘
  SW latch reset (pulse)──┴──▶ OR ──▶ RESET ────│ dominant)│    │  auxiliary contact
                                                └──────────┘    ▼
                                                               back to the PLC as
                                                               motor_contactor_engaged
```

Two properties are load-bearing:

* **Set-dominant.** Holding a reset while any trip source is still asserted does
  not re-engage the contactor. The trip wins.
* **Latching.** The stop is held after the trip source clears. Only a deliberate
  reset releases it, so a momentary fault cannot silently self-clear.

The contactor's **auxiliary contact is fed back** to the PLC and is readable by
software. That is what makes a welded contactor detectable; see §5.

---

## 2. Transport and object map

Modbus TCP to the safety controller at `192.168.88.11:502` (URDF
`modbus_host` / `modbus_port`), unit id 255, single-bit function codes only
(FC1 read coil, FC2 read discrete input, FC5 write single coil).

| Signal | Modbus object | Direction | Writable |
|---|---|---|---|
| `hw_e_stop_user_button` | `CONTACT_0` (FC2) | PLC → us | — |
| `motor_contactor_engaged` | `COIL_0` | PLC → us | no |
| `cpu_wdg_heartbeat` | `COIL_1` | us → PLC | yes |
| `sw_e_stop_user_button` | `COIL_2` | us → PLC | yes |
| `sw_e_stop_motor_driver_fault` | `COIL_3` | us → PLC | yes |
| `sw_e_stop_latch_reset` | `COIL_4` | us → PLC (pulse) | yes |
| `sw_e_stop_latch_status` | `COIL_5` | PLC → us | no |

`COIL_0` and `COIL_5` are relay outputs. `CoilInfo::is_coil_engage_allowed` is
`false` for both, and `ModbusDiscreteIoClient::writeDiscreteCoil()` refuses a
write to either.

Source of truth: `rover_hardware_interface/src/rover_safety_controller/rover_safety_controller.cpp`
(the contact and coil tables) and `domain/rover_gpio_types.hpp` (the enum).

---

## 3. Who talks to the PLC

The PLC is reached from inside the `ros2_control` hardware component
`rover_hardware_interface/RoverA1System`, **never from `read()` or `write()`**.
Two background threads inside `ContactCoilHandler` own the link:

| Thread | Period | Job |
|---|---|---|
| watchdog | `safety_wdg_kick_period_ms` (200 ms) | toggles `COIL_1` to feed the relay's watchdog |
| IO poll | `safety_io_poll_period_ms` (100 ms) | reads all 7 objects into a cache |

`read()` copies that cache under a `try_lock` and never performs I/O, which is
what keeps the RT path clean (enforced by `scripts/check_rt_path_purity.sh`).

### Why two threads

They used to be one loop: the heartbeat was written at the end of a sweep that
performed 7 blocking Modbus reads, so the real toggle interval was
`period + 7 × round-trip`. With a 500 ms response timeout, **one slow
transaction pushed the toggle past the relay's ~1 s watchdog window** and latched
a nuisance E-Stop that then needed a manual reset. Separating them, running the
heartbeat on absolute `steady_clock` deadlines, and giving it priority on the
link bounds its worst-case wait to a single in-flight transaction.

`ModbusLink` is that priority lock. A plain (even timed) mutex is not enough:
under sustained slow IO the poll thread re-acquires between transactions and no
standard mutex promises fairness, so a heartbeat tick could lose the race
arbitrarily often. A waiting priority holder blocks *new* poll acquisitions.

### Timing budget

| Stage | Rate |
|---|---|
| `controller_manager`, `read()`/`write()` | 100 Hz |
| `safety_status` / `safety_command_echo` publish | 20 Hz |
| Modbus IO poll | 10 Hz |
| Watchdog heartbeat toggle | 5 Hz |
| Relay watchdog window | ~1 s (measured) |

Every value above except the last is a URDF `<param>` on the `<ros2_control>`
block. The last is a property of the hardware and the reason the others are what
they are — re-measure it before changing `safety_wdg_kick_period_ms`.

---

## 4. What reaches ROS

Two topics, both 20 Hz, `reliable` + `volatile` + `KeepLast(1)`:

| Topic | Message | Meaning |
|---|---|---|
| `hardware_interface/safety_status` | `rover_msgs/SafetyStatus` | plant state: what the hardware is doing |
| `hardware_interface/safety_command_echo` | `rover_msgs/SafetyCommandEcho` | read-backs of coils *we* drive |

**The split is the point.** These were one `GpioState` message of seven
undifferentiated bools, and the distinction between "the contactor is closed" and
"we asked for a stop" survived only as prose comments — which two packages then
re-implemented independently, each hand-writing which fields to exclude and why.

The rule is one-directional:

* Anything in `SafetyStatus` may be used to permit or inhibit.
* Anything in `SafetyCommandEcho` may be used **only to inhibit**. "We asked for
  a stop" is a sound reason not to drive, and it is visible a poll or two before
  the resulting latch. It is never evidence that the hardware did anything.

Commands are **services, not a topic**: `hardware_interface/sw_user_e_stop_set`,
`.../sw_user_e_stop_reset`, `.../sw_e_stop_latch_reset`, all `std_srvs/Trigger`.
A safety command needs an acknowledgement and must not be replayed to a late
joiner, which a latched command topic would do. There is deliberately no
`GpioControl` message.

Neither topic is `transient_local`. It once was, and two consumers commented that
it therefore "cannot go stale" and skipped their staleness checks — a latched
sample tells a late joiner what was true when the publisher last ran, not that it
is still running.

`SafetyStatus.link_healthy` reports whether the Modbus link and both threads are
actually alive. Without it a dead link is indistinguishable from a quiet one,
because the messages keep flowing with last-known-good values in them.

---

## 5. Where motion is gated

Five independent gates, in order from the wire inwards:

1. **The contactor.** Hardware. Everything below is defense in depth.
2. **`rover_twist_mux` / `rover_motion_lock_node`** — maps both safety topics to
   `motion_lock` (`std_msgs/Bool`, priority 200 on the mux). Fail-safe closed:
   locked before the first message, when either topic is stale past
   `gpio_timeout` (1.0 s), and when `link_healthy` is false.
3. **`RoverA1System::write()`** — `RoverControlLoopUseCase::decideWriteCommand()`.
   On inhibit it sends **zeros** rather than going silent, because the Phidget
   DCC1000's own failsafe (`motor_failsafe_timeout_ms`, 500 ms) is fed by
   commands actually reaching the drivers.
4. **The contactor plausibility check** — `ContactorMonitor`. `latch_active &&
   motor_contactor_engaged` sustained past the drop-out tolerance is a latched,
   fatal fault: the stop was commanded and the contacts did not open. This is the
   software half of an EDM loop, and it is the reason the aux contact matters.
5. **`IsMotionLocked`** (`rover_navigation`) — a BT condition, fail-safe on
   staleness, so a navigation tree aborts rather than navigating into a closed
   mux.

Plus the **E-Stop reset invariant** (`EmergencyStop::resetEStop()`): the latch
cannot be cleared while the rover is being commanded to move *or* while the
wheels are still turning. The second half exists because the first is weak on its
own — the wheel PIDs park their command at a frozen I-term whenever motion is
inhibited, which is why the command-side deadband cannot be tightened below
`i_clamp_max` and had drifted to 1.2 rad/s (~0.2 m/s).

---

## 6. Known gaps

* **The PLC cannot say why it tripped.** Four SET sources, one `latch_active`
  bit. `SafetyStatus.latch_cause` exists and is plumbed through, but reads
  `LATCH_CAUSE_UNKNOWN` until the PLC program latches the cause into readable
  coils. This is the single most useful change available on the PLC side: it
  turns "the rover stopped" into "the rover stopped because the watchdog
  expired".
* **`velocity_command_zero_tolerance` is 0.35 rad/s, not 0.01.** Bounded below by
  the PIDs' `i_clamp_max`. Getting to 0.01 means resetting the PID integral when
  motion is inhibited.
* **No end-to-end test** that an asserted `motion_lock` actually stops `cmd_vel`
  at the mux. Every stage either side of the mux is covered; the mux itself is a
  third-party node and testing it needs a launch fixture.
* **`ros2_control`'s own GPIO mechanism is not used.** No `<gpio>` tags, no
  `gpio_controllers`. This is deliberate — the safety IO is polled at 10 Hz
  behind a blocking link and hanging it off the 100 Hz resource manager buys
  nothing — but it is a deviation worth knowing about rather than rediscovering.
* **`read()`/`write()` always return `OK`.** PLC link health reaches diagnostics
  and the topics, never the resource manager, so `controller_manager` cannot
  deactivate on a dead safety link.

---

## 7. Where to look

| Concern | File |
|---|---|
| Modbus object map | `rover_hardware_interface/src/rover_safety_controller/rover_safety_controller.cpp` |
| Threads, priority lock, heartbeat | same file, `ContactCoilHandler` / `ModbusLink` |
| Link + reconnection | `rover_transport/rover_modbus_driver/src/application/modbus_discrete_io_client.cpp` |
| E-Stop domain rules | `rover_hardware_interface/include/rover_hardware_interface/domain/emergency_stop.hpp` |
| Welded-contactor check | `.../domain/contactor_monitor.hpp` |
| Motion gate decision | `.../application/rover_control_loop_use_case.hpp` |
| Motion lock policy | `rover_twist_mux/include/rover_twist_mux/domain/motion_lock_policy.hpp` |
| Message contracts | `rover_msgs/msg/SafetyStatus.msg`, `rover_msgs/msg/SafetyCommandEcho.msg` |
| All timing parameters | `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro` |
