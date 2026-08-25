# Audit: `rover_hardware_interface` (branch `jazzy`)

Full-package audit (no pending diff existed at review time). Performed by the
`ros2-controllers-reviewer` agent on 2026-08-25. Package reviewed in full
(no `test/` directory exists). Cross-checked against
`rover_a1_macro.urdf.xacro`, `imu.urdf.xacro`, and
`wheel_01_controller.yaml` in sibling packages.

## Must fix

**1. [FIXED] `on_error()` dereferences possibly-null members → crash in the error path itself**
`src/rover_system/rover_system.cpp:168-192`
`on_error()` unconditionally calls `e_stop_->setEStop()` (line 171) and later
`rover_driver_->deinitialize()` (line 185). Both `e_stop_` and `rover_driver_`
are only assigned inside `on_configure()`'s `configureRoverDriver()`/
`configureEStop()` (lines 84-91, 382-407), in that order. If
`configureRoverController()` or `configureRoverDriver()` throws (very
plausible — Phidget attach failure, Modbus unreachable), `on_configure()`
returns `ERROR` with `rover_driver_`/`e_stop_` still null, the lifecycle
framework invokes `on_error()`, and it null-derefs. **Failure scenario:** any
hardware bring-up failure (USB not attached, wrong serial, Modbus host down)
segfaults the controller_manager process instead of reporting a clean
`ERROR` state — the exact case `on_error()` exists to handle.

**2. [FIXED] E-stop is never consulted from the RT loop; safety gating is dead code**
- `include/rover_hardware_interface/system_e_stop/system_e_stop.hpp:37,39` +
  `src/system_e_stop/system_e_stop.cpp:39-57` —
  `readEStopState()`/`readEStopLatchState()` are defined but **never called**
  anywhere (confirmed by grep across the package).
- `src/rover_system/rover_system.cpp:238` — `// TODO: Update e-stop state`
  inside `read()`, never resolved.
- `src/rover_system/rover_system.cpp:243-255` — `write()` gates only on
  lifecycle state (`PRIMARY_STATE_ACTIVE`); it never checks e-stop status
  before calling `sendSpeedCmd()`.

**Failure scenario:** the only thing preventing motion during an E-stop is
the physical motor-contactor relay driven by Modbus coils; the ros2_control
software stack has no independent knowledge of E-stop state and will happily
keep writing velocity commands to the Phidget motors the whole time. Any
future refactor that removes/bypasses the hardware relay (or a coil-write
failure) removes the *only* safety interlock.

*Fix:* added `RoverSystem::updateEStopState()`, called every `read()` cycle,
which polls both `EmergencyStop::readEStopState()` (user-triggered) and
`readEStopLatchState()` (latched) and caches the OR of the two in
`e_stop_active_` (`include/.../rover_system.hpp`, defaults `true` —
fail-safe until the first successful read). `write()` now returns early
without calling `handleRoverDriverWriteOperation()`/`sendSpeedCmd()`
whenever `e_stop_active_` is true, so software no longer commands motion
while E-Stop is active, independent of the hardware relay. Note: this does
**not** fix item #3 below — a stale nonzero `hw_commands_velocities_` at the
moment of an E-Stop reset can still be written on the very next cycle.

**3. [FIXED] `zeroVelocityCheck` safety check is constructed but never invoked**
`include/rover_hardware_interface/system_e_stop/system_e_stop.hpp:57,77`,
wired in `src/rover_system/rover_system.cpp:404`
(`std::bind(&RoverSystem::areVelocityCommandsNearZero, this)`), consumed
nowhere in `src/system_e_stop/system_e_stop.cpp`. `resetEStop()` (lines
70-79) clears the E-stop unconditionally. **Failure scenario:** operator
resets E-stop while a nonzero velocity command is still latched in
`hw_commands_velocities_` (e.g., controller never zeroed cmd_vel before the
stop) → rover lurches the instant the contactor re-engages, exactly what
this dead check was clearly meant to prevent.

*Fix:* `EmergencyStop::resetEStop()` (`src/system_e_stop/system_e_stop.cpp`)
now calls `zeroVelocityCheck()` before writing the reset coil and throws
`std::runtime_error` (surfaced to the caller as
`sw_user_e_stop_reset`'s `response.success=false` /
`response.message`, per the existing `ROSServiceWrapper` exception-to-
response mapping in `system_ros_interface.cpp:43-56`) if velocity commands
are not near zero — the reset is refused rather than silently applied.
Since `RoverSystem::write()` (fix for item #2) blocks motion while *either*
the user-trigger or the latch is active, gating only `resetEStop()` is
sufficient: the latch alone being reset (`resetEStopLatch()`, left
unchecked, matching the audit's scope) cannot re-enable motion on its own.

**4. [FIXED] Data race on `PhidgetMotorDriver::state_`**
`src/rover_driver/phidget_driver/phidget_motor_driver.cpp:310-376`
(`positionChangeHandler`/`currentChangeHandler`/`temperatureChangeHandler`,
invoked on the Phidget SDK's internal thread) write
`state_.vel/pos/current/temp` with no synchronization; `readState()` (lines
300-303, called from `PhidgetRoverDriver::updateMotorsState()` →
`RoverSystem::read()` on the RT thread) reads the same fields with no
lock/atomic. Undefined behavior; can surface as torn/stale state feedback
fed straight into `hw_states_*`.

*Fix:* added `state_mtx_` guarding `state_` (`phidget_motor_driver.hpp`).
The three SDK-thread callback handlers now take a blocking
`std::lock_guard` before writing their field(s) of `state_` (cheap, tiny
critical sections, not RT-reachable). `readState()` (RT thread) uses a
**non-blocking** `try_lock` — matching the `ContactCoilHandler` idiom
already used elsewhere in this codebase — and returns a cached
`state_snapshot_` on contention rather than ever blocking the control
loop.

**5. [FIXED] Data race on `PhidgetImuSensor::imu_sensor_state_` — the exact buffer backing exported `StateInterface`s**
`include/.../phidget_imu_sensor.hpp:127` /
`src/rover_sensors/phidget_imu_sensor.cpp:159-168`
(`export_state_interfaces()` hands out raw pointers into this vector) is
written by `updateAllStatesValues()`/`updateAccelerationAndGyrationStateValues()`
(lines 515-534) called from `spatialDataCallback()` on the Phidget SDK
thread, while controller_manager/broadcasters read those same doubles
through the loaned state interfaces on the RT thread — zero synchronization.
`imu_connected_` (line 156, checked in `read()` at line 172) has the same
problem: plain `bool` written from the SDK callback thread, read from RT.

*Fix:* `imu_connected_` is now `std::atomic_bool` — a complete, trivial
fix for that half of the item.

For `imu_sensor_state_` the situation needed more care: it's not our own
`read()` that races with the SDK thread, it's the hardware_interface
framework's `get_value()` on the exported `StateInterface` — code we
don't control, which (because this component uses the deprecated raw
`double*` `Handle` constructor) does no locking against direct writes
into that memory. A mutex on the writer side alone can't create a
happens-before edge with a reader that never takes it. So instead of
locking the exported buffer directly, introduced a staging buffer
(`imu_sensor_state_staging_`, guarded by `imu_sensor_state_mtx_`) that
the SDK thread (`spatialDataCallback`'s `updateAllStatesValues()` /
`updateAccelerationAndGyrationStateValues()`, and
`spatialDetachCallback`'s `setStateValuesToNans()`) writes to under a
blocking lock. A new `updateExportedStateValues()`, called every
`read()` cycle, non-blockingly (`try_lock`, same pattern as item #4 and
`ContactCoilHandler`) copies the staging buffer into `imu_sensor_state_`
— the actually-exported memory. Because `imu_sensor_state_` is now
*only* ever written by our own RT-thread `read()`, and the framework's
`get_value()` reads happen on that same RT thread afterward (single-
threaded controller_manager execution model: `read()` → controllers
update → `write()`), there is no remaining concurrent access to the
exported memory — the race is fully closed, not just narrowed.
`updateAllStatesValues()` also now holds the staging lock for the whole
sample (orientation + angular velocity + linear acceleration together),
so a reader never sees a torn cross-field sample either.

Incidental fix while touching `updateAccelerationAndGyrationStateValues()`:
the gravity-removal branch had a real bug (see nice-to-have #1 below) —
both the y and z entries were being written to the `linear_acceleration_y`
index (`... = lin_acc.y - gy;` then `... = lin_acc.y - gz;`), so
`linear_acceleration_z` was never actually updated when
`remove_gravity_vector` was enabled. Fixed as part of rewriting this
function to target the staging buffer, since carrying a known-broken
formula into new code made no sense; now writes
`linear_acceleration_z = lin_acc.z - gz`.

**6. [FIXED] Lifecycle transitions invoked directly from the vendor SDK's own callback thread**
`src/rover_sensors/phidget_imu_sensor.cpp:476-495` —
`spatialAttachCallback()`/`spatialDetachCallback()` call
`on_activate(...)`/`on_deactivate(...)` directly. This bypasses
`rclcpp_lifecycle`'s managed state machine entirely (controller_manager's
view of the component's lifecycle state is never updated to match), and
re-enters the Phidget API (`configureCompassParams`,
`spatial_->setHeatingEnabled`, etc.) from inside a callback owned by that
same Phidget device — a reentrancy hazard into `libphidget22`.

*Fix:* removed the `on_activate(...)`/`on_deactivate(...)` calls from both
SDK-thread callbacks entirely. `spatialAttachCallback()` now only sets
`imu_connected_ = true` and logs; `spatialDetachCallback()` keeps its
existing non-SDK bookkeeping (`imu_connected_ = false`,
`algorithm_initialized_ = false`, `setStateValuesToNans()`,
`restartMadgwickAlgorithm()` — none of which touch the Phidget SDK or
`spatial_`). This also incidentally closes a latent self-deadlock: the old
attach path called `on_activate()` → `calibrate()`, which blocks on
`calibration_cv_` until a *subsequent* `spatialDataCallback()` notifies
it — if Phidget serializes callback delivery on a single dispatch thread
(typical for this kind of event-driven vendor SDK), that data callback
could never run while the attach callback was still blocked waiting for
it, deadlocking the SDK's own thread on every reattach. Residual
behavior change worth knowing: on a hot reattach, the sensor no longer
re-runs `spatial_->zero()` / compass / heating configuration (that still
happens once, correctly, via the real `on_activate()` triggered by
controller_manager on first activation) — recalibrating those from a
foreign thread was the actual bug, and I didn't have a safe way to defer
that work to a proper thread without larger new infrastructure, so I
removed it rather than leave the reentrancy hazard in place. If
reconfiguration-on-reattach turns out to be operationally necessary,
that needs a deliberate design (e.g. a queued task picked up by a
dedicated worker thread), not a quick patch.

**7. Suspicious front-right/rear-right index swap (state read + command write)**
- `src/rover_system/rover_a1_system.cpp:55-68` (`updateHwStates`): index
  0(`fl`)←`FRONT_LEFT`, index 2(`rl`)←`REAR_LEFT` are consistent, but index
  **1(`fr`)←`REAR_RIGHT`** and index **3(`rr`)←`FRONT_RIGHT`** are swapped
  relative to that pattern.
- `src/rover_driver/rover_a1_driver.cpp:41-50` (`sendSpeedCmd`): same swap —
  `speeds.at(1)` (the `fr` command) is sent to the `REAR_RIGHT` physical
  driver, `speeds.at(3)` (`rr`) to `FRONT_RIGHT`.

The swap is self-consistent (each URDF joint's command/feedback pair still
agree with each other), so it may be masking a real driving-behavior bug,
but every diagnostic/state-topic label keyed by `DriverNames`
(`diagnoseErrors`/`diagnoseStatus` in `rover_a1_system.cpp:102-172`,
`driverNamesToString` used in `system_ros_interface.cpp:227`) will report
the wrong physical wheel — a technician chasing a "Front Right driver
error" will be looking at the wrong motor. Given `fl`/`rl` are mapped
straightforwardly, this looks like a copy/paste bug rather than an
intentional wiring convention; needs verification against the actual
harness, and a comment either way.

**8. Heap allocation every `write()` cycle**
`src/rover_system/rover_a1_system.cpp:174-181` — `getSpeedCmd()` returns a
freshly-constructed `std::vector<float>` on every call, invoked
unconditionally from `RoverSystem::write()` (`rover_system.cpp:243-255`)
whenever the component is ACTIVE. Classic RT allocation violation
(`ros2_control_architecture.md` §5).

**9. Exception-as-control-flow inside the RT `write()` path**
`src/rover_system/rover_system.cpp:456-478`
(`handleRoverDriverWriteOperation`) — a failed `try_lock()` on the write
mutex (an *expected*, recoverable condition under contention) throws
`std::runtime_error`, caught two frames up. Throwing/catching on every
lock-contention cycle in the hot path is expensive and explicitly
disallowed by the RT rules.

## Should fix

1. **Hardware "activation" happens in `on_configure()`, not `on_activate()`.**
   `rover_system.cpp:98-103` calls `RoverDriverInterface::activate()`
   (which sends a zero-velocity command and sleeps 1s,
   `phidget_rover_driver.cpp:71-84`) from inside `on_configure()`;
   `on_activate()`/`on_deactivate()` are empty no-ops
   (`rover_system.cpp:147-155`). This blurs the inactive/active boundary —
   a merely-*configured* hardware component has already commanded and
   armed the motors' Phidget fail-safe timers.
2. **Hardcoded Modbus endpoint.** `src/rover_controller/rover_controller.cpp:196`
   — `RoverModbus("192.168.88.11", 502)` is a literal, not read from
   `info_.hardware_parameters`/URDF `<param>`. Can't be reconfigured per-robot
   without a rebuild, unlike every other tunable in this package.
3. **Implicit/unmanaged build dependency on `Modbus_Core`.**
   `CMakeLists.txt:62-65` does
   `target_link_libraries(${PROJECT_NAME} phidgets_spatial_parameters Modbus_Core)`
   with no `find_package`/`pkg_check_modules` for `Modbus_Core`, and neither
   `CMakeLists.txt` nor `package.xml` declare a dependency on `rover_modbus`
   (which, notably, has **no `package.xml`** — it isn't a colcon package at
   all, just a plain CMake project someone appears to have built out-of-tree
   in `src/rover_modbus/build/`). This will fail to link on any fresh
   checkout/CI unless that sibling tree happens to have been built first by
   hand.
4. **Periodic RT-thread allocations, gated but not eliminated.**
   `RoverSystem::read()` (`rover_system.cpp:226-236`) →
   `ContactCoilHandler::getIoState()` (`rover_controller.cpp:129-139`)
   copies an `std::unordered_map`; `updateDriverStateMsg()` →
   `SystemROSInterface::getDriverStateByName()`
   (`system_ros_interface.cpp:223-244`) calls `driverNamesToString()`
   (allocates a `std::string`) on every invocation. Both run on the RT
   thread, throttled by `driver_states_update_frequency` (20 Hz vs. 100 Hz
   control loop per `wheel_01_controller.yaml`) but not removed.
5. **Unthrottled logging in the write() error path.** `rover_system.cpp:474`
   — `RCLCPP_WARN_STREAM` (no throttle) inside
   `handleRoverDriverWriteOperation`'s catch block; sustained lock
   contention would flood the logger every RT cycle. The read-path
   equivalents at lines 439-441/450-452 correctly use
   `RCLCPP_ERROR_STREAM_THROTTLE`.
6. **Unbounded blocking wait in `on_activate()`.**
   `src/rover_sensors/phidget_imu_sensor.cpp:276-286` (`calibrate()`) —
   `calibration_cv_.wait(lock, ...)` has no timeout; a dead/disconnected IMU
   hangs the `on_activate()` transition (and thus the calling
   service/controller_manager activation) forever with no failure path.
7. **No test coverage.** No `test/` directory anywhere in the package, and
   `CMakeLists.txt` has no `if(BUILD_TESTING)` block at all, despite
   `package.xml:41-43` declaring `ament_cmake_gtest`, `google-mock`,
   `ros_testing` as test dependencies. None of the lifecycle transitions,
   interface export, or (especially) E-stop gating logic is exercised by
   any test.
8. **`pluginlib` declared only as `build_depend`.** `package.xml:17` —
   should be a `<depend>` given `pluginlib` headers are used and the plugin
   mechanism is exercised at runtime by the loading process; current
   declaration is inconsistent with the rest of the manifest.

## Nice to have

1. **[FIXED, incidentally via item #5] Copy/paste typo in gravity removal.**
   `src/rover_sensors/phidget_imu_sensor.cpp:528` (original) —
   `imu_sensor_state_[linear_acceleration_y] = lin_acc.y - gz;` — this was
   actually worse than a typo: both branches wrote to the
   `linear_acceleration_y` index, so `linear_acceleration_z` was never
   set at all when `remove_gravity_vector` was enabled. Fixed while
   rewriting `updateAccelerationAndGyrationStateValues()` for the item #5
   staging-buffer race fix; now correctly writes
   `linear_acceleration_z = lin_acc.z - gz`.
2. **Broken include guard.**
   `include/rover_hardware_interface/rover_system/rover_system.hpp:15-16` —
   `#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_` /
   `#define ROVER_HARDWARE_INTERFACE_ROBOT_SYSTEM_ROVER_SYSTEM_HPP_` (ROVER
   vs. ROBOT) — the guard doesn't actually guard; harmless today only
   because the header isn't currently double-included. `#endif` comment at
   line 146 has the same typo.
3. **Fragile substring joint-name matching.**
   `sortAndCheckJointNames()`/`checkIfJointNameContainValidSequence`
   (`rover_system.cpp:266-284`) matches joints by substring containment of
   `"fl"/"fr"/"rl"/"rr"` — would silently misassign if a namespaced joint
   name ever contained more than one token.

---

## Priority note

The two most concerning items are **#2/#3** (e-stop not actually gating
motion in software) and **#7** (possible wheel-label swap) — worth
confirming against the physical harness and safety requirements before
anything else here.

## Relevant files

- `src/rover_system/rover_system.cpp`
- `src/rover_system/rover_a1_system.cpp`
- `src/rover_driver/rover_a1_driver.cpp`
- `src/rover_driver/phidget_driver/phidget_motor_driver.cpp`
- `src/rover_sensors/phidget_imu_sensor.cpp`
- `src/system_e_stop/system_e_stop.cpp`
- `src/rover_controller/rover_controller.cpp`
- `CMakeLists.txt`
- `package.xml`
