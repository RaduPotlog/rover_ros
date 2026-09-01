# Audit: `rover_hardware_interface` (branch `jazzy`)

Full-package audit (no pending diff existed at review time). Performed by the
`ros2-controllers-reviewer` agent on 2026-08-25. Package reviewed in full
(no `test/` directory exists). Cross-checked against
`rover_a1_macro.urdf.xacro`, `imu.urdf.xacro`, and
`wheel_01_controller.yaml` in sibling packages.

## Regression found and fixed after the audit (2026-08-26)

**E-Stop polarity inversion made `write()` permanently refuse to command motion.**
`src/system_e_stop/system_e_stop.cpp` — after wiring the (previously dead)
`EmergencyStop::readEStopState()`/`readEStopLatchState()` into
`RoverSystem::updateEStopState()` for item #2 above, the user reported
that `RoverSystem::write()` never reached
`handleRoverDriverWriteOperation(...)` at all, even under healthy
conditions. Investigation found `readEStopState()`/`readEStopLatchState()`
computed `!rover_controller_->isPinActive(pin)` — inverted relative to
the write-side convention established in the same file by `setEStop()`
(writes coil `true` = triggered) / `resetEStop()` (writes coil `false`
= cleared). Under healthy conditions both relevant coils correctly read
back `false` ("clear"), so the stray `!` made both functions report
`true` ("triggered") permanently, holding `e_stop_active_` stuck `true`
forever. This bug was **pre-existing in the original dead code**, not
introduced by the item #2 fix — activating that dead code just exposed
it for the first time.

*Fix:* removed the `!` in both `readEStopState()` and
`readEStopLatchState()`, matching the write-side polarity. Also split
the single shared `std::atomic_bool e_stop_triggered_` member (used by
both functions) into two independent members
(`user_e_stop_triggered_`, `latch_triggered_` in
`system_e_stop.hpp`) — a related latent bug found during the same
investigation: under lock contention, each function could return the
*other* function's last-known value instead of its own.

**Not verified against real hardware.** This is safety-critical E-Stop
logic; no Modbus rig or colcon build is available in this environment.
Before trusting this on the physical robot: build the package, then run
a supervised E-Stop press/release test and confirm (a) motion is
refused while the E-Stop is held and (b) motion resumes after a clean
release + reset, in that order — don't just trust the code-review trace
above.

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

**7. [FIXED] Suspicious front-right/rear-right index swap (state read + command write)**
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

*Fix:* user confirmed (2026-08-25) this was a genuine copy/paste bug, not
intentional wiring compensation — URDF joint `fr` (`hw_states_*[1]` /
`hw_commands_velocities_[1]`) should map to `DriverNames::FRONT_RIGHT`,
and `rr` (index 3) to `DriverNames::REAR_RIGHT`. Swapped both
`RoverA1System::updateHwStates()` (state read) and
`RoverA1Driver::sendSpeedCmd()` (command write) to match, and added a
one-line comment at each site pointing at `joint_order_` in
`rover_a1_system.hpp` (`{"fl", "fr", "rl", "rr"}`) as the source of
truth for the index ↔ joint mapping, so this doesn't drift again
silently. **This changes which physical motor receives which command —
verify on the real robot (e.g. a slow, supervised `cmd_vel` test with
each wheel hand-checked) before trusting it in the field.**

**8. [FIXED] Heap allocation every `write()` cycle**
`src/rover_system/rover_a1_system.cpp:174-181` — `getSpeedCmd()` returns a
freshly-constructed `std::vector<float>` on every call, invoked
unconditionally from `RoverSystem::write()` (`rover_system.cpp:243-255`)
whenever the component is ACTIVE. Classic RT allocation violation
(`ros2_control_architecture.md` §5).

*Fix:* changed the (pure virtual) `getSpeedCmd()` contract from
"return a freshly-built `std::vector<float>`" to "fill the caller-owned
`speed_cmd` buffer in place" (`getSpeedCmd(std::vector<float> &)`,
`rover_system.hpp`/`rover_a1_system.hpp`). Added a persistent
`speed_cmd_buffer_` member, sized once in `setInitialValues()`
(`on_init()`, non-RT). `write()` now calls
`getSpeedCmd(speed_cmd_buffer_)` and passes that same buffer to
`sendSpeedCmd()` — no allocation on the RT path after startup.

**9. [FIXED] Exception-as-control-flow inside the RT `write()` path**
`src/rover_system/rover_system.cpp:456-478`
(`handleRoverDriverWriteOperation`) — a failed `try_lock()` on the write
mutex (an *expected*, recoverable condition under contention) throws
`std::runtime_error`, caught two frames up. Throwing/catching on every
lock-contention cycle in the hot path is expensive and explicitly
disallowed by the RT rules.

*Fix:* restructured `handleRoverDriverWriteOperation()` so a failed
`try_lock()` returns early with a throttled warning
(`RCLCPP_WARN_STREAM_THROTTLE`, 5s) instead of throwing — lock
contention is an expected, recoverable condition (e.g. a concurrent
E-Stop reset service call), not exceptional control flow. The
`try`/`catch` around `write_operation()` itself is kept, since a
`sendSpeedCmd()` failure from the driver *is* a genuine, rare error
condition — that remains a legitimate use of exceptions, just no longer
on the routine contended-lock path.

## Should fix

1. **[FIXED] Hardware "activation" happens in `on_configure()`, not `on_activate()`.**
   `rover_system.cpp:98-103` calls `RoverDriverInterface::activate()`
   (which sends a zero-velocity command and sleeps 1s,
   `phidget_rover_driver.cpp:71-84`) from inside `on_configure()`;
   `on_activate()`/`on_deactivate()` are empty no-ops
   (`rover_system.cpp:147-155`). This blurs the inactive/active boundary —
   a merely-*configured* hardware component has already commanded and
   armed the motors' Phidget fail-safe timers.

   *Fix:* moved the `operationWithAttempts(..., RoverDriverInterface::activate, ...)`
   call (and its error handling) out of `on_configure()` and into
   `on_activate()`. `on_configure()` now only prepares resources —
   creating/initializing the driver and controller objects, the E-Stop
   object, the ROS services/diagnostics, and zeroing the command/state
   buffers — none of which command motion. `on_deactivate()` is left as
   a no-op, matching pre-existing behavior: `RoverDriverInterface` has no
   `deactivate()` counterpart, and `write()` already stops being called
   once the component leaves `ACTIVE`, so no further commands are sent.
   Note this is a **pre-existing, unchanged residual gap**, not something
   introduced by this fix: once a component goes `INACTIVE`, the motors
   simply keep running at their last commanded velocity until Phidget's
   own failsafe timer (5s with no `sendCmdVel`, enabled during
   `initialize()`) eventually stops them — there's no explicit "send zero
   velocity on deactivate" safety command. Adding one is a reasonable
   follow-up but changes real motor-stop behavior on the physical robot,
   so I left it out of this fix rather than bundle it in unasked.
2. **[FIXED] Hardcoded Modbus endpoint.** `src/rover_controller/rover_controller.cpp:196`
   — `RoverModbus("192.168.88.11", 502)` is a literal, not read from
   `info_.hardware_parameters`/URDF `<param>`. Can't be reconfigured per-robot
   without a rebuild, unlike every other tunable in this package.

   *Fix:* added a `ModbusSettings {host, port}` struct (`utils.hpp`,
   alongside `DrivetrainSettings`) and `RoverSystem::readModbusSettings()`
   (`rover_system.cpp`, called from `on_init()` alongside the other
   parameter readers — throws `std::invalid_argument` on a missing/empty
   `modbus_host` or unparseable `modbus_port`, same convention as the
   existing readers). `RoverController`'s constructor now takes
   `(modbus_host, modbus_port)` instead of hardcoding them, and
   `RoverSystem::configureRoverController()` passes `modbus_settings_`
   through. Added `<param name="modbus_host">192.168.88.11</param>` /
   `<param name="modbus_port">502</param>` to
   `rover_description/urdf/rover_a1/rover_a1_macro.urdf.xacro`
   (defaulting to the previous hardcoded values, so behavior is unchanged
   unless a deployment overrides them) — reconfiguring the endpoint is
   now a URDF edit, not a rebuild, matching every other tunable in this
   package.
3. **[FIXED] Implicit/unmanaged build dependency on `Modbus_Core`.**
   `CMakeLists.txt:62-65` does
   `target_link_libraries(${PROJECT_NAME} phidgets_spatial_parameters Modbus_Core)`
   with no `find_package`/`pkg_check_modules` for `Modbus_Core`, and neither
   `CMakeLists.txt` nor `package.xml` declare a dependency on `rover_modbus`
   (which, notably, has **no `package.xml`** — it isn't a colcon package at
   all, just a plain CMake project someone appears to have built out-of-tree
   in `src/rover_modbus/build/`). This will fail to link on any fresh
   checkout/CI unless that sibling tree happens to have been built first by
   hand.

   *Investigation confirmed this was live, not hypothetical:* on this
   machine `Modbus_Core` only linked because someone had manually copied
   `libModbus_Core.so` into `/usr/local/lib` and its headers flat into
   `/usr/local/include` — an undocumented, unreproducible step.

   *Fix (user chose "CMake `find_package` export"):*
   - `src/rover_modbus/src/CMakeLists.txt` — uncompleted/uncommented the
     `install(EXPORT Modbus_CoreTargets ...)` block and added a generated
     `Modbus_CoreConfig.cmake` (new template:
     `src/rover_modbus/cmake/Modbus_CoreConfig.cmake.in`) via
     `configure_package_config_file`, so `find_package(Modbus_Core)` now
     works against any prefix it's installed to.
   - Fixed two latent packaging bugs this surfaced (both previously
     dormant because the export was never active):
     1. `target_sources(... INTERFACE ${CORE_HEADER_FILES})` embedded
        bare source-tree paths, which CMake refuses to export — dropped
        the `INTERFACE` header listing (cosmetic/IDE-only, not needed
        for consumers, who already get headers via `install(FILES...)`
        + the include dirs below).
     2. `target_include_directories` pointed the `PUBLIC` interface at
        the **build-tree source path**, which would silently break for
        any consumer using the installed package from a different
        machine/prefix. Now uses
        `$<BUILD_INTERFACE:.../include/MB>` / `$<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>`
        generator expressions so the exported target is relocatable —
        matching the flat header layout `install(FILES...)` actually
        produces (`<prefix>/include/*.hpp`, no `MB/` subdir), which is
        what `rover_hardware_interface`'s existing unqualified
        `#include <modbusRequest.hpp>` already expects. Also had to add
        `include/` back as a **private-only** dir for `Modbus_Core`'s
        own compilation, since `crc.cpp` (unlike its siblings) does
        `#include "MB/crc.hpp"` — a pre-existing inconsistency in this
        vendored library, not something to "fix" beyond keeping it
        compiling.
   - `rover_hardware_interface/CMakeLists.txt` — added
     `find_package(Modbus_Core REQUIRED)` (kept separate from the
     ament `PACKAGE_DEPENDENCIES` loop, since `Modbus_Core` isn't an
     ament package) and changed `target_link_libraries` to the proper
     imported target `Modbus_Core::Modbus_Core`.
   - Documented the required manual build+install step in
     `src/rover_modbus/README.md` (a new subsection — the file is
     otherwise the upstream `Mazurel/Modbus` README and didn't cover
     the `find_package` flow at all).
   - **Verified end-to-end**, outside the actual workspace: built +
     installed `Modbus_Core` to a throwaway prefix, then configured,
     built, linked, and ran a standalone consumer against it via
     `find_package(Modbus_Core REQUIRED)` — confirmed the exported
     target resolves correctly and is relocatable.
   - Rebuilt the same fixed `Modbus_Core` targeting
     `-DCMAKE_INSTALL_PREFIX=/usr/local` (matching this machine's
     existing ad-hoc install location) so this dev environment stays
     buildable, but the `sudo cmake --install` step itself is blocked by
     this session's permissions — **the user needs to run it manually**
     (command left in chat).
   - **Verified (2026-09-01):** `colcon build --packages-select
     rover_hardware_interface` succeeds end-to-end against the installed
     `Modbus_Core` (found at `/usr/local`, per the manual install step
     above having since been run) — no new warnings under this package's
     `-Wall -Wextra -Wpedantic -Wshadow -Wold-style-cast`. The "not yet
     verified" gap this bullet used to describe is closed.
   - Vendoring decision (plain-CMake package, not full ament conversion)
     re-confirmed and documented directly in
     `src/rover_modbus/README.md` (new "Why this stays a plain-CMake
     vendored package" subsection) rather than left implicit here only.

   Residual limitation, by design of the option chosen: `rover_modbus`
   is still **not** a colcon/ament package, so `colcon build` will not
   build or order it automatically — it must be built and installed
   with plain CMake to a prefix on `CMAKE_PREFIX_PATH` *before* building
   this workspace. That's the explicit tradeoff of this option vs. the
   "full ament_cmake package" alternative that was not chosen.
4. **[FIXED] Periodic RT-thread allocations, gated but not eliminated.**
   `RoverSystem::read()` (`rover_system.cpp:226-236`) →
   `ContactCoilHandler::getIoState()` (`rover_controller.cpp:129-139`)
   copies an `std::unordered_map`; `updateDriverStateMsg()` →
   `SystemROSInterface::getDriverStateByName()`
   (`system_ros_interface.cpp:223-244`) calls `driverNamesToString()`
   (allocates a `std::string`) on every invocation. Both run on the RT
   thread, throttled by `driver_states_update_frequency` (20 Hz vs. 100 Hz
   control loop per `wheel_01_controller.yaml`) but not removed.

   *Note: this got hotter than described here* — the item #2 E-Stop fix
   (`RoverSystem::updateEStopState()`) added two calls per **every**
   `read()` cycle (100 Hz, unthrottled) into
   `RoverController::isPinActive()` → the same `getIoState()`/map-copy
   path, not just the throttled 20 Hz diagnostics path this item
   originally described.

   *Fix — GPIO/IO-state path (the real allocation: `std::unordered_map`
   has no SSO):*
   - `ContactCoilHandler::getIoState()` now fills a caller-provided
     `std::unordered_map&` in place instead of returning a fresh one by
     value, and on lock contention **leaves the caller's buffer
     unchanged** (previously returned an empty map on contention — a
     latent correctness quirk, not just a perf one: via `isPinActive()`'s
     old `io_state[pin]` default-to-`false` behavior, contention could
     transiently read as "pin inactive," and since both e-stop checks use
     `!isPinActive(...)`, that briefly reads as e-stop *triggered* —
     fail-safe direction, but a spurious stop under load. Fixed for free
     by this same change).
   - `RoverController` now owns a persistent `io_state_cache_` member;
     `queryControlInterfaceIOStates()` returns `const std::unordered_map&`
     into it (refreshed in place) instead of a fresh copy each call.
   - `RoverController::isPinActive()` now uses `.find()` instead of
     `operator[]` on the (now shared, persistent) cache — `operator[]`
     would have inserted a spurious `false` entry into the shared cache
     for any pin never yet reported, silently growing/mutating it as a
     side effect of a "read" call.
   - Updated the two `rover_system.cpp` call sites
     (`on_configure()`/`read()`) from `const auto gpio_state = ...` to
     `const auto & gpio_state = ...` so the reference is used directly
     with no further copy.

   *Fix — driver-name lookup (smaller impact than described: `"rear_left"`
   etc. all fit libstdc++'s small-string-optimization buffer, so
   `driverNamesToString()`'s returned `std::string` likely never actually
   heap-allocates today — still fixed since a `std::string` construction
   happens on every call either way, and it's what the audit named):*
   `SystemROSInterface::getDriverStateByName()` now looks the name up via
   a new file-local `cachedDriverName()` helper
   (`system_ros_interface.cpp`) backed by a `static const std::array`
   computed once, instead of calling `driverNamesToString()` (which
   constructs a new `std::string`) on every call. Left `driverNamesToString()`
   itself untouched — it's also used in several non-RT exception-message
   call sites (`phidget_rover_driver.cpp`, `phidget_motor_driver.cpp`) via
   `operator+` string concatenation, which a `std::string`→`const char*`
   signature change would have broken; not worth that ripple for a
   throttled, SSO-avoided allocation.
5. **[FIXED] Unthrottled logging in the write() error path.** `rover_system.cpp:474`
   — `RCLCPP_WARN_STREAM` (no throttle) inside
   `handleRoverDriverWriteOperation`'s catch block; sustained lock
   contention would flood the logger every RT cycle. The read-path
   equivalents at lines 439-441/450-452 correctly use
   `RCLCPP_ERROR_STREAM_THROTTLE`.

   *Fix:* changed to `RCLCPP_WARN_STREAM_THROTTLE(logger_, steady_clock_, 5000, ...)`,
   matching the read-path convention exactly (same 5s window, same
   `steady_clock_` member already used elsewhere in this class). Note the
   lock-contention branch right above this one in the same function was
   already throttled as a side effect of the item #9 fix (exception-as-
   control-flow) — this closes the one remaining unthrottled spot in
   `handleRoverDriverWriteOperation()`.
6. **[FIXED] Unbounded blocking wait in `on_activate()`.**
   `src/rover_sensors/phidget_imu_sensor.cpp:276-286` (`calibrate()`) —
   `calibration_cv_.wait(lock, ...)` has no timeout; a dead/disconnected IMU
   hangs the `on_activate()` transition (and thus the calling
   service/controller_manager activation) forever with no failure path.

   *Fix:* `calibrate()` now uses `calibration_cv_.wait_for(lock,
   kCalibrationTimeout, pred)` with a fixed 5s timeout (generous vs. the
   "remains stationary for 2 seconds" the log message already asks for)
   and throws `std::runtime_error` on timeout instead of blocking
   forever. `on_activate()` gained a `catch (const std::runtime_error &)`
   alongside its existing `catch (const phidgets::Phidget22Error &)`, so
   a calibration timeout now returns `CallbackReturn::ERROR` with a
   logged reason, same as any other activation failure, instead of
   hanging the whole activation service call. Note: like the existing
   `Phidget22Error` catch branch it sits beside, this doesn't reset
   `imu_connected_`/`spatial_` on failure — matching, not fixing, that
   pre-existing gap, since that's a separate concern from the unbounded
   wait this item is about.
7. **No test coverage.** No `test/` directory anywhere in the package, and
   `CMakeLists.txt` has no `if(BUILD_TESTING)` block at all, despite
   `package.xml:41-43` declaring `ament_cmake_gtest`, `google-mock`,
   `ros_testing` as test dependencies. None of the lifecycle transitions,
   interface export, or (especially) E-stop gating logic is exercised by
   any test.
8. **[FIXED] `pluginlib` declared only as `build_depend`.** `package.xml:17` —
   should be a `<depend>` given `pluginlib` headers are used and the plugin
   mechanism is exercised at runtime by the loading process; current
   declaration is inconsistent with the rest of the manifest.

   *Fix:* moved `pluginlib` from `<build_depend>` into the main
   `<depend>` block in `package.xml` (which covers build + exec + their
   export variants), alongside the other real dependencies. Left the
   other `build_depend`s (`autoconf`, `autoconf-archive`, `libtool`,
   `m4`, `pkg-config`) as-is — those are genuinely build-only autotools
   tooling, not something this package's own code depends on at
   runtime.

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
2. **[FIXED] Broken include guard.**
   `include/rover_hardware_interface/rover_system/rover_system.hpp:15-16` —
   `#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_` /
   `#define ROVER_HARDWARE_INTERFACE_ROBOT_SYSTEM_ROVER_SYSTEM_HPP_` (ROVER
   vs. ROBOT) — the guard doesn't actually guard; harmless today only
   because the header isn't currently double-included. `#endif` comment at
   line 146 has the same typo.

   *Fix:* corrected `ROBOT_SYSTEM` → `ROVER_SYSTEM` in both the `#define`
   and the trailing `#endif` comment, so all three tokens
   (`#ifndef`/`#define`/`#endif` comment) match. Grepped the whole
   package for the same typo elsewhere — this was the only occurrence.
3. **[FIXED] Fragile substring joint-name matching.**
   `sortAndCheckJointNames()`/`checkIfJointNameContainValidSequence`
   (`rover_system.cpp:266-284`) matches joints by substring containment of
   `"fl"/"fr"/"rl"/"rr"` — would silently misassign if a namespaced joint
   name ever contained more than one token.

   *Investigation note:* re-reading the original implementation
   (`utils.cpp`), the `match_count != 1` check in `sortAndCheckJointNames()`
   already throws (rather than silently misassigning) for most ambiguous
   cases — an accidental extra match on another joint pushes the count to
   2+. So this was less "silent misassignment" than "fragile and hard to
   verify by inspection," but it is still possible to construct a real
   silent false-positive: with the old delimiter-based substring scan,
   a namespace segment that happened to equal a full delimited token
   (e.g. `"my_fr_robot/fl_wheel_base_to_fl_wheel_joint"` — "fr" appears
   `_`-delimited inside the namespace) would match sequence `"fr"` even
   though the joint is actually `"fl"`.

   *Fix:* rewrote `checkIfJointNameContainValidSequence()` (`utils.cpp`)
   to require `sequence` be the **leading token of the joint's own local
   name** — i.e. right after any namespace prefix (everything up to and
   including the last `/`), and either the whole local name or
   immediately followed by `_`. This directly reflects the actual xacro
   naming convention (`${prefix}_wheel_base_to_${prefix}_wheel_joint`,
   confirmed in `rover_description/urdf/common/wheel.urdf.xacro:46`) —
   the prefix is always the leading segment of the joint's own name — and
   it structurally can't match a namespace segment anymore, since the
   namespace is stripped before comparison rather than merely
   delimiter-checked. No behavior change for the current (unnamespaced)
   joint names; only namespaced-deployment edge cases are affected, and
   only for the better.

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
