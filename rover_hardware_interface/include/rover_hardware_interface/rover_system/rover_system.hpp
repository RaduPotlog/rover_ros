// Copyright 2025 Mechatronics Academy
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_

#include <atomic>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <diagnostic_updater/diagnostic_status_wrapper.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include "hardware_interface/hardware_component_interface.hpp"

#include "rover_hardware_interface/domain/rover_driver.hpp"
#include "rover_hardware_interface/system_ros_interface/system_ros_interface.hpp"

#include "rover_hardware_interface/application/rover_control_loop_use_case.hpp"
#include "rover_hardware_interface/domain/emergency_stop.hpp"
#include "rover_hardware_interface/domain/rover_error_filter.hpp"
#include "rover_hardware_interface/domain/rover_gpio_port.hpp"

namespace rover_hardware_interface
{

using return_type = hardware_interface::return_type;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;
using StateInterface = hardware_interface::StateInterface;
using CommandInterface = hardware_interface::CommandInterface;

// RoverSystem is the generic hardware_interface::SystemInterface adapter: lifecycle, joint
// bookkeeping, the RT read()/write() cycle, and the domain-port-only view of the driver/safety
// controller (RoverDriverInterface, RoverGpioPort, EmergencyStopInterface). It has no knowledge of
// any concrete rover variant's backend. A concrete rover (e.g. RoverA1System in rover_a1_system.hpp,
// the Phidget+Modbus variant) derives from this class and implements only the pure-virtual
// extension points below (defineRoverDriver(), defineRoverController(), updateHwStates(),
// getSpeedCmd(), ...). If you're adding a new use case or domain rule, it belongs here or in
// domain/application/ - not in a per-variant subclass; if you're adding rover-specific wiring
// (a new backend, new URDF params), it belongs in the subclass instead.
//
// Real-time contract for read()/write() (see .claude/rules/ros2_control_architecture.md §5):
//   - Driver feedback is a cached snapshot refreshed opportunistically under an uncontended lock
//     (RoverDriverInterface::getData(), backed by PhidgetRoverDriver) - never blocks on the
//     diagnostics thread.
//   - GPIO/E-Stop IO reads (RoverGpioPort::queryControlInterfaceIOStates(),
//     EmergencyStopInterface::readEStopState()/readEStopLatchState()) use try_lock and return the
//     last-known-good value on contention rather than blocking, since the actual Modbus polling
//     happens on a dedicated background thread (see ContactCoilHandler).
//   - All read()-path publishing goes through realtime_tools::RealtimePublisher (see
//     SystemROSInterface), never a plain rclcpp::Publisher::publish() call.
//   - Motor commands in write() use the Phidget SDK's async call
//     (PhidgetDCMotor_setTargetVelocity_async()) and drop the command rather than block if a
//     prior async call hasn't completed yet.
//   - handleRoverDriverWriteOperation() delegates the actual write attempt to
//     RoverControlLoopUseCase::performWriteOperation(), which also uses try_lock and skips the
//     write cycle on contention instead of blocking (see application/rover_control_loop_use_case.hpp).
//   - Accepted exception: RCLCPP_*_STREAM_THROTTLE calls on error/contention branches build a
//     std::stringstream (allocates) but are throttled to at most once per 5s and only fire off
//     the happy path - a deliberate tradeoff, not an oversight.
class RoverSystem : public hardware_interface::SystemInterface
{

public:

    explicit RoverSystem(const std::vector<std::string> & joint_order);

    virtual ~RoverSystem() = default;

    CallbackReturn on_init(const hardware_interface::HardwareComponentInterfaceParams & params) override;
    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;
    CallbackReturn on_error(const rclcpp_lifecycle::State & previous_state) override;

    std::vector<StateInterface> export_state_interfaces() override;
    std::vector<CommandInterface> export_command_interfaces() override;

    return_type read(const rclcpp::Time & time, const rclcpp::Duration & /* period */) override;
    return_type write(const rclcpp::Time & /* time */, const rclcpp::Duration & /* period */) override;

protected:

    void checkJointSize() const;
    void sortAndCheckJointNames();
    void setInitialValues();
    void checkInterfaces() const;

    void readDrivetrainSettings();
    void readDriverStatesUpdateFrequency();
    void readDriverInitAndActivationAttempts();
    void readErrorFilterMaxErrorsCounts();

    void configureRoverController();
    void configureRoverDriver();

    // Shared by on_cleanup()/on_shutdown(): both tear down the same set of owned components, and
    // on_shutdown() in particular must tolerate being called from Unconfigured (rover_driver_ /
    // rover_controller_ / e_stop_ / system_ros_interface_ still null - on_configure() was never
    // reached), which the null check on rover_driver_ guards against.
    void teardownRoverComponents();

    virtual void defineRoverDriver() = 0;

    // Reads whatever URDF hardware_parameters this rover variant's safety-controller backend
    // needs (e.g. Modbus host/port for RoverA1System) into a member owned by the subclass. This
    // is a pure virtual extension point rather than a concrete RoverSystem method (unlike
    // readDrivetrainSettings() etc. above) because, unlike the drivetrain parameters, the
    // safety-controller transport is backend-specific: RoverSystem itself has no business
    // requiring a "modbus_host" parameter to exist. Called from on_init() inside the same
    // try/catch as the other read*Settings() calls, so any std::exception it throws (e.g.
    // std::invalid_argument for a missing/empty parameter) is handled uniformly. See
    // defineRoverController() for the matching construction-time extension point.
    virtual void readRoverControllerSettings() = 0;

    // Pure virtual extension point (mirrors defineRoverDriver()): constructs and stores
    // rover_controller_ (RoverGpioPort) and e_stop_ (EmergencyStopInterface) using whatever
    // safety-IO backend this rover variant uses. RoverSystem only depends on these two domain
    // ports here - never on a concrete backend type - so a rover variant using a different
    // safety-IO transport (e.g. CAN or direct GPIO instead of Modbus) only has to provide a new
    // implementation of this method, not modify RoverSystem. configureRoverController() (which
    // calls this) then performs the generic start()/arm sequence common to every backend, purely
    // through the RoverGpioPort interface. See RoverA1System::defineRoverController() for the
    // reference (Modbus-backed) implementation.
    virtual void defineRoverController() = 0;

    void resetEStop();
    void resetEStopLatch();

    void updateMotorsState(const rclcpp::Time & time);
    void updateDriverState();
    void updateCommunicationStatus();
    void updateMotorsStateDataTimedOut();
    void updateDriverStateDataTimedOut();
    void updateFlagErrors();
    void updateEStopState();
    virtual void updateHwStates(const rclcpp::Time & time) = 0;

    virtual void updateDriverStateMsg() = 0;

    void handleRoverDriverWriteOperation(std::function<void()> write_operation);

    // Logs the outcome of a RoverControlLoopUseCase::performWriteOperation() call at the same
    // throttled WARN level and with the same messages this package used before the write-path
    // decision logic moved into RoverControlLoopUseCase. A no-op on kSucceeded.
    void logWriteOperationResult(const WriteOperationResult & result);

    bool areVelocityCommandsNearZero();

    // Fills the (already correctly-sized) `speed_cmd` buffer in place — no allocation, so it's
    // safe to call from write() every RT cycle.
    virtual void getSpeedCmd(std::vector<float> & speed_cmd) const = 0;

    virtual void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;
    virtual void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) = 0;

    const size_t joint_size_;

    const std::vector<std::string> joint_order_;
    std::vector<std::string> joints_names_sorted_;

    std::vector<double> hw_commands_velocities_;
    std::vector<double> hw_states_positions_;
    std::vector<double> hw_states_velocities_;
    std::vector<double> hw_states_efforts_;

    // Reused across write() cycles by getSpeedCmd() to avoid allocating on the RT thread.
    // Sized once in setInitialValues().
    std::vector<float> speed_cmd_buffer_;

    // Rover driver interface
    std::shared_ptr<RoverDriverInterface> rover_driver_;
    // Rover safety controller GPIO port. read()-path GPIO polling and initial arming go through
    // this domain port, never a concrete backend type - defineRoverController() (implemented per
    // rover variant, e.g. RoverA1System's Modbus-backed one) constructs the concrete adapter as a
    // scoped local and never stores it as a member, so nothing on the RT read()/write() path can
    // reach it directly.
    std::shared_ptr<RoverGpioPort> rover_controller_;
    // Rover emergency stop interface
    std::shared_ptr<EmergencyStopInterface> e_stop_;
    // Debounced/latched per-category error state (write cmds, read-motor-states,
    // read-driver-state, fault-flag). Diagnostics-only: never gates read()/write() return codes
    // or triggers a lifecycle transition. Built once in on_init() (readErrorFilterMaxErrorsCounts())
    // from URDF thresholds; unlike rover_driver_/rover_controller_/e_stop_ it owns no hardware
    // handles, so it is NOT reset in on_cleanup()/on_shutdown()/on_error().
    std::unique_ptr<RoverErrorFilter> rover_error_filter_;
    // Owns the RT-loop decision logic (E-Stop state, fault-flag reset, write-command gating and
    // serialization) that previously lived directly in RoverSystem's own methods - see
    // application/rover_control_loop_use_case.hpp. Constructed once rover_driver_/e_stop_ are
    // both ready (end of on_configure()) and reset alongside them in teardownRoverComponents(),
    // so it always references the same live rover_driver_/e_stop_/rover_error_filter_ this
    // RoverSystem instance owns.
    std::unique_ptr<RoverControlLoopUseCase> control_loop_use_case_;
    // Cached result of the last updateEStopState() poll; gates write() while true.
    // Defaults to true (fail-safe: no motion commands until confirmed clear).
    std::atomic_bool e_stop_active_{true};

    // Drive train system settings
    DrivetrainSettings drivetrain_settings_;

    // ROS hardware interface
    std::unique_ptr<SystemROSInterface> system_ros_interface_;

    // Imported from URDF
    unsigned max_rover_driver_initialization_attempts_;
    unsigned max_rover_driver_activation_attempts_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
    rclcpp::Clock steady_clock_{RCL_STEADY_TIME};

    // Lazily seeded from the first `time` argument read() receives (see
    // RoverSystem::read()) rather than default-constructed with a fixed clock type: the
    // controller_manager passes `time` using its own node clock (RCL_ROS_TIME by default),
    // and rclcpp::Time comparisons throw std::runtime_error when the two operands use
    // different clock types. Seeding from the actual `time` guarantees the clock types
    // always match, regardless of what clock the caller happens to use.
    bool driver_state_update_time_initialized_{false};
    rclcpp::Time next_driver_state_update_time_{0, 0, RCL_ROS_TIME};
    rclcpp::Duration driver_states_update_period_{0, 0};
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_SYSTEM_HPP_
