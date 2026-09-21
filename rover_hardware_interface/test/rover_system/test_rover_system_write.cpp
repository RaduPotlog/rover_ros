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
//
// Tests RoverSystem::write()'s wiring around RoverControlLoopUseCase::decideWriteCommand() -
// which command actually reaches the driver, and what happens to the controller's pending command
// - for every lifecycle / E-Stop / failsafe-latch combination that matters. The pure decision
// itself is covered by test/application/test_rover_control_loop_use_case.cpp.
//
// Unlike test_rover_a1_system_on_init.cpp (which uses the real RoverA1System and so can't go past
// on_init() without Modbus/Phidget hardware), this drives a minimal RoverSystem subclass whose
// defineRoverDriver()/defineRoverController() extension points return in-memory fakes, so
// on_configure() and write() run for real with no hardware. on_configure() does create a real
// SystemROSInterface node, hence the rclcpp::init()/shutdown() suite fixture.

#include <gtest/gtest.h>

#include <algorithm>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_component_interface_params.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "rover_hardware_interface/rover_system/rover_system.hpp"
#include "fakes/fake_rover_driver.hpp"

namespace rover_hardware_interface
{
namespace
{

class FakeRoverGpioPort : public RoverGpioPort
{

public:

    void start() override {}
    void eStopUserBtnTrigger(const bool /* state */) override {}
    void eStopMotorDriverFaultTrigger(const bool /* state */) override {}

    const std::unordered_map<RoverControllerGpio, bool> & queryControlInterfaceIOStates() override
    {
        return io_states_;
    }

    SafetyLinkHealth linkHealth() const override { return health; }

    // Public so a test can present an unhealthy link without another accessor.
    SafetyLinkHealth health;

private:

    std::unordered_map<RoverControllerGpio, bool> io_states_;
};

// Fills RoverSystem's extension points with fakes and exposes just enough protected state to put
// write() into each case under test. Adds no behaviour of its own on the write() path.
class TestableRoverSystem : public RoverSystem
{

public:

    TestableRoverSystem()
    : RoverSystem({"fl", "fr", "rl", "rr"})
    {
    }

    std::shared_ptr<FakeRoverDriver> fakeDriver() const { return fake_driver_; }

    void setLifecycle(const std::uint8_t id, const std::string & label)
    {
        set_lifecycle_state(rclcpp_lifecycle::State(id, label));
    }

    void setEStopActive(const bool active) { e_stop_active_ = active; }

    // What the controller would write into the exported velocity command interfaces.
    void setControllerCommand(const double velocity)
    {
        std::fill(hw_commands_velocities_.begin(), hw_commands_velocities_.end(), velocity);
    }

    // What the encoders would report back as the measured wheel velocity.
    void setMeasuredVelocity(const double velocity)
    {
        std::fill(hw_states_velocities_.begin(), hw_states_velocities_.end(), velocity);
    }

    // The two flags the E-Stop reset service reads (see areVelocityCommandsNearZero()).
    bool commandsAreZeroFlag() const { return commands_are_zero_.load(); }
    bool statesAreZeroFlag() const { return states_are_zero_.load(); }

    bool eStopResetWouldBeAllowed() { return areVelocityCommandsNearZero(); }

    void refreshZeroFlags() { refreshVelocityCommandsZeroFlag(); }

    void latchMotorFailsafe()
    {
        fake_driver_->failsafe_tripped = true;
        control_loop_use_case_->updateMotorFailsafeTrippedStatus();
    }

protected:

    void defineRoverDriver() override { rover_driver_ = fake_driver_; }
    void readRoverControllerSettings() override {}

    void defineRoverController() override
    {
        rover_controller_ = std::make_shared<FakeRoverGpioPort>();
        e_stop_ = std::make_shared<FakeEmergencyStop>();
    }

    void updateHwStates(const rclcpp::Time & /* time */) override {}
    void updateDriverStateMsg() override {}

    void getSpeedCmd(std::vector<float> & speed_cmd) const override
    {
        // Same mapping as RoverA1System::getSpeedCmd().
        for (std::size_t i = 0; i < speed_cmd.size(); ++i) {
            speed_cmd[i] = static_cast<float>(hw_commands_velocities_[i]);
        }
    }

    void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & /* status */) override {}
    void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & /* status */) override {}

private:

    std::shared_ptr<FakeRoverDriver> fake_driver_ = std::make_shared<FakeRoverDriver>();
};

hardware_interface::ComponentInfo makeWheelJoint(const std::string & name)
{
    hardware_interface::ComponentInfo joint;
    joint.name = name;
    joint.type = "joint";

    hardware_interface::InterfaceInfo command_velocity;
    command_velocity.name = hardware_interface::HW_IF_VELOCITY;
    joint.command_interfaces.push_back(command_velocity);

    for (const auto & name_ : {hardware_interface::HW_IF_POSITION,
            hardware_interface::HW_IF_VELOCITY, hardware_interface::HW_IF_EFFORT}) {
        hardware_interface::InterfaceInfo state;
        state.name = name_;
        joint.state_interfaces.push_back(state);
    }

    return joint;
}

// Mirrors test_rover_a1_system_on_init.cpp's buildValidHardwareInfo(), minus the Modbus
// parameters (TestableRoverSystem::readRoverControllerSettings() reads none).
hardware_interface::HardwareComponentInterfaceParams makeParams()
{
    hardware_interface::HardwareInfo info;
    info.name = "TestableRoverSystem";
    info.type = "system";

    info.joints.push_back(makeWheelJoint("fl_wheel_base_to_fl_wheel_joint"));
    info.joints.push_back(makeWheelJoint("fr_wheel_base_to_fr_wheel_joint"));
    info.joints.push_back(makeWheelJoint("rl_wheel_base_to_rl_wheel_joint"));
    info.joints.push_back(makeWheelJoint("rr_wheel_base_to_rr_wheel_joint"));

    info.hardware_parameters = {
        {"driver_states_update_frequency", "20"},
        {"max_rover_driver_initialization_attempts", "1"},
        {"max_rover_driver_activation_attempts", "1"},
        {"max_write_cmds_errors_count", "2"},
        {"max_read_motor_states_errors_count", "2"},
        {"max_read_driver_state_errors_count", "2"},
        {"driver_comm_timeout_ms", "300"},
        {"motor_torque_constant", "0.11"},
        {"max_rpm_motor_speed", "2800"},
        {"gear_ratio", "23.3"},
        {"gearbox_efficiency", "0.70"},
        {"raw_current_to_amps_scale", "0.1"},
        {"encoder_resolution", "1024"},
    };

    hardware_interface::HardwareComponentInterfaceParams params;
    params.hardware_info = std::move(info);
    return params;
}

const std::vector<float> kZeros(4, 0.0f);

}  // namespace

class RoverSystemWriteTest : public ::testing::Test
{
protected:

    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        ASSERT_EQ(system_.on_init(makeParams()), CallbackReturn::SUCCESS);
        ASSERT_EQ(system_.on_configure(rclcpp_lifecycle::State()), CallbackReturn::SUCCESS);
        driver_ = system_.fakeDriver();
        driver_->sent_speed_cmds.clear();
    }

    void TearDown() override { system_.on_cleanup(rclcpp_lifecycle::State()); }

    void setActive() { system_.setLifecycle(State::PRIMARY_STATE_ACTIVE, "active"); }
    void setInactive() { system_.setLifecycle(State::PRIMARY_STATE_INACTIVE, "inactive"); }

    void write() { system_.write(rclcpp::Time(0, 0, RCL_ROS_TIME), rclcpp::Duration(0, 0)); }

    using State = lifecycle_msgs::msg::State;

    TestableRoverSystem system_;
    std::shared_ptr<FakeRoverDriver> driver_;
};

TEST_F(RoverSystemWriteTest, ActiveWithoutEStopForwardsControllerCommand)
{
    setActive();
    system_.setEStopActive(false);
    system_.setControllerCommand(1.5);

    write();

    ASSERT_EQ(driver_->sent_speed_cmds.size(), 1u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), std::vector<float>(4, 1.5f));
}

TEST_F(RoverSystemWriteTest, ActiveWithEStopSendsZerosAndDropsPendingCommand)
{
    setActive();
    system_.setEStopActive(true);
    system_.setControllerCommand(1.5);

    write();

    // Zeros still reach the drivers (keeps the motor watchdog fed), not the controller's command.
    ASSERT_EQ(driver_->sent_speed_cmds.size(), 1u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), kZeros);
    // The reset-invariant flag is computed BEFORE the command is dropped, so a non-zero command
    // during the E-Stop still refuses the reset.
    EXPECT_FALSE(system_.commandsAreZeroFlag());

    // E-Stop clears, controller writes nothing new: the stale 1.5 must not resurface.
    system_.setEStopActive(false);
    write();

    ASSERT_EQ(driver_->sent_speed_cmds.size(), 2u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), kZeros);
    EXPECT_TRUE(system_.commandsAreZeroFlag());
}

TEST_F(RoverSystemWriteTest, ActiveWithLatchedMotorFailsafeSendsZeros)
{
    setActive();
    system_.setEStopActive(false);
    system_.latchMotorFailsafe();
    system_.setControllerCommand(1.5);

    write();

    ASSERT_EQ(driver_->sent_speed_cmds.size(), 1u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), kZeros);
}

TEST_F(RoverSystemWriteTest, InactiveSendsZeros)
{
    setInactive();
    system_.setEStopActive(false);
    system_.setControllerCommand(1.5);

    write();

    ASSERT_EQ(driver_->sent_speed_cmds.size(), 1u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), kZeros);
}

TEST_F(RoverSystemWriteTest, UnconfiguredSendsNothingButStillDropsPendingCommand)
{
    system_.setLifecycle(State::PRIMARY_STATE_UNCONFIGURED, "unconfigured");
    system_.setEStopActive(false);
    system_.setControllerCommand(1.5);

    write();

    EXPECT_TRUE(driver_->sent_speed_cmds.empty());

    // The dropped command must not be forwarded once motion is allowed again.
    setActive();
    write();

    ASSERT_EQ(driver_->sent_speed_cmds.size(), 1u);
    EXPECT_EQ(driver_->sent_speed_cmds.back(), kZeros);
}


// --- E-Stop reset invariant: measured velocity ----------------------------------------------
//
// The command-side check alone is a weak guarantee: the wheel PIDs park their command at a frozen
// I-term (up to i_clamp_max, 0.33 rad/s) whenever motion is inhibited, which is why the URDF
// tolerance had drifted up to 1.2 rad/s (~0.2 m/s) - fast enough to walk beside. The measured
// velocity has no such artifact, so it is what actually prevents clearing the E-Stop mid-roll.

TEST_F(RoverSystemWriteTest, RefusesEStopResetWhileTheWheelsAreStillTurning)
{
    // Commands have settled to a plausible frozen I-term, well inside the command tolerance, but
    // the rover is still rolling.
    system_.setControllerCommand(0.005);
    system_.setMeasuredVelocity(3.0);
    system_.refreshZeroFlags();

    EXPECT_TRUE(system_.commandsAreZeroFlag());
    EXPECT_FALSE(system_.statesAreZeroFlag());
    EXPECT_FALSE(system_.eStopResetWouldBeAllowed());
}

TEST_F(RoverSystemWriteTest, RefusesEStopResetWhileCommandsAreNonZeroEvenIfStopped)
{
    system_.setControllerCommand(5.0);
    system_.setMeasuredVelocity(0.0);
    system_.refreshZeroFlags();

    EXPECT_FALSE(system_.commandsAreZeroFlag());
    EXPECT_TRUE(system_.statesAreZeroFlag());
    EXPECT_FALSE(system_.eStopResetWouldBeAllowed());
}

TEST_F(RoverSystemWriteTest, AllowsEStopResetOnlyWhenBothCommandsAndWheelsAreAtRest)
{
    system_.setControllerCommand(0.0);
    system_.setMeasuredVelocity(0.0);
    system_.refreshZeroFlags();

    EXPECT_TRUE(system_.eStopResetWouldBeAllowed());
}

TEST_F(RoverSystemWriteTest, NonFiniteMeasuredVelocityFailsSafe)
{
    system_.setControllerCommand(0.0);
    system_.setMeasuredVelocity(std::numeric_limits<double>::quiet_NaN());
    system_.refreshZeroFlags();

    EXPECT_FALSE(system_.statesAreZeroFlag());
    EXPECT_FALSE(system_.eStopResetWouldBeAllowed());
}

}  // namespace rover_hardware_interface
