// Copyright 2026 Mechatronics Academy
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
// Reproduces the rover's "Either the left or right wheel velocity is invalid" chain failure at
// the unit level: right after activation, before any update(), what does a PID export to the
// diff_drive_controller chained in front of it?

#include <array>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <gtest/gtest.h>
#include <controller_interface/controller_interface_params.hpp>
#include <hardware_interface/loaned_command_interface.hpp>
#include <hardware_interface/loaned_state_interface.hpp>
#include <lifecycle_msgs/msg/state.hpp>
#include <pid_controller/pid_controller.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_controller/seeded_pid_controller.hpp"

namespace
{

const std::string kJoint = "rl_wheel_base_to_rl_wheel_joint";

class SeededPidControllerTest : public ::testing::Test
{
protected:
  static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
  static void TearDownTestSuite() { rclcpp::shutdown(); }

  // Configure, loan interfaces the way controller_manager does, export, activate. Returns the
  // exported velocity state diff_drive would read on its first cycle.
  template<typename ControllerT>
  double exported_after_activation(
    double hw_velocity, std::shared_ptr<ControllerT> & controller,
    const std::vector<rclcpp::Parameter> & extra_parameters = {})
  {
    hw_state_ = hw_velocity;
    hw_command_ = 0.0;

    controller = std::make_shared<ControllerT>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "pid_controller_test";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    std::vector<rclcpp::Parameter> overrides = {
      {"dof_names", std::vector<std::string>{kJoint}},
      {"command_interface", "velocity"},
      {"reference_and_state_interfaces", std::vector<std::string>{"velocity"}},
      {"gains." + kJoint + ".p", 0.05},
      {"gains." + kJoint + ".i", 1.0},
      {"gains." + kJoint + ".feedforward_gain", 1.0},
      {"gains." + kJoint + ".antiwindup_strategy", "back_calculation"},
    };
    overrides.insert(overrides.end(), extra_parameters.begin(), extra_parameters.end());
    params.node_options = rclcpp::NodeOptions().parameter_overrides(overrides);
    EXPECT_EQ(controller->init(params), controller_interface::return_type::OK);
    EXPECT_EQ(
      controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    exported_ = controller->export_state_interfaces();
    reference_ = controller->export_reference_interfaces();

    command_itf_ = std::make_shared<hardware_interface::CommandInterface>(
      kJoint, "velocity", &hw_command_);
    state_itf_ = std::make_shared<hardware_interface::StateInterface>(
      kJoint, "velocity", &hw_state_);
    std::vector<hardware_interface::LoanedCommandInterface> commands;
    commands.emplace_back(command_itf_);
    std::vector<hardware_interface::LoanedStateInterface> states;
    states.emplace_back(state_itf_);
    controller->assign_interfaces(std::move(commands), std::move(states));

    EXPECT_EQ(
      controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);

    EXPECT_EQ(exported_.size(), 1u);
    return exported_.at(0)->get_optional().value();
  }

  // Two wheels in one PID, each with its own hardware velocity. `claimed_order` is the order the
  // state interfaces are loaned in; controller_manager uses the controller's configured order.
  std::shared_ptr<rover_controller::SeededPidController> activate_two_wheels(
    const std::vector<size_t> & claimed_order)
  {
    auto controller = std::make_shared<rover_controller::SeededPidController>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "pid_controller_test";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options = rclcpp::NodeOptions().parameter_overrides({
      {"dof_names", kTwoJoints},
      {"command_interface", "velocity"},
      {"reference_and_state_interfaces", std::vector<std::string>{"velocity"}},
      {"gains." + kTwoJoints[0] + ".p", 0.05},
      {"gains." + kTwoJoints[1] + ".p", 0.05},
    });
    EXPECT_EQ(controller->init(params), controller_interface::return_type::OK);
    EXPECT_EQ(
      controller->configure().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE);

    exported_ = controller->export_state_interfaces();
    reference_ = controller->export_reference_interfaces();

    std::vector<hardware_interface::LoanedCommandInterface> commands;
    std::vector<hardware_interface::LoanedStateInterface> states;
    for (size_t i = 0; i < kTwoJoints.size(); ++i) {
      two_command_itfs_[i] = std::make_shared<hardware_interface::CommandInterface>(
        kTwoJoints[i], "velocity", &two_hw_commands_[i]);
      commands.emplace_back(two_command_itfs_[i]);
    }
    for (const auto i : claimed_order) {
      two_state_itfs_[i] = std::make_shared<hardware_interface::StateInterface>(
        kTwoJoints[i], "velocity", &two_hw_states_[i]);
      states.emplace_back(two_state_itfs_[i]);
    }
    controller->assign_interfaces(std::move(commands), std::move(states));

    EXPECT_EQ(
      controller->get_node()->activate().id(), lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE);
    EXPECT_EQ(exported_.size(), kTwoJoints.size());
    return controller;
  }

  const std::vector<std::string> kTwoJoints = {
    "rl_wheel_base_to_rl_wheel_joint", "rr_wheel_base_to_rr_wheel_joint"};
  std::array<double, 2> two_hw_states_ = {0.37, -0.52};
  std::array<double, 2> two_hw_commands_ = {0.0, 0.0};
  std::array<hardware_interface::CommandInterface::SharedPtr, 2> two_command_itfs_;
  std::array<hardware_interface::StateInterface::SharedPtr, 2> two_state_itfs_;

  double hw_state_ = 0.0;
  double hw_command_ = 0.0;
  hardware_interface::CommandInterface::SharedPtr command_itf_;
  hardware_interface::StateInterface::SharedPtr state_itf_;
  std::vector<hardware_interface::StateInterface::ConstSharedPtr> exported_;
  std::vector<hardware_interface::CommandInterface::SharedPtr> reference_;
};

TEST_F(SeededPidControllerTest, UpstreamPidExportsNanUntilItsFirstUpdate)
{
  // Documents the upstream behaviour the rover tripped over; if this ever starts failing,
  // upstream fixed it and SeededPidController can go.
  std::shared_ptr<pid_controller::PidController> pid;
  EXPECT_TRUE(std::isnan(exported_after_activation(0.37, pid)));
}

TEST_F(SeededPidControllerTest, SeededPidExportsTheHardwareStateOnActivation)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  EXPECT_DOUBLE_EQ(exported_after_activation(0.37, pid), 0.37);
}

TEST_F(SeededPidControllerTest, SeededPidKeepsNanWhenTheHardwareHasNoValueYet)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  EXPECT_TRUE(std::isnan(
    exported_after_activation(std::numeric_limits<double>::quiet_NaN(), pid)));
}

TEST_F(SeededPidControllerTest, SeededPidStillTracksTheHardwareAfterUpdates)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  exported_after_activation(0.1, pid);
  hw_state_ = -0.25;
  ASSERT_EQ(
    pid->update(rclcpp::Time(0, 10'000'000), rclcpp::Duration::from_seconds(0.01)),
    controller_interface::return_type::OK);
  EXPECT_DOUBLE_EQ(exported_.at(0)->get_optional().value(), -0.25);
}

TEST_F(SeededPidControllerTest, SeededPidSeedsEachWheelFromItsOwnHardwareState)
{
  const auto pid = activate_two_wheels({0, 1});
  for (size_t i = 0; i < kTwoJoints.size(); ++i) {
    EXPECT_EQ(exported_.at(i)->get_prefix_name(), "pid_controller_test/" + kTwoJoints[i]);
    EXPECT_DOUBLE_EQ(exported_.at(i)->get_optional().value(), two_hw_states_[i]);
  }
}

TEST_F(SeededPidControllerTest, SeededPidLeavesNanRatherThanSeedingAnotherWheel)
{
  // Claimed interfaces in a different order than exported (what an upstream reorder would look
  // like): seeding by index would give each wheel the other's velocity.
  const auto pid = activate_two_wheels({1, 0});
  for (const auto & exported : exported_) {
    EXPECT_TRUE(std::isnan(exported->get_optional().value())) << exported->get_name();
  }
}

// Runs one 50 Hz update with `reference` on the PID's reference interface (non-chained mode
// keeps it, since no reference message has arrived); returns what went to the hardware.
double command_after_update(
  const std::shared_ptr<rover_controller::SeededPidController> & pid,
  const hardware_interface::CommandInterface::SharedPtr & reference, double reference_value,
  const double & hw_command, int cycle)
{
  EXPECT_TRUE(reference->set_value(reference_value));
  EXPECT_EQ(
    pid->update(rclcpp::Time(0, 20'000'000 * cycle), rclcpp::Duration::from_seconds(0.02)),
    controller_interface::return_type::OK);
  return hw_command;
}

TEST_F(SeededPidControllerTest, WithoutTheStopOptionALeftoverIntegralReachesTheHardware)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  exported_after_activation(0.0, pid);
  hw_state_ = 1.0;
  for (int k = 1; k <= 25; ++k) {
    command_after_update(pid, reference_.at(0), 2.0, hw_command_, k);
  }
  hw_state_ = 0.0;
  // Twice at zero, so the derivative of the step down is gone and only the integral is left.
  command_after_update(pid, reference_.at(0), 0.0, hw_command_, 26);
  EXPECT_GT(command_after_update(pid, reference_.at(0), 0.0, hw_command_, 27), 0.1);
}

TEST_F(SeededPidControllerTest, StopAtZeroReferenceSendsExactlyZeroToTheHardware)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  exported_after_activation(0.0, pid, {{"stop_at_zero_reference", true}});
  hw_state_ = 1.0;
  for (int k = 1; k <= 25; ++k) {
    command_after_update(pid, reference_.at(0), 2.0, hw_command_, k);
  }
  EXPECT_GT(hw_command_, 2.0);
  hw_state_ = 0.8;  // still coasting
  EXPECT_EQ(command_after_update(pid, reference_.at(0), 0.0, hw_command_, 26), 0.0);
}

TEST_F(SeededPidControllerTest, ScaledIntegralFadesWithTheReferenceOnceSetAtRuntime)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  exported_after_activation(0.0, pid);
  ASSERT_TRUE(pid->get_node()->set_parameter({"scale_integral_with_reference", true}).successful);
  hw_state_ = 1.0;
  for (int k = 1; k <= 25; ++k) {
    command_after_update(pid, reference_.at(0), 2.0, hw_command_, k);
  }
  ASSERT_GT(hw_command_ - 2.0, 0.1);  // the integral the slow wheel built up
  // Reference ramped down to 1 % of where the integral was built; the wheel follows it. Twice,
  // so the derivative of the step is gone: what is left is feed-forward + 1 % of the integral.
  hw_state_ = 0.02;
  command_after_update(pid, reference_.at(0), 0.02, hw_command_, 26);
  EXPECT_NEAR(command_after_update(pid, reference_.at(0), 0.02, hw_command_, 27), 0.02, 0.01);
}

TEST_F(SeededPidControllerTest, WheelLoopParametersAreValidatedAtRuntime)
{
  std::shared_ptr<rover_controller::SeededPidController> pid;
  exported_after_activation(0.0, pid);
  auto node = pid->get_node();
  EXPECT_TRUE(node->set_parameter({"integral_reference_delay", 0.25}).successful);
  EXPECT_TRUE(node->set_parameter({"integral_reference_time_constant", 0.15}).successful);
  EXPECT_FALSE(node->set_parameter({"integral_reference_delay", -0.1}).successful);
  EXPECT_FALSE(node->set_parameter({"integral_reference_delay", 5.0}).successful);
  EXPECT_FALSE(node->set_parameter({"zero_reference_tolerance", -1.0}).successful);
  EXPECT_DOUBLE_EQ(node->get_parameter("integral_reference_delay").as_double(), 0.25);
}

TEST_F(SeededPidControllerTest, OutOfRangeWheelLoopParameterFailsInit)
{
  auto controller = std::make_shared<rover_controller::SeededPidController>();
  controller_interface::ControllerInterfaceParams params;
  params.controller_name = "pid_controller_test";
  params.update_rate = 50;
  params.controller_manager_update_rate = 50;
  params.node_options = rclcpp::NodeOptions().parameter_overrides({
    {"dof_names", std::vector<std::string>{kJoint}},
    {"command_interface", "velocity"},
    {"reference_and_state_interfaces", std::vector<std::string>{"velocity"}},
    {"integral_reference_delay", 3.0},
  });
  EXPECT_EQ(controller->init(params), controller_interface::return_type::ERROR);
}

}  // namespace
