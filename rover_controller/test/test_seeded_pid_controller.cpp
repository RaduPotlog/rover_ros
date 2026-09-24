// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.
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
  double exported_after_activation(double hw_velocity, std::shared_ptr<ControllerT> & controller)
  {
    hw_state_ = hw_velocity;
    hw_command_ = 0.0;

    controller = std::make_shared<ControllerT>();
    controller_interface::ControllerInterfaceParams params;
    params.controller_name = "pid_controller_test";
    params.update_rate = 100;
    params.controller_manager_update_rate = 100;
    params.node_options = rclcpp::NodeOptions().parameter_overrides({
      {"dof_names", std::vector<std::string>{kJoint}},
      {"command_interface", "velocity"},
      {"reference_and_state_interfaces", std::vector<std::string>{"velocity"}},
      {"gains." + kJoint + ".p", 0.05},
      {"gains." + kJoint + ".i", 1.0},
    });
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

}  // namespace
