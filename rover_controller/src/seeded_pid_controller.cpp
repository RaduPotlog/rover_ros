// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#include "rover_controller/seeded_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include <pluginlib/class_list_macros.hpp>

namespace rover_controller
{

controller_interface::CallbackReturn SeededPidController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = pid_controller::PidController::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  seed_exported_state_from_hardware();
  return result;
}

void SeededPidController::seed_exported_state_from_hardware()
{
  // With external measured states the PID claims no state interfaces; nothing to seed from.
  if (params_.use_external_measured_states) {
    return;
  }

  // PidController claims one state interface per (interface, dof) in the same order it keeps
  // measured_state_values_ and ordered_exported_state_interfaces_, and its update() copies
  // index for index, so the three vectors line up. That order is upstream's (checked against
  // pid_controller 6.9.0), so each pair's names are compared before seeding: the claimed
  // "<dof>/<interface>" is exported as "<controller>/<dof>/<interface>".
  const auto count = std::min(
    {state_interfaces_.size(), measured_state_values_.size(),
      ordered_exported_state_interfaces_.size()});
  const std::string exported_prefix = std::string(get_node()->get_name()) + "/";

  for (size_t i = 0; i < count; ++i) {
    const auto & claimed_name = state_interfaces_[i].get_name();
    const auto & exported_name = ordered_exported_state_interfaces_[i]->get_name();
    if (exported_name != exported_prefix + claimed_name) {
      RCLCPP_WARN(
        get_node()->get_logger(),
        "Not seeding exported state '%s' from '%s': the interfaces don't line up",
        exported_name.c_str(), claimed_name.c_str());
      continue;
    }
    const auto value = state_interfaces_[i].get_optional();
    if (!value.has_value() || !std::isfinite(*value)) {
      continue;  // leave NaN: better an honest "no data yet" than a made-up zero
    }
    measured_state_values_[i] = *value;
    if (!ordered_exported_state_interfaces_[i]->set_value(*value)) {
      RCLCPP_WARN(
        get_node()->get_logger(), "Could not seed exported state '%s'",
        ordered_exported_state_interfaces_[i]->get_name().c_str());
    }
  }
}

}  // namespace rover_controller

PLUGINLIB_EXPORT_CLASS(
  rover_controller::SeededPidController, controller_interface::ChainableControllerInterface)
