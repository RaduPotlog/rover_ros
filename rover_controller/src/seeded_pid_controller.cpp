// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#include "rover_controller/seeded_pid_controller.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>
#include <vector>

#include <pluginlib/class_list_macros.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>

namespace rover_controller
{

namespace
{

constexpr char kStopAtZeroReference[] = "stop_at_zero_reference";
constexpr char kZeroReferenceTolerance[] = "zero_reference_tolerance";
constexpr char kIntegralReferenceDelay[] = "integral_reference_delay";
constexpr char kIntegralReferenceTimeConstant[] = "integral_reference_time_constant";

// Why `value` is not acceptable for parameter `name`, or empty if it is.
std::string reject_reason(const std::string & name, double value)
{
  if (!std::isfinite(value) || value < 0.0) {
    return name + " must be finite and >= 0";
  }
  if (name == kIntegralReferenceDelay &&
    value > SeededPidController::kMaxIntegralReferenceDelay)
  {
    return name + " must be <= " +
           std::to_string(SeededPidController::kMaxIntegralReferenceDelay) + " s";
  }
  return {};
}

}  // namespace

controller_interface::CallbackReturn SeededPidController::on_init()
{
  const auto result = pid_controller::PidController::on_init();
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  try {
    auto_declare<bool>(kStopAtZeroReference, stop_at_zero_reference_.load());
    auto_declare<double>(kZeroReferenceTolerance, zero_reference_tolerance_.load());
    auto_declare<double>(kIntegralReferenceDelay, integral_reference_delay_.load());
    auto_declare<double>(
      kIntegralReferenceTimeConstant, integral_reference_time_constant_.load());
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "Declaring the wheel loop parameters failed: %s", e.what());
    return controller_interface::CallbackReturn::ERROR;
  }

  // Values from the parameter file: validated here, then kept current by the callback below.
  for (const char * name :
    {kZeroReferenceTolerance, kIntegralReferenceDelay, kIntegralReferenceTimeConstant})
  {
    const auto reason = reject_reason(name, get_node()->get_parameter(name).as_double());
    if (!reason.empty()) {
      RCLCPP_ERROR(get_node()->get_logger(), "%s", reason.c_str());
      return controller_interface::CallbackReturn::ERROR;
    }
  }
  stop_at_zero_reference_ = get_node()->get_parameter(kStopAtZeroReference).as_bool();
  zero_reference_tolerance_ = get_node()->get_parameter(kZeroReferenceTolerance).as_double();
  integral_reference_delay_ = get_node()->get_parameter(kIntegralReferenceDelay).as_double();
  integral_reference_time_constant_ =
    get_node()->get_parameter(kIntegralReferenceTimeConstant).as_double();

  // Runtime changes (ros2 param set) take effect on the next update, like the gains do.
  on_set_parameters_handle_ = get_node()->add_on_set_parameters_callback(
    [this](const std::vector<rclcpp::Parameter> & parameters) {
      rcl_interfaces::msg::SetParametersResult result;
      result.successful = true;
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == kStopAtZeroReference) {
          continue;  // the type check is rclcpp's
        }
        if (name == kZeroReferenceTolerance || name == kIntegralReferenceDelay ||
        name == kIntegralReferenceTimeConstant)
        {
          result.reason = reject_reason(name, parameter.as_double());
          if (!result.reason.empty()) {
            result.successful = false;
            return result;
          }
        }
      }
      for (const auto & parameter : parameters) {
        const auto & name = parameter.get_name();
        if (name == kStopAtZeroReference) {
          stop_at_zero_reference_ = parameter.as_bool();
        } else if (name == kZeroReferenceTolerance) {
          zero_reference_tolerance_ = parameter.as_double();
        } else if (name == kIntegralReferenceDelay) {
          integral_reference_delay_ = parameter.as_double();
        } else if (name == kIntegralReferenceTimeConstant) {
          integral_reference_time_constant_ = parameter.as_double();
        }
      }
      return result;
    });
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn SeededPidController::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = pid_controller::PidController::on_configure(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  // Enough reference history for the longest accepted delay at this controller's rate.
  const double rate = std::max(1u, get_update_rate());
  const auto capacity = static_cast<size_t>(std::ceil(kMaxIntegralReferenceDelay * rate)) + 2;
  wheel_loops_.assign(dof_, WheelSpeedLoop(capacity));
  return result;
}

controller_interface::CallbackReturn SeededPidController::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  const auto result = pid_controller::PidController::on_activate(previous_state);
  if (result != controller_interface::CallbackReturn::SUCCESS) {
    return result;
  }
  seed_exported_state_from_hardware();
  // Same rule upstream applies to its PIDs on activation.
  for (size_t i = 0; i < wheel_loops_.size() && i < params_.dof_names.size(); ++i) {
    if (!params_.gains.dof_names_map[params_.dof_names[i]].save_i_term) {
      wheel_loops_[i].reset();
    }
  }
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

bool SeededPidController::uses_wheel_loops() const
{
  if (params_.use_external_measured_states || wheel_loops_.size() != dof_ ||
    ordered_exported_reference_interfaces_.size() != dof_ ||
    measured_state_values_.size() != dof_ || command_interfaces_.size() < dof_ ||
    state_interfaces_.size() < dof_)
  {
    return false;
  }
  for (const auto & dof : params_.dof_names) {
    const auto gains = params_.gains.dof_names_map.find(dof);
    if (gains == params_.gains.dof_names_map.end() || gains->second.angle_wraparound) {
      return false;
    }
  }
  return true;
}

WheelLoopGains SeededPidController::wheel_gains(size_t dof_index) const
{
  const auto & g = params_.gains.dof_names_map.at(params_.dof_names[dof_index]);
  WheelLoopGains gains;
  gains.p = g.p;
  gains.i = g.i;
  gains.d = g.d;
  gains.feedforward = g.feedforward_gain;
  gains.i_min = g.i_clamp_min;
  gains.i_max = g.i_clamp_max;
  gains.u_min = g.u_clamp_min;
  gains.u_max = g.u_clamp_max;
  if (g.antiwindup_strategy == "back_calculation") {
    gains.anti_windup = WheelLoopGains::AntiWindup::kBackCalculation;
  } else if (g.antiwindup_strategy == "conditional_integration") {
    gains.anti_windup = WheelLoopGains::AntiWindup::kConditionalIntegration;
  }
  gains.tracking_time_constant = g.tracking_time_constant;
  gains.error_deadband = g.error_deadband;
  return gains;
}

WheelLoopOptions SeededPidController::wheel_options() const
{
  WheelLoopOptions options;
  options.stop_at_zero_reference = stop_at_zero_reference_;
  options.zero_reference_tolerance = zero_reference_tolerance_;
  options.integral_reference_delay = integral_reference_delay_;
  options.integral_reference_time_constant = integral_reference_time_constant_;
  return options;
}

controller_interface::return_type SeededPidController::update_and_write_commands(
  const rclcpp::Time & time, const rclcpp::Duration & period)
{
  if (!uses_wheel_loops()) {
    return pid_controller::PidController::update_and_write_commands(time, period);
  }

  // Below mirrors upstream PidController::update_and_write_commands() (pid_controller 6.9.0) for
  // one velocity interface per dof; only the command itself comes from the wheel loop.
  param_listener_->try_get_params(params_);

  for (size_t i = 0; i < dof_; ++i) {
    const auto value = state_interfaces_[i].get_optional();
    if (value.has_value()) {
      measured_state_values_[i] = *value;
    }
    // Upstream ignores a failed write here too: the next cycle writes again.
    static_cast<void>(ordered_exported_state_interfaces_[i]->set_value(measured_state_values_[i]));
  }

  const auto options = wheel_options();
  const double dt = period.seconds();
  for (size_t i = 0; i < dof_; ++i) {
    const double reference = ordered_exported_reference_interfaces_[i]->get_optional<double>()
      .value_or(std::numeric_limits<double>::quiet_NaN());
    if (!std::isfinite(reference) || !std::isfinite(measured_state_values_[i])) {
      continue;  // upstream leaves the last command in place too
    }
    const double command =
      wheel_loops_[i].update(reference, measured_state_values_[i], dt, wheel_gains(i), options);
    if (!command_interfaces_[i].set_value(command)) {
      RCLCPP_ERROR(
        get_node()->get_logger(), "Failed to set command value for %s",
        command_interfaces_[i].get_name().c_str());
    }
  }

  if (state_publisher_) {
    state_msg_.header.stamp = time;
    for (size_t i = 0; i < dof_; ++i) {
      const double reference = ordered_exported_reference_interfaces_[i]->get_optional<double>()
        .value_or(std::numeric_limits<double>::quiet_NaN());
      auto & state = state_msg_.dof_states[i];
      state.reference = reference;
      state.feedback = measured_state_values_[i];
      state.error = reference - measured_state_values_[i];
      state.time_step = dt;
      const auto command = command_interfaces_[i].get_optional();
      if (command.has_value()) {
        state.output = *command;
      }
    }
    state_publisher_->try_publish(state_msg_);
  }
  return controller_interface::return_type::OK;
}

}  // namespace rover_controller

PLUGINLIB_EXPORT_CLASS(
  rover_controller::SeededPidController, controller_interface::ChainableControllerInterface)
