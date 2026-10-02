// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#ifndef ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_
#define ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_

#include <atomic>
#include <vector>

#include <pid_controller/pid_controller.hpp>
#include <rclcpp/node_interfaces/node_parameters_interface.hpp>

#include "rover_controller/wheel_speed_loop.hpp"

namespace rover_controller
{

/**
 * @brief pid_controller::PidController whose exported state is valid from the first cycle, and
 * whose wheel loop stops cleanly at zero and doesn't wind up through the plant's dead time.
 *
 * The rover's drive chain is diff_drive_controller -> one PID per wheel -> hardware, with
 * diff_drive reading wheel feedback from each PID's exported velocity state
 * (config/wheel_01_controller.yaml, open_loop: false).
 *
 * Seeding: upstream PidController::on_activate() resets that exported state to NaN and only
 * fills it in its own update(). A PID can switch into chained mode only while inactive, so every
 * time diff_drive joins the chain the PIDs are (re)activated, and on real hardware diff_drive
 * updates first in that very cycle: it reads NaN, fails with "Either the left or right wheel
 * velocity is invalid", and controller_manager deactivates the whole chain. (In Gazebo the
 * first diff_drive update happened to come later, which is why only the rover failed.)
 * This subclass runs the upstream activation, then seeds the exported state - and the
 * measured state backing it - from the hardware state interfaces it has just claimed.
 *
 * Wheel loop: for the rover's configuration (one velocity interface per wheel, measured from the
 * state interfaces) each wheel's command comes from a WheelSpeedLoop instead of the upstream
 * control_toolbox::Pid, using the same gains.<dof>.* parameters. With its extra parameters at
 * their defaults it computes exactly what upstream does. The extra parameters (runtime-settable):
 *   stop_at_zero_reference            output exactly 0 and clear the integral at a zero reference
 *   zero_reference_tolerance          rad/s under which the reference counts as zero
 *   integral_reference_delay          s, dead time of the reference the integral compares against
 *   integral_reference_time_constant  s, first-order lag of that reference
 *   scale_integral_with_reference     fade the integral with |reference| as it ramps down
 * Any other configuration (external measured states, position + velocity references, angle
 * wraparound) keeps the upstream update unchanged.
 */
class SeededPidController : public pid_controller::PidController
{
public:
  /** Longest integral_reference_delay accepted; sizes the reference history. */
  static constexpr double kMaxIntegralReferenceDelay = 1.0;

  controller_interface::CallbackReturn on_init() override;

  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  controller_interface::return_type update_and_write_commands(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  /** @brief Copy finite hardware state values into the measured and exported state. */
  void seed_exported_state_from_hardware();

  /** @brief True when the wheel loops can replace the upstream update for this configuration. */
  bool uses_wheel_loops() const;

  WheelLoopGains wheel_gains(size_t dof_index) const;
  WheelLoopOptions wheel_options() const;

  std::vector<WheelSpeedLoop> wheel_loops_;

  std::atomic<bool> stop_at_zero_reference_{false};
  std::atomic<double> zero_reference_tolerance_{1e-3};
  std::atomic<double> integral_reference_delay_{0.0};
  std::atomic<double> integral_reference_time_constant_{0.0};
  std::atomic<bool> scale_integral_with_reference_{false};

private:
  rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr on_set_parameters_handle_;
};

}  // namespace rover_controller

#endif  // ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_
