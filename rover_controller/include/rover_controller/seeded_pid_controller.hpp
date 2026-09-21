// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#ifndef ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_
#define ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_

#include <pid_controller/pid_controller.hpp>

namespace rover_controller
{

/**
 * @brief pid_controller::PidController whose exported state is valid from the first cycle.
 *
 * The rover's drive chain is diff_drive_controller -> one PID per wheel -> hardware, with
 * diff_drive reading wheel feedback from each PID's exported velocity state
 * (config/wheel_01_controller.yaml, open_loop: false).
 *
 * Upstream PidController::on_activate() resets that exported state to NaN and only fills it in
 * its own update(). A PID can switch into chained mode only while inactive, so every time
 * diff_drive joins the chain the PIDs are (re)activated, and on real hardware diff_drive
 * updates first in that very cycle: it reads NaN, fails with "Either the left or right wheel
 * velocity is invalid", and controller_manager deactivates the whole chain. (In Gazebo the
 * first diff_drive update happened to come later, which is why only the rover failed.)
 *
 * This subclass runs the upstream activation, then seeds the exported state - and the
 * measured state backing it - from the hardware state interfaces it has just claimed.
 * Everything else is upstream behaviour: from the first update() on, the PID overwrites
 * both with the fresh measurement as before.
 */
class SeededPidController : public pid_controller::PidController
{
public:
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

protected:
  /** @brief Copy finite hardware state values into the measured and exported state. */
  void seed_exported_state_from_hardware();
};

}  // namespace rover_controller

#endif  // ROVER_CONTROLLER__SEEDED_PID_CONTROLLER_HPP_
