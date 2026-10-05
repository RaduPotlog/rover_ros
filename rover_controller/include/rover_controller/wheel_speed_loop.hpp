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

#ifndef ROVER_CONTROLLER__WHEEL_SPEED_LOOP_HPP_
#define ROVER_CONTROLLER__WHEEL_SPEED_LOOP_HPP_

#include <cstddef>
#include <limits>
#include <vector>

namespace rover_controller
{

/** @brief One wheel's PID gains, with the meaning control_toolbox::Pid gives them. */
struct WheelLoopGains
{
  enum class AntiWindup { kNone, kBackCalculation, kConditionalIntegration };

  double p = 0.0;
  double i = 0.0;
  double d = 0.0;
  double feedforward = 0.0;
  double i_min = -std::numeric_limits<double>::infinity();
  double i_max = std::numeric_limits<double>::infinity();
  double u_min = -std::numeric_limits<double>::infinity();
  double u_max = std::numeric_limits<double>::infinity();
  AntiWindup anti_windup = AntiWindup::kNone;
  double tracking_time_constant = 0.0;  // 0 = control_toolbox's default for back calculation
  double error_deadband = 1e-16;
};

/** @brief The rover-specific behaviour on top of the PID. All off = plain control_toolbox::Pid. */
struct WheelLoopOptions
{
  // With a zero reference, output exactly 0 and clear the integral. The DCC1000 brakes only at a
  // target of exactly 0, so a leftover integral otherwise keeps the brake off (or creeps the
  // wheel).
  bool stop_at_zero_reference = false;
  double zero_reference_tolerance = 1e-3;  // rad/s
  // The integral works on (delayed, lagged reference - measurement) instead of the raw error.
  // The encoders (20 Hz) and the DCC1000 duty ramp delay the response ~0.25 s, so the raw error
  // after a step is mostly the plant catching up; integrating it winds the I-term up and
  // overshoots. Against a reference shaped like the plant's own response, only the error the
  // plant will NOT remove by itself (load, friction, skid) is integrated.
  double integral_reference_delay = 0.0;           // s
  double integral_reference_time_constant = 0.0;   // s, first-order lag after the delay
  // Scale the integral with |reference| while the reference is below the largest magnitude
  // reached since it last stopped or reversed, and clear it when the reference changes sign.
  // The trim a skid turn needs is roughly proportional to the commanded speed, so when the
  // reference ramps down to a stop the trim fades with it instead of pushing the wheel on until
  // the reference reaches 0; when the reference comes back up (command noise, a brief slow-down)
  // the trim is restored, so it does not leak away.
  bool scale_integral_with_reference = false;
};

/**
 * @brief Velocity loop for one wheel: feed-forward + PID, stop at zero, model-reference integral.
 *
 * With WheelLoopOptions at their defaults it computes exactly what pid_controller computes with
 * control_toolbox::Pid (single velocity interface): same P/I/D, derivative on the error,
 * integral clamp and all three anti-windup strategies. No ROS dependency; update() doesn't allocate.
 */
class WheelSpeedLoop
{
public:
  /** @param history_capacity reference samples kept for the delay (>= delay * update rate + 1). */
  explicit WheelSpeedLoop(std::size_t history_capacity = 64);

  /** @brief Command (rad/s) for one control period. dt <= 0 repeats the last command. */
  double update(
    double reference, double measured, double dt, const WheelLoopGains & gains,
    const WheelLoopOptions & options);

  /** @brief Clear the integral, the derivative memory and the reference history. */
  void reset();

  double integral() const {return i_term_;}
  /** @brief The reference the integral last compared the measurement with. */
  double integral_reference() const {return model_;}

private:
  void clear_history(double value);
  void push_reference(double reference);
  double reference_at(double time) const;
  void scale_integral(double reference);

  std::vector<double> times_;
  std::vector<double> values_;
  std::size_t next_ = 0;
  std::size_t count_ = 0;
  // The reference before the oldest kept sample: the last one evicted, or the value at reset.
  double before_history_ = 0.0;
  double clock_ = 0.0;

  double model_ = 0.0;
  double i_term_ = 0.0;
  double last_reference_ = 0.0;
  double reference_peak_ = 0.0;  // largest |reference| since the last stop or reversal
  double last_error_ = 0.0;
  double last_output_ = 0.0;
};

}  // namespace rover_controller

#endif  // ROVER_CONTROLLER__WHEEL_SPEED_LOOP_HPP_
