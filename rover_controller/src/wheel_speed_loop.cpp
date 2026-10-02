// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#include "rover_controller/wheel_speed_loop.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace rover_controller
{

namespace
{

// control_toolbox::Pid::set_gains() picks this when back calculation is given no time constant.
double tracking_time_constant(const WheelLoopGains & gains)
{
  if (gains.tracking_time_constant > 0.0 || gains.i == 0.0) {
    return gains.tracking_time_constant;
  }
  return gains.d != 0.0 ? std::sqrt(gains.d / gains.i) : gains.p / gains.i;
}

}  // namespace

WheelSpeedLoop::WheelSpeedLoop(std::size_t history_capacity)
: times_(std::max<std::size_t>(history_capacity, 1)),
  values_(std::max<std::size_t>(history_capacity, 1))
{
}

void WheelSpeedLoop::reset()
{
  clear_history(0.0);
  model_ = 0.0;
  i_term_ = 0.0;
  last_reference_ = 0.0;
  reference_peak_ = 0.0;
  last_error_ = 0.0;
  last_output_ = 0.0;
}

void WheelSpeedLoop::clear_history(double value)
{
  next_ = 0;
  count_ = 0;
  before_history_ = value;
}

void WheelSpeedLoop::push_reference(double reference)
{
  if (count_ == values_.size()) {
    before_history_ = values_[next_];  // evicting the oldest sample
  } else {
    ++count_;
  }
  times_[next_] = clock_;
  values_[next_] = reference;
  next_ = (next_ + 1) % values_.size();
}

double WheelSpeedLoop::reference_at(double time) const
{
  // Newest to oldest: the first sample at or before `time` is the reference in force then.
  for (std::size_t k = 1; k <= count_; ++k) {
    const std::size_t index = (next_ + values_.size() - k) % values_.size();
    if (times_[index] <= time + 1e-9) {
      return values_[index];
    }
  }
  return before_history_;
}

void WheelSpeedLoop::scale_integral(double reference)
{
  const double magnitude = std::abs(reference);
  if (reference * last_reference_ < 0.0) {
    // Reversal: the trim was for the other direction.
    i_term_ = 0.0;
    reference_peak_ = magnitude;
  } else if (last_reference_ != 0.0) {
    // last |reference| never exceeds the peak, so this scales down while the reference falls
    // and back up (at most to the peak's integral) while it recovers.
    i_term_ *= std::min(magnitude, reference_peak_) / std::abs(last_reference_);
    reference_peak_ = std::max(reference_peak_, magnitude);
  } else {
    reference_peak_ = magnitude;
  }
  last_reference_ = reference;
}

double WheelSpeedLoop::update(
  double reference, double measured, double dt, const WheelLoopGains & gains,
  const WheelLoopOptions & options)
{
  if (!(dt > 0.0)) {
    return last_output_;
  }
  const double error = reference - measured;
  if (!std::isfinite(error)) {
    return last_output_ = std::numeric_limits<double>::quiet_NaN();
  }
  clock_ += dt;

  if (options.stop_at_zero_reference &&
    std::abs(reference) <= options.zero_reference_tolerance)
  {
    // Keep the derivative memory so the next start sees the same D kick as upstream.
    clear_history(0.0);
    model_ = 0.0;
    i_term_ = 0.0;
    last_reference_ = 0.0;
    reference_peak_ = 0.0;
    last_error_ = error;
    return last_output_ = 0.0;
  }

  if (options.scale_integral_with_reference) {
    scale_integral(reference);
    i_term_ = std::clamp(i_term_, gains.i_min, gains.i_max);
  }

  // Reference the integral compares against: delayed, then first-order lagged.
  push_reference(reference);
  const double delayed = options.integral_reference_delay > 0.0 ?
    reference_at(clock_ - options.integral_reference_delay) : reference;
  if (options.integral_reference_time_constant > 0.0) {
    model_ += (delayed - model_) *
      (1.0 - std::exp(-dt / options.integral_reference_time_constant));
  } else {
    model_ = delayed;
  }
  const double integral_error = model_ - measured;

  // From here on the same arithmetic as control_toolbox::Pid::compute_command(error, dt).
  const double error_dot = (error - last_error_) / dt;
  last_error_ = error;

  const double unsaturated = gains.p * error + i_term_ + gains.d * error_dot;
  double command = unsaturated;
  if (std::isfinite(gains.u_min) || std::isfinite(gains.u_max)) {
    command = std::clamp(unsaturated, gains.u_min, gains.u_max);
  }

  if (std::abs(integral_error) > gains.error_deadband) {
    switch (gains.anti_windup) {
      case WheelLoopGains::AntiWindup::kBackCalculation:
        if (gains.i != 0.0) {
          const double tt = tracking_time_constant(gains);
          i_term_ += dt * (gains.i * integral_error +
            (tt > 0.0 ? (command - unsaturated) / tt : 0.0));
        }
        break;
      case WheelLoopGains::AntiWindup::kConditionalIntegration:
        if (!(command != unsaturated && integral_error * unsaturated > 0.0)) {
          i_term_ += dt * gains.i * integral_error;
        }
        break;
      case WheelLoopGains::AntiWindup::kNone:
        i_term_ += dt * gains.i * integral_error;
        break;
    }
  }
  i_term_ = std::clamp(i_term_, gains.i_min, gains.i_max);

  return last_output_ = gains.feedforward * reference + command;
}

}  // namespace rover_controller
