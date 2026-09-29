// Copyright 2026 Rover A1 contributors
// Licensed under the Apache License, Version 2.0.

#include <algorithm>
#include <cmath>
#include <deque>
#include <vector>

#include <gtest/gtest.h>
#include <control_toolbox/pid.hpp>

#include "rover_controller/wheel_speed_loop.hpp"

namespace
{

using rover_controller::WheelLoopGains;
using rover_controller::WheelLoopOptions;
using rover_controller::WheelSpeedLoop;

constexpr double kDt = 0.02;  // the rover's 50 Hz controller rate

// The shipped rover gains (config/wheel_01_controller.yaml, front wheels).
WheelLoopGains rover_gains(double i_clamp = 0.25)
{
  WheelLoopGains g;
  g.p = 0.05;
  g.i = 1.0;
  g.d = 0.04;
  g.feedforward = 1.0;
  g.i_min = -i_clamp;
  g.i_max = i_clamp;
  g.u_min = -12.58;
  g.u_max = 12.58;
  g.anti_windup = WheelLoopGains::AntiWindup::kBackCalculation;
  return g;
}

// A wheel as the ground tests saw it: the command reaches the wheel after a dead time, through a
// first-order lag, scaled by how much of the duty turns into speed (skid turns ~0.7, straight ~1).
class WheelPlant
{
public:
  explicit WheelPlant(double gain, double dead_time = 0.25, double time_constant = 0.12)
  : gain_(gain), time_constant_(time_constant),
    pipe_(static_cast<size_t>(std::round(dead_time / kDt)), 0.0) {}

  double step(double command)
  {
    pipe_.push_back(command);
    const double delayed = pipe_.front();
    pipe_.pop_front();
    speed_ += (gain_ * delayed - speed_) * (1.0 - std::exp(-kDt / time_constant_));
    return speed_;
  }

  double speed() const {return speed_;}

private:
  double gain_;
  double time_constant_;
  std::deque<double> pipe_;
  double speed_ = 0.0;
};

struct StepResult
{
  double overshoot;     // fraction of the step
  double final_speed;   // mean over the last 0.5 s
};

StepResult run_step(
  double target, double plant_gain, const WheelLoopGains & gains,
  const WheelLoopOptions & options, double duration = 4.0)
{
  WheelSpeedLoop loop;
  WheelPlant plant(plant_gain);
  double peak = 0.0;
  double tail = 0.0;
  int tail_samples = 0;
  const int steps = static_cast<int>(duration / kDt);
  for (int k = 0; k < steps; ++k) {
    const double speed = plant.step(loop.update(target, plant.speed(), kDt, gains, options));
    peak = std::max(peak, speed);
    if (k >= steps - static_cast<int>(0.5 / kDt)) {
      tail += speed;
      ++tail_samples;
    }
  }
  return {std::max(0.0, peak / target - 1.0), tail / tail_samples};
}

WheelLoopOptions delayed_integral()
{
  WheelLoopOptions options;
  options.integral_reference_delay = 0.25;
  options.integral_reference_time_constant = 0.15;
  return options;
}

TEST(WheelSpeedLoop, DefaultOptionsMatchControlToolboxPid)
{
  // The upstream loop the rover used until now: feed-forward + control_toolbox::Pid.
  const auto gains = rover_gains();
  control_toolbox::AntiWindupStrategy strategy;
  strategy.set_type("back_calculation");
  strategy.i_min = gains.i_min;
  strategy.i_max = gains.i_max;
  control_toolbox::Pid pid(gains.p, gains.i, gains.d, gains.u_max, gains.u_min, strategy);

  WheelSpeedLoop loop;
  const WheelLoopOptions defaults;
  double measured = 0.0;
  for (int k = 0; k < 400; ++k) {
    const double reference = k < 100 ? 1.8 : (k < 200 ? -3.0 : (k < 300 ? 0.0 : 4.6));
    measured += (0.8 * reference - measured) * 0.1 + 0.01 * std::sin(k);
    const double expected = gains.feedforward * reference + pid.compute_command(
      reference - measured, kDt);
    ASSERT_NEAR(loop.update(reference, measured, kDt, gains, defaults), expected, 1e-12)
      << "cycle " << k;
  }
}

TEST(WheelSpeedLoop, LeftoverIntegralKeepsTheOutputOffZeroWithoutTheStopOption)
{
  // The rover's ground runs: at a zero reference the output stayed at the old integral, so the
  // DCC1000 target was never exactly 0 and the brake never engaged.
  WheelSpeedLoop loop;
  const auto gains = rover_gains();
  for (int k = 0; k < 100; ++k) {
    loop.update(2.0, 1.5, kDt, gains, WheelLoopOptions{});
  }
  EXPECT_NE(loop.update(0.0, 0.0, kDt, gains, WheelLoopOptions{}), 0.0);
}

TEST(WheelSpeedLoop, ZeroReferenceGivesExactlyZeroAndClearsTheIntegral)
{
  WheelSpeedLoop loop;
  const auto gains = rover_gains(2.0);
  WheelLoopOptions options;
  options.stop_at_zero_reference = true;
  for (int k = 0; k < 100; ++k) {
    loop.update(2.0, 1.5, kDt, gains, options);
  }
  ASSERT_GT(loop.integral(), 0.5);

  // Still coasting at 1.2 rad/s: zero command anyway, so the drive brakes.
  EXPECT_EQ(loop.update(0.0, 1.2, kDt, gains, options), 0.0);
  EXPECT_EQ(loop.integral(), 0.0);
  EXPECT_EQ(loop.update(5e-4, 0.3, kDt, gains, options), 0.0);  // inside the tolerance

  // The next start begins from a clean integral: feed-forward + P + D only.
  const double start = loop.update(1.0, 0.0, kDt, gains, options);
  EXPECT_NEAR(start, 1.0 + 0.05 * 1.0 + 0.04 * (1.0 - (5e-4 - 0.3)) / kDt, 1e-9);
}

TEST(WheelSpeedLoop, DelayedIntegralCutsStepOvershootWithAWideClamp)
{
  // A clamp wide enough for skid turns (~2 rad/s) is what made straight steps overshoot on the
  // rover: the raw error integrates through the dead time.
  const auto gains = rover_gains(2.0);
  const auto raw = run_step(1.8, 1.0, gains, WheelLoopOptions{});
  const auto delayed = run_step(1.8, 1.0, gains, delayed_integral());
  EXPECT_GT(raw.overshoot, 0.10);
  EXPECT_LT(delayed.overshoot, 0.05);
  EXPECT_LT(delayed.overshoot, raw.overshoot / 3.0);
  EXPECT_NEAR(delayed.final_speed, 1.8, 0.02 * 1.8);
}

TEST(WheelSpeedLoop, DelayedIntegralStillRemovesAPersistentLoadError)
{
  // Skid turn at 1.5 rad/s: only ~70 % of the duty becomes wheel speed (rover, 2026-09-29), so
  // 4.65 rad/s needs ~2 rad/s of integral on top of the feed-forward.
  const auto gains = rover_gains(2.5);
  const auto delayed = run_step(4.65, 0.7, gains, delayed_integral(), 6.0);
  EXPECT_NEAR(delayed.final_speed, 4.65, 0.03 * 4.65);

  // With the shipped clamp the same turn stays far short, as on the rover.
  const auto shipped = run_step(4.65, 0.7, rover_gains(0.25), delayed_integral(), 6.0);
  EXPECT_LT(shipped.final_speed, 0.8 * 4.65);
}

TEST(WheelSpeedLoop, IntegralReferenceFollowsTheDelayedReference)
{
  WheelSpeedLoop loop;
  WheelLoopOptions options;
  options.integral_reference_delay = 0.1;  // 5 cycles
  const auto gains = rover_gains();
  for (int k = 0; k < 5; ++k) {
    loop.update(1.0, 0.0, kDt, gains, options);
    EXPECT_EQ(loop.integral_reference(), 0.0) << "cycle " << k;
  }
  loop.update(1.0, 0.0, kDt, gains, options);
  EXPECT_EQ(loop.integral_reference(), 1.0);
}

TEST(WheelSpeedLoop, ShortHistoryFallsBackToTheOldestEvictedReference)
{
  WheelSpeedLoop loop(3);
  WheelLoopOptions options;
  options.integral_reference_delay = 1.0;  // far longer than 3 samples
  const auto gains = rover_gains();
  loop.update(1.0, 0.0, kDt, gains, options);
  loop.update(2.0, 0.0, kDt, gains, options);
  loop.update(3.0, 0.0, kDt, gains, options);
  EXPECT_EQ(loop.integral_reference(), 0.0);  // nothing evicted yet: the value at reset
  loop.update(4.0, 0.0, kDt, gains, options);
  EXPECT_EQ(loop.integral_reference(), 1.0);
}

TEST(WheelSpeedLoop, NonPositivePeriodRepeatsTheLastCommand)
{
  WheelSpeedLoop loop;
  const auto gains = rover_gains();
  const double first = loop.update(1.0, 0.2, kDt, gains, WheelLoopOptions{});
  EXPECT_EQ(loop.update(3.0, 0.0, 0.0, gains, WheelLoopOptions{}), first);
}

TEST(WheelSpeedLoop, ResetClearsEverything)
{
  WheelSpeedLoop loop;
  const auto gains = rover_gains();
  for (int k = 0; k < 50; ++k) {
    loop.update(1.0, 0.5, kDt, gains, delayed_integral());
  }
  loop.reset();
  EXPECT_EQ(loop.integral(), 0.0);
  EXPECT_EQ(loop.integral_reference(), 0.0);
}

}  // namespace
