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

#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <vector>

#include "rover_crsf_teleop/application/teleop_use_case.hpp"

namespace rover_crsf_teleop
{

class FakeVelocityCommandPort : public VelocityCommandPort
{

public:

    void publish(const VelocityCommand & command) override { published.push_back(command); }

    std::vector<VelocityCommand> published;
};

class FakeSafetySwitchPort : public SafetySwitchPort
{

public:

    void requestUserEStopSet() override { e_stop_set_calls++; }

    void requestUserEStopReset() override { e_stop_reset_calls++; }

    void requestLatchReset() override { latch_reset_calls++; }

    int e_stop_set_calls{0};
    int e_stop_reset_calls{0};
    int latch_reset_calls{0};
};

namespace
{

using namespace std::chrono_literals;

constexpr int kLinearChannel = 3;
constexpr int kAngularChannel = 1;
constexpr int kEStopChannel = 5;
constexpr int kLatchResetChannel = 4;

constexpr int kSwitchLow = kDefaultCrsfChannelMin;
constexpr int kSwitchHigh = kDefaultCrsfChannelMax;

}  // namespace

class TeleopUseCaseTest : public ::testing::Test
{

protected:

    void SetUp() override
    {
        config_.linear_x_mapping.out_min = -2.0;
        config_.linear_x_mapping.out_max = 2.0;
        config_.angular_z_mapping.out_min = -5.0;
        config_.angular_z_mapping.out_max = 5.0;
        config_.linear_x_channel = kLinearChannel;
        config_.angular_z_channel = kAngularChannel;
        config_.e_stop_channel = kEStopChannel;
        config_.e_stop_latch_reset_channel = kLatchResetChannel;
        config_.switch_settle_frames = 0;
        config_.link.channel_timeout = 200ms;
        config_.link.link_stats_timeout = 1000ms;

        frame_.channels.fill(kDefaultCrsfChannelMid);
        setChannel(kEStopChannel, kSwitchHigh);
        setChannel(kLatchResetChannel, kSwitchHigh);

        use_case_ = std::make_unique<TeleopUseCase>(config_, velocity_, safety_);
    }

    void setChannel(const int channel_number, const int value)
    {
        frame_.channels[static_cast<std::size_t>(channel_number - 1)] = value;
    }

    // Delivers the current frame with a good link report and ticks, all at `now_`.
    TickStatus feedAndTick()
    {
        use_case_->onChannels(frame_, now_);
        use_case_->onLinkStats(100, now_);
        return use_case_->tick(now_);
    }

    TeleopConfig config_;
    std::shared_ptr<FakeVelocityCommandPort> velocity_ = std::make_shared<FakeVelocityCommandPort>();
    std::shared_ptr<FakeSafetySwitchPort> safety_ = std::make_shared<FakeSafetySwitchPort>();
    std::unique_ptr<TeleopUseCase> use_case_;

    RcFrame frame_;
    SteadyTime now_{};
};

TEST(ApplyCalibrationTest, BothAxesTakeTheEndpointsOfTheChannelTheyRead)
{
    TeleopConfig config;
    config.linear_x_channel = kLinearChannel;
    config.angular_z_channel = kAngularChannel;
    config.linear_x_mapping.out_max = 1.2;
    config.linear_x_mapping.out_min = -1.2;
    config.angular_z_mapping.out_max = 1.0;
    config.angular_z_mapping.invert = true;

    // The two sticks rest in different places - the whole point of per-channel calibration.
    ChannelCalibration calibration = defaultCalibration();
    calibration.in_mid[kLinearChannel - 1] = 1004;
    calibration.deadband[kLinearChannel - 1] = 14;
    calibration.in_mid[kAngularChannel - 1] = 987;
    calibration.deadband[kAngularChannel - 1] = 7;

    const TeleopConfig calibrated = applyCalibration(config, calibration);

    EXPECT_EQ(calibrated.linear_x_mapping.in_mid, 1004);
    EXPECT_EQ(calibrated.linear_x_mapping.deadband_counts, 14);
    EXPECT_EQ(calibrated.angular_z_mapping.in_mid, 987);
    EXPECT_EQ(calibrated.angular_z_mapping.deadband_counts, 7);

    // The parameters' business, not the calibration's.
    EXPECT_DOUBLE_EQ(calibrated.linear_x_mapping.out_max, 1.2);
    EXPECT_DOUBLE_EQ(calibrated.angular_z_mapping.out_max, 1.0);
    EXPECT_TRUE(calibrated.angular_z_mapping.invert);

    // Carried whole, so it can be persisted and displayed for every channel, not just the two
    // that drive something.
    EXPECT_EQ(calibrated.calibration.in_mid[kEStopChannel - 1], calibration.in_mid[kEStopChannel - 1]);
}

TEST(ApplyCalibrationTest, ApplyingTwiceIsTheSameAsApplyingOnce)
{
    TeleopConfig config;
    config.linear_x_channel = kLinearChannel;
    config.angular_z_channel = kAngularChannel;

    ChannelCalibration first = defaultCalibration();
    first.in_mid[kLinearChannel - 1] = 1004;

    ChannelCalibration second = defaultCalibration();
    second.in_mid[kLinearChannel - 1] = 970;
    second.deadband[kLinearChannel - 1] = 20;

    // on_configure applies the stored calibration onto a config that already carries the
    // parameter one; the endpoints must be replaced, never accumulated.
    const TeleopConfig once = applyCalibration(config, second);
    const TeleopConfig stacked = applyCalibration(applyCalibration(config, first), second);

    EXPECT_EQ(stacked.linear_x_mapping.in_mid, once.linear_x_mapping.in_mid);
    EXPECT_EQ(stacked.linear_x_mapping.deadband_counts, once.linear_x_mapping.deadband_counts);
    EXPECT_EQ(stacked.calibration.in_mid[kLinearChannel - 1], 970);
}

TEST(ApplyCalibrationTest, LeavesTheInputUntouched)
{
    TeleopConfig base;
    base.linear_x_channel = kLinearChannel;
    base.angular_z_channel = kAngularChannel;
    const int before = base.linear_x_mapping.in_mid;

    ChannelCalibration calibration = defaultCalibration();
    calibration.in_mid[kLinearChannel - 1] = 1004;

    const TeleopConfig calibrated = applyCalibration(base, calibration);

    // rebuildTeleop() relies on this: it applies onto base_config_ every time, so base_config_
    // must still be the pre-calibration base afterwards.
    EXPECT_EQ(base.linear_x_mapping.in_mid, before);
    EXPECT_EQ(calibrated.linear_x_mapping.in_mid, 1004);
}

TEST_F(TeleopUseCaseTest, NothingPublishedBeforeTheFirstFrame)
{
    EXPECT_EQ(use_case_->tick(now_), TickStatus::kWaitingForFirstFrame);
    EXPECT_TRUE(velocity_->published.empty());
}

TEST_F(TeleopUseCaseTest, DeflectedStickPublishesEveryTick)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);

    EXPECT_EQ(feedAndTick(), TickStatus::kActive);
    EXPECT_EQ(feedAndTick(), TickStatus::kActive);

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_DOUBLE_EQ(velocity_->published[0].linear_x, 2.0);
    EXPECT_DOUBLE_EQ(velocity_->published[1].linear_x, 2.0);
}

TEST_F(TeleopUseCaseTest, FullForwardAndTurnIsScaledToTheRimSpeedBudget)
{
    config_.max_wheel_rim_speed = 1.7;
    config_.half_track_width = 0.5;
    use_case_ = std::make_unique<TeleopUseCase>(config_, velocity_, safety_);
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    setChannel(kAngularChannel, kDefaultCrsfChannelMax);

    feedAndTick();

    // Unlimited this is 2.0 m/s + 5.0 rad/s: an outer rim speed of 4.5 m/s.
    ASSERT_EQ(velocity_->published.size(), 1u);
    const auto & command = velocity_->published[0];
    EXPECT_NEAR(command.linear_x + command.angular_z * 0.5, 1.7, 1e-12);
    EXPECT_NEAR(command.angular_z / command.linear_x, 5.0 / 2.0, 1e-12);
}

TEST_F(TeleopUseCaseTest, CentredStickPublishesZeroOnlyOnce)
{
    feedAndTick();
    feedAndTick();
    feedAndTick();

    ASSERT_EQ(velocity_->published.size(), 1u);
    EXPECT_TRUE(velocity_->published[0].isZero());
}

TEST_F(TeleopUseCaseTest, FirstDeflectionAfterZeroIsPublishedImmediately)
{
    // The previous implementation skipped the first non-zero command after a zero.
    feedAndTick();
    setChannel(kAngularChannel, kDefaultCrsfChannelMin);
    feedAndTick();

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_FALSE(velocity_->published[1].isZero());
}

TEST_F(TeleopUseCaseTest, LinkLossPublishesOneZeroThenStaysSilent)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    feedAndTick();

    // No more frames: the channel timeout expires.
    now_ += 201ms;
    EXPECT_EQ(use_case_->tick(now_), TickStatus::kLinkLost);
    now_ += 20ms;
    EXPECT_EQ(use_case_->tick(now_), TickStatus::kLinkLost);

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_TRUE(velocity_->published[1].isZero());
}

TEST_F(TeleopUseCaseTest, LowLinkQualityStopsDespiteFreshChannels)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    feedAndTick();

    use_case_->onChannels(frame_, now_);
    use_case_->onLinkStats(0, now_);
    EXPECT_EQ(use_case_->tick(now_), TickStatus::kLinkLost);

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_TRUE(velocity_->published[1].isZero());
}

TEST_F(TeleopUseCaseTest, RecoveryResumesCommanding)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    feedAndTick();
    now_ += 201ms;
    use_case_->tick(now_);

    EXPECT_EQ(feedAndTick(), TickStatus::kActive);

    ASSERT_EQ(velocity_->published.size(), 3u);
    EXPECT_DOUBLE_EQ(velocity_->published[2].linear_x, 2.0);
}

TEST_F(TeleopUseCaseTest, CentredStickAfterLinkLossDoesNotRepublishZero)
{
    feedAndTick();
    now_ += 201ms;
    use_case_->tick(now_);
    feedAndTick();

    EXPECT_EQ(velocity_->published.size(), 1u);
}

TEST_F(TeleopUseCaseTest, EStopSwitchEdgesCallTheMatchingRequest)
{
    feedAndTick();

    setChannel(kEStopChannel, kSwitchLow);
    feedAndTick();
    feedAndTick();
    EXPECT_EQ(safety_->e_stop_set_calls, 1);

    setChannel(kEStopChannel, kSwitchHigh);
    feedAndTick();
    EXPECT_EQ(safety_->e_stop_reset_calls, 1);
    EXPECT_EQ(safety_->latch_reset_calls, 0);
}

TEST_F(TeleopUseCaseTest, LatchResetFiresOnlyOnTheLowEdge)
{
    feedAndTick();

    setChannel(kLatchResetChannel, kSwitchLow);
    feedAndTick();
    setChannel(kLatchResetChannel, kSwitchHigh);
    feedAndTick();

    EXPECT_EQ(safety_->latch_reset_calls, 1);
    EXPECT_EQ(safety_->e_stop_set_calls, 0);
    EXPECT_EQ(safety_->e_stop_reset_calls, 0);
}

TEST_F(TeleopUseCaseTest, SwitchesAreIgnoredWhileTheLinkIsLost)
{
    feedAndTick();

    setChannel(kEStopChannel, kSwitchLow);
    use_case_->onChannels(frame_, now_);
    use_case_->onLinkStats(0, now_);
    use_case_->tick(now_);

    EXPECT_EQ(safety_->e_stop_set_calls, 0);

    // The flip is honoured once the link is back.
    feedAndTick();
    EXPECT_EQ(safety_->e_stop_set_calls, 1);
}

TEST_F(TeleopUseCaseTest, StopPublishesZeroOnlyIfStillMoving)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    feedAndTick();

    use_case_->stop();
    use_case_->stop();

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_TRUE(velocity_->published[1].isZero());
}

TEST_F(TeleopUseCaseTest, InvalidAxisChannelCommandsNothingOnThatAxis)
{
    config_.linear_x_channel = 17;
    use_case_ = std::make_unique<TeleopUseCase>(config_, velocity_, safety_);
    setChannel(kAngularChannel, kDefaultCrsfChannelMax);

    feedAndTick();

    ASSERT_EQ(velocity_->published.size(), 1u);
    EXPECT_DOUBLE_EQ(velocity_->published[0].linear_x, 0.0);
}

TEST_F(TeleopUseCaseTest, DiagnosticsBeforeFirstFrameWarn)
{
    const auto diagnostics = use_case_->diagnostics(now_);

    EXPECT_FALSE(diagnostics.first_frame_received);
    EXPECT_EQ(diagnostics.health.level, HealthLevel::kWarn);
    EXPECT_FALSE(diagnostics.e_stop_switch.has_value());
}

TEST_F(TeleopUseCaseTest, DiagnosticsReportHealthyLinkAndLastCommand)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);

    const auto diagnostics = use_case_->diagnostics(now_);

    EXPECT_TRUE(diagnostics.first_frame_received);
    EXPECT_EQ(diagnostics.health.level, HealthLevel::kOk);
    EXPECT_DOUBLE_EQ(diagnostics.last_command.linear_x, 2.0);
    ASSERT_TRUE(diagnostics.e_stop_switch.has_value());
    EXPECT_EQ(*diagnostics.e_stop_switch, SwitchPosition::kHigh);
}

TEST_F(TeleopUseCaseTest, DiagnosticsAgreeWithTickOnLinkLoss)
{
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);

    const auto later = now_ + 500ms;
    ASSERT_EQ(use_case_->tick(later), TickStatus::kLinkLost);

    const auto diagnostics = use_case_->diagnostics(later);

    EXPECT_EQ(diagnostics.health.level, HealthLevel::kWarn);
    EXPECT_EQ(diagnostics.link.loss_reason, LinkLossReason::kChannelsStale);
    EXPECT_TRUE(diagnostics.last_command.isZero());
}

TEST_F(TeleopUseCaseTest, AnInhibitedTeleopPublishesOneZeroAndThenNothing)
{
    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);
    ASSERT_EQ(velocity_->published.size(), 1u);

    use_case_->setCommandInhibited(true);

    EXPECT_EQ(feedAndTick(), TickStatus::kInhibited);
    EXPECT_EQ(feedAndTick(), TickStatus::kInhibited);

    ASSERT_EQ(velocity_->published.size(), 2u);
    EXPECT_TRUE(velocity_->published.back().isZero());
}

TEST_F(TeleopUseCaseTest, AnInhibitedTeleopIgnoresTheSticksEvenBeforeTheFirstFrame)
{
    // The inhibit has to dominate every branch of tick(), including the ones that come before the
    // stick mapping - there must be no route that commands while a calibration is running.
    use_case_->setCommandInhibited(true);
    EXPECT_EQ(use_case_->tick(now_), TickStatus::kInhibited);

    setChannel(kLinearChannel, kDefaultCrsfChannelMax);
    EXPECT_EQ(feedAndTick(), TickStatus::kInhibited);

    for (const auto & command : velocity_->published) {
        EXPECT_TRUE(command.isZero());
    }
}

TEST_F(TeleopUseCaseTest, SweepingTheEStopSwitchWhileInhibitedCallsNoSafetyServices)
{
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);

    use_case_->setCommandInhibited(true);

    // Exactly what an RC calibration sweep does to the switch channels.
    for (const int value : {kSwitchLow, kSwitchHigh, kSwitchLow, kSwitchHigh}) {
        setChannel(kEStopChannel, value);
        setChannel(kLatchResetChannel, value);
        EXPECT_EQ(feedAndTick(), TickStatus::kInhibited);
    }

    EXPECT_EQ(safety_->e_stop_set_calls, 0);
    EXPECT_EQ(safety_->e_stop_reset_calls, 0);
    EXPECT_EQ(safety_->latch_reset_calls, 0);
}

TEST_F(TeleopUseCaseTest, ReleasingTheInhibitAfterRearmingDoesNotFireAStaleEdge)
{
    // The switch rests high; the sweep leaves it low. Re-arming makes the debouncer treat that
    // as the new resting position instead of as an edge.
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);

    use_case_->setCommandInhibited(true);
    setChannel(kEStopChannel, kSwitchLow);
    ASSERT_EQ(feedAndTick(), TickStatus::kInhibited);

    use_case_->rearmSwitches();
    use_case_->setCommandInhibited(false);

    EXPECT_EQ(feedAndTick(), TickStatus::kActive);
    EXPECT_EQ(safety_->e_stop_set_calls, 0);

    // A genuine flip after the re-arm still gets through.
    setChannel(kEStopChannel, kSwitchHigh);
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);
    EXPECT_EQ(safety_->e_stop_reset_calls, 1);
}

TEST_F(TeleopUseCaseTest, DiagnosticsReportTheInhibit)
{
    ASSERT_EQ(feedAndTick(), TickStatus::kActive);
    EXPECT_FALSE(use_case_->diagnostics(now_).inhibited);

    use_case_->setCommandInhibited(true);
    EXPECT_TRUE(use_case_->diagnostics(now_).inhibited);
}

}  // namespace rover_crsf_teleop
