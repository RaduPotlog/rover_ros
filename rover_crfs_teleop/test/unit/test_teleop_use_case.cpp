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

#include "rover_crfs_teleop/application/teleop_use_case.hpp"

namespace rover_crfs_teleop
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

}  // namespace rover_crfs_teleop
