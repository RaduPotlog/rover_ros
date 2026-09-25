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

#include <gtest/gtest.h>

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <utility>
#include <vector>

#include "rover_crsf_teleop/application/teleop_config_validation.hpp"
#include "rover_crsf_teleop/application/teleop_use_case.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"

namespace rover_crsf_teleop
{
namespace
{

constexpr int kLinearChannel = 3;
constexpr int kAngularChannel = 1;
constexpr int kEStopChannel = 5;
constexpr int kLatchResetChannel = 4;

// What the node hands over with nothing overridden: the declared defaults, read from parameters
// before any calibration is applied.
struct Inputs
{
    TeleopConfig config;
    TeleopIntegerParameters integers;
    ChannelCalibration calibration{defaultCalibration()};
};

TeleopConfigCheck validate(const Inputs & inputs)
{
    return validateTeleopConfig(inputs.config, inputs.integers, inputs.calibration);
}

// Channel N lives at index N-1, as everywhere else in the package.
int & at(std::array<int, RcFrame::kChannelCount> & values, const int channel_number)
{
    return values[static_cast<std::size_t>(channel_number - 1)];
}

// The four roles, in the order the rules check them.
const std::array<std::pair<const char *, int TeleopConfig::*>, 4> kRoles{{
    {"linear_x_channel", &TeleopConfig::linear_x_channel},
    {"angular_z_channel", &TeleopConfig::angular_z_channel},
    {"e_stop_channel", &TeleopConfig::e_stop_channel},
    {"e_stop_latch_reset_channel", &TeleopConfig::e_stop_latch_reset_channel}}};

const char kSettleOrTimeoutProblem[] =
    "switch_settle_frames must be >= 0 and channel_timeout_ms / link_stats_timeout_ms > 0.";

std::string linkQualityProblem(const std::int64_t lost, const std::int64_t recovered)
{
    return "Link quality thresholds must satisfy 0 <= link_quality_lost_below <= "
           "link_quality_recovered_at <= 100 (got " + std::to_string(lost) + " and " +
           std::to_string(recovered) + ").";
}

}  // namespace

TEST(TeleopConfigValidationTest, TheDefaultsAreValid)
{
    const TeleopConfigCheck check = validate(Inputs{});

    EXPECT_TRUE(check.problems.empty());
    EXPECT_TRUE(check.warnings.empty());
}

// The calibrated-range check on this parameter is only a warning, and it is skipped when there is
// no calibration - so without the hard bound a threshold off the wire domain configured silently
// and pinned both switch channels to one position for the life of the node.
TEST(TeleopConfigValidationTest, ASwitchThresholdOffTheWireIsRejected)
{
    for (const int threshold : {-1, 2048}) {
        Inputs inputs;
        inputs.config.channel_switch_threshold = threshold;

        const TeleopConfigCheck check = validate(inputs);

        ASSERT_EQ(check.problems.size(), 1u) << "threshold " << threshold;
        EXPECT_EQ(
            check.problems.front(),
            "Parameter channel_switch_threshold = " + std::to_string(threshold) +
            " is outside 0-2047.");
    }
}

TEST(TeleopConfigValidationTest, TheWireBoundsThemselvesAreAccepted)
{
    // Both sit outside the default switch range, so the range warning may fire; that is not a
    // configure error.
    for (const int threshold : {0, 2047}) {
        Inputs inputs;
        inputs.config.channel_switch_threshold = threshold;

        EXPECT_TRUE(validate(inputs).problems.empty()) << "threshold " << threshold;
    }
}

// Reading a bad channel number as 0 used to clamp the stick mapping to full negative deflection.
TEST(TeleopConfigValidationTest, EachChannelRoleMustBeOneToSixteen)
{
    for (const auto & [name, member] : kRoles) {
        for (const int channel : {0, 17}) {
            Inputs inputs;
            inputs.config.*member = channel;

            const TeleopConfigCheck check = validate(inputs);

            ASSERT_EQ(check.problems.size(), 1u) << name << " = " << channel;
            EXPECT_EQ(
                check.problems.front(),
                std::string("Parameter ") + name + " = " + std::to_string(channel) +
                " is outside 1-16.");
        }
    }
}

// The node only ever logs front(), but every failing role is collected, in role order.
TEST(TeleopConfigValidationTest, TheFirstInvalidRoleIsReportedFirst)
{
    Inputs inputs;
    inputs.config.linear_x_channel = 0;
    inputs.config.e_stop_channel = 17;

    EXPECT_EQ(
        validate(inputs).problems,
        (std::vector<std::string>{
            "Parameter linear_x_channel = 0 is outside 1-16.",
            "Parameter e_stop_channel = 17 is outside 1-16."}));
}

// Two roles on one channel would drive e.g. the E-Stop from a stick.
TEST(TeleopConfigValidationTest, TwoRolesOnOneChannelAreRejected)
{
    Inputs on_a_stick;
    on_a_stick.config.e_stop_channel = kLinearChannel;

    const TeleopConfigCheck stick = validate(on_a_stick);

    ASSERT_EQ(stick.problems.size(), 1u);
    EXPECT_EQ(
        stick.problems.front(),
        "Parameters linear_x_channel and e_stop_channel both use channel 3.");

    Inputs on_the_latch_reset;
    on_the_latch_reset.config.e_stop_channel = kLatchResetChannel;

    const TeleopConfigCheck latch = validate(on_the_latch_reset);

    ASSERT_EQ(latch.problems.size(), 1u);
    EXPECT_EQ(
        latch.problems.front(),
        "Parameters e_stop_channel and e_stop_latch_reset_channel both use channel 4.");
}

TEST(TeleopConfigValidationTest, EveryPairOfRolesIsChecked)
{
    for (std::size_t i = 0; i < kRoles.size(); ++i) {
        for (std::size_t j = i + 1; j < kRoles.size(); ++j) {
            Inputs inputs;
            const int shared = inputs.config.*kRoles[i].second;
            inputs.config.*kRoles[j].second = shared;

            const TeleopConfigCheck check = validate(inputs);

            ASSERT_EQ(check.problems.size(), 1u) << kRoles[i].first << " / " << kRoles[j].first;
            EXPECT_EQ(
                check.problems.front(),
                std::string("Parameters ") + kRoles[i].first + " and " + kRoles[j].first +
                " both use channel " + std::to_string(shared) + ".");
        }
    }
}

TEST(TeleopConfigValidationTest, AnUnusableStickCalibrationIsRejected)
{
    Inputs swallowed;
    at(swallowed.calibration.deadband, kLinearChannel) = 900;

    const TeleopConfigCheck wide = validate(swallowed);

    ASSERT_EQ(wide.problems.size(), 1u);
    EXPECT_EQ(
        wide.problems.front(),
        "Unusable stick calibration: Channel 3: the 900-count deadband swallows one side of the "
        "throw (172 / 992 / 1811), which would leave that direction dead.");

    Inputs negative;
    at(negative.calibration.deadband, kLinearChannel) = -1;

    const TeleopConfigCheck below_zero = validate(negative);

    ASSERT_EQ(below_zero.problems.size(), 1u);
    EXPECT_EQ(
        below_zero.problems.front(),
        "Unusable stick calibration: Channel 3: deadband -1 is negative.");
}

// A correctly measured switch rests at one end of its travel, so its centre sits on an endpoint -
// which every stick check would refuse.
TEST(TeleopConfigValidationTest, ASwitchChannelIsNotCheckedAsAStick)
{
    Inputs inputs;
    at(inputs.calibration.in_mid, kEStopChannel) = at(inputs.calibration.in_min, kEStopChannel);

    EXPECT_TRUE(validate(inputs).problems.empty());
}

TEST(TeleopConfigValidationTest, AThresholdOutsideASwitchsCalibratedRangeIsOnlyAWarning)
{
    const std::string e_stop_warning =
        "channel_switch_threshold 500 is outside channel 5's calibrated range 600-1811 "
        "(e_stop_channel), so that switch will always read the same position. Re-measure the "
        "channel or move the threshold.";
    const std::string latch_warning =
        "channel_switch_threshold 500 is outside channel 4's calibrated range 172-400 "
        "(e_stop_latch_reset_channel), so that switch will always read the same position. "
        "Re-measure the channel or move the threshold.";

    Inputs e_stop;
    at(e_stop.calibration.in_min, kEStopChannel) = 600;

    const TeleopConfigCheck e_stop_check = validate(e_stop);

    EXPECT_TRUE(e_stop_check.problems.empty());
    EXPECT_EQ(e_stop_check.warnings, std::vector<std::string>{e_stop_warning});

    Inputs latch;
    at(latch.calibration.in_max, kLatchResetChannel) = 400;

    const TeleopConfigCheck latch_check = validate(latch);

    EXPECT_TRUE(latch_check.problems.empty());
    EXPECT_EQ(latch_check.warnings, std::vector<std::string>{latch_warning});

    Inputs both;
    at(both.calibration.in_min, kEStopChannel) = 600;
    at(both.calibration.in_max, kLatchResetChannel) = 400;

    const TeleopConfigCheck both_check = validate(both);

    EXPECT_TRUE(both_check.problems.empty());
    EXPECT_EQ(both_check.warnings, (std::vector<std::string>{e_stop_warning, latch_warning}));
}

// A switch that rests exactly on the threshold never crosses it either.
TEST(TeleopConfigValidationTest, TheCalibratedRangeEndpointsCountAsOutside)
{
    Inputs at_low;
    at(at_low.calibration.in_min, kEStopChannel) = at_low.config.channel_switch_threshold;

    EXPECT_EQ(validate(at_low).warnings.size(), 1u);

    Inputs at_high;
    at(at_high.calibration.in_max, kEStopChannel) = at_high.config.channel_switch_threshold;

    EXPECT_EQ(validate(at_high).warnings.size(), 1u);
}

// No measured range, nothing to compare the threshold against.
TEST(TeleopConfigValidationTest, AnUncalibratedSwitchRangeIsNotWarnedAbout)
{
    Inputs inputs;
    at(inputs.calibration.in_min, kEStopChannel) = 900;
    at(inputs.calibration.in_max, kEStopChannel) = 900;

    const TeleopConfigCheck check = validate(inputs);

    EXPECT_TRUE(check.problems.empty());
    EXPECT_TRUE(check.warnings.empty());
}

// The warning indexes the calibration by the switch channels, so it waits for them to be valid,
// and a calibration that is itself unusable has no range worth comparing against.
TEST(TeleopConfigValidationTest, TheRangeWarningNeedsValidRolesAndCalibration)
{
    auto with_a_warning = []() {
        Inputs inputs;
        at(inputs.calibration.in_min, kEStopChannel) = 600;
        return inputs;
    };

    Inputs bad_threshold = with_a_warning();
    bad_threshold.config.channel_switch_threshold = 2048;

    Inputs bad_role = with_a_warning();
    bad_role.config.linear_x_channel = 0;

    Inputs bad_switch_role = with_a_warning();
    bad_switch_role.config.e_stop_channel = 17;

    Inputs shared_channel = with_a_warning();
    shared_channel.config.e_stop_latch_reset_channel = kAngularChannel;

    Inputs bad_calibration = with_a_warning();
    at(bad_calibration.calibration.deadband, kLinearChannel) = 900;

    for (const Inputs & inputs :
         {bad_threshold, bad_role, bad_switch_role, shared_channel, bad_calibration})
    {
        const TeleopConfigCheck check = validate(inputs);

        ASSERT_FALSE(check.problems.empty());
        EXPECT_TRUE(check.warnings.empty()) << check.problems.front();
    }
}

// The link-quality band and the rest are checked after the warning, so it is still reported.
TEST(TeleopConfigValidationTest, TheRangeWarningStillComesWithALaterProblem)
{
    Inputs inputs;
    at(inputs.calibration.in_min, kEStopChannel) = 600;
    inputs.integers.link_quality_lost_below = 60;
    inputs.integers.link_quality_recovered_at = 50;

    const TeleopConfigCheck check = validate(inputs);

    EXPECT_EQ(check.warnings.size(), 1u);
    ASSERT_EQ(check.problems.size(), 1u);
    EXPECT_EQ(check.problems.front(), linkQualityProblem(60, 50));
}

TEST(TeleopConfigValidationTest, ANegativeZeroBurstIsRejected)
{
    Inputs negative;
    negative.integers.zero_burst_duration_ms = -1;

    const TeleopConfigCheck check = validate(negative);

    ASSERT_EQ(check.problems.size(), 1u);
    EXPECT_EQ(check.problems.front(), "zero_burst_duration_ms must be >= 0.");

    Inputs single_zero;
    single_zero.integers.zero_burst_duration_ms = 0;

    EXPECT_TRUE(validate(single_zero).problems.empty());
}

TEST(TeleopConfigValidationTest, NonPositiveTimeoutsAndNegativeSettleFramesAreRejected)
{
    Inputs settle;
    settle.integers.switch_settle_frames = -1;

    Inputs channel_timeout;
    channel_timeout.integers.channel_timeout_ms = 0;

    Inputs link_stats_timeout;
    link_stats_timeout.integers.link_stats_timeout_ms = 0;

    for (const Inputs & inputs : {settle, channel_timeout, link_stats_timeout}) {
        const TeleopConfigCheck check = validate(inputs);

        ASSERT_EQ(check.problems.size(), 1u);
        EXPECT_EQ(check.problems.front(), kSettleOrTimeoutProblem);
    }

    Inputs smallest;
    smallest.integers.switch_settle_frames = 0;
    smallest.integers.channel_timeout_ms = 1;
    smallest.integers.link_stats_timeout_ms = 1;

    EXPECT_TRUE(validate(smallest).problems.empty());
}

TEST(TeleopConfigValidationTest, LinkQualityThresholdsMustBeOrderedWithinZeroToHundred)
{
    for (const auto & [lost, recovered] :
         {std::pair<std::int64_t, std::int64_t>{-1, 50}, {30, 101}, {60, 50}})
    {
        Inputs inputs;
        inputs.integers.link_quality_lost_below = lost;
        inputs.integers.link_quality_recovered_at = recovered;

        const TeleopConfigCheck check = validate(inputs);

        ASSERT_EQ(check.problems.size(), 1u) << lost << " / " << recovered;
        EXPECT_EQ(check.problems.front(), linkQualityProblem(lost, recovered));
    }

    for (const auto & [lost, recovered] :
         {std::pair<std::int64_t, std::int64_t>{0, 0}, {100, 100}, {0, 100}, {50, 50}})
    {
        Inputs inputs;
        inputs.integers.link_quality_lost_below = lost;
        inputs.integers.link_quality_recovered_at = recovered;

        EXPECT_TRUE(validate(inputs).problems.empty()) << lost << " / " << recovered;
    }
}

// TeleopConfig keeps these as uint8_t / unsigned: 300 would become 44 and -1 frames 4294967295,
// and both would pass if the checks ran on the narrowed values.
TEST(TeleopConfigValidationTest, ValuesAreCheckedBeforeNarrowing)
{
    Inputs link_quality;
    link_quality.integers.link_quality_lost_below = 30;
    link_quality.integers.link_quality_recovered_at = 300;

    const TeleopConfigCheck lq_check = validate(link_quality);

    ASSERT_EQ(lq_check.problems.size(), 1u);
    EXPECT_EQ(lq_check.problems.front(), linkQualityProblem(30, 300));

    Inputs settle;
    settle.integers.switch_settle_frames = -1;

    const TeleopConfigCheck settle_check = validate(settle);

    ASSERT_EQ(settle_check.problems.size(), 1u);
    EXPECT_EQ(settle_check.problems.front(), kSettleOrTimeoutProblem);
}

// The node logs front() and fails configure, so the order is the contract: it is the order the
// node checked in when each rule returned on its own.
TEST(TeleopConfigValidationTest, ProblemsAreReportedInTheNodesCheckOrder)
{
    Inputs inputs;
    inputs.config.channel_switch_threshold = -1;
    inputs.config.e_stop_channel = kLinearChannel;
    inputs.config.e_stop_latch_reset_channel = 0;
    at(inputs.calibration.deadband, kLinearChannel) = 900;
    inputs.integers.zero_burst_duration_ms = -1;
    inputs.integers.switch_settle_frames = -1;
    inputs.integers.link_quality_lost_below = 60;
    inputs.integers.link_quality_recovered_at = 50;

    const TeleopConfigCheck check = validate(inputs);

    EXPECT_EQ(
        check.problems,
        (std::vector<std::string>{
            "Parameter channel_switch_threshold = -1 is outside 0-2047.",
            "Parameter e_stop_latch_reset_channel = 0 is outside 1-16.",
            "Parameters linear_x_channel and e_stop_channel both use channel 3.",
            "Unusable stick calibration: Channel 3: the 900-count deadband swallows one side of "
            "the throw (172 / 992 / 1811), which would leave that direction dead.",
            "zero_burst_duration_ms must be >= 0.",
            kSettleOrTimeoutProblem,
            linkQualityProblem(60, 50)}));
    EXPECT_TRUE(check.warnings.empty());
}

TEST(AxisChannelsTest, FlagsOnlyTheTwoStickChannels)
{
    const std::array<bool, RcFrame::kChannelCount> axes = axisChannels(TeleopConfig{});

    for (int channel = 1; channel <= static_cast<int>(RcFrame::kChannelCount); ++channel) {
        const bool expected = channel == kLinearChannel || channel == kAngularChannel;
        EXPECT_EQ(axes[static_cast<std::size_t>(channel - 1)], expected) << "channel " << channel;
    }
}

TEST(AxisChannelsTest, SkipsAnInvalidChannelNumber)
{
    for (const int linear_channel : {0, 17}) {
        TeleopConfig config;
        config.linear_x_channel = linear_channel;

        const std::array<bool, RcFrame::kChannelCount> axes = axisChannels(config);

        for (int channel = 1; channel <= static_cast<int>(RcFrame::kChannelCount); ++channel) {
            EXPECT_EQ(axes[static_cast<std::size_t>(channel - 1)], channel == kAngularChannel)
                << "linear_x_channel " << linear_channel << ", channel " << channel;
        }
    }
}

}  // namespace rover_crsf_teleop
