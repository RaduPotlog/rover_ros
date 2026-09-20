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

#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/stick_mapping.hpp"

namespace rover_crsf_teleop
{
namespace
{

// The A1's real transmitter, as measured in config/rover_crsf_teleop.yaml: the linear stick (ch3)
// rests at 1004 and the angular one (ch1) at 987, 12 and 5 counts off the nominal 992 midpoint.
constexpr int kLinearChannel = 3;
constexpr int kAngularChannel = 1;
constexpr int kLinearRest = 1004;
constexpr int kAngularRest = 987;
constexpr int kSwitchChannel = 5;

RcFrame frameAt(const int value)
{
    RcFrame frame;
    frame.channels.fill(value);
    return frame;
}

void set(RcFrame & frame, const int channel_number, const int value)
{
    frame.channels[static_cast<std::size_t>(channel_number - 1)] = value;
}

int at(const std::array<int, RcFrame::kChannelCount> & values, const int channel_number)
{
    return values[static_cast<std::size_t>(channel_number - 1)];
}

std::array<bool, RcFrame::kChannelCount> axisMask(const std::initializer_list<int> channels)
{
    std::array<bool, RcFrame::kChannelCount> mask{};
    for (const int channel : channels) {
        mask[static_cast<std::size_t>(channel - 1)] = true;
    }
    return mask;
}

// Feeds `count` frames of the sticks sitting still, alternating +/- `jitter` around their rest
// value so the recorded spread is exactly 2 * jitter.
void feedRest(RcCalibrator & calibrator, const unsigned int count, const int jitter)
{
    for (unsigned int i = 0; i < count; ++i) {
        const int offset = (i % 2 == 0) ? jitter : -jitter;
        RcFrame frame = frameAt(kDefaultCrsfChannelMid);
        set(frame, kLinearChannel, kLinearRest + offset);
        set(frame, kAngularChannel, kAngularRest + offset);
        set(frame, kSwitchChannel, kDefaultCrsfChannelMax);
        calibrator.onFrame(frame);
    }
}

// Walks a channel to both extremes and back to its rest value, leaving every other channel where
// feedRest() left it - otherwise the frame itself moves channels the test never touched.
void sweepChannel(
    RcCalibrator & calibrator, const int channel_number, const int low, const int high,
    const int rest)
{
    for (const int value : {low, high, rest}) {
        RcFrame frame = frameAt(kDefaultCrsfChannelMid);
        set(frame, kLinearChannel, kLinearRest);
        set(frame, kAngularChannel, kAngularRest);
        set(frame, kSwitchChannel, kDefaultCrsfChannelMax);
        set(frame, channel_number, value);
        calibrator.onFrame(frame);
    }
}

TEST(RcCalibrationTest, DefaultCalibrationIsTheNominalCrsfRangeOnEveryChannel)
{
    const ChannelCalibration calibration = defaultCalibration();

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        EXPECT_EQ(calibration.in_min[i], kDefaultCrsfChannelMin);
        EXPECT_EQ(calibration.in_mid[i], kDefaultCrsfChannelMid);
        EXPECT_EQ(calibration.in_max[i], kDefaultCrsfChannelMax);
        EXPECT_EQ(calibration.deadband[i], kDefaultChannelDeadband);
    }
}

TEST(RcCalibrationTest, AxisMappingPicksOutOneChannel)
{
    ChannelCalibration calibration = defaultCalibration();
    calibration.in_mid[2] = kLinearRest;
    calibration.deadband[2] = 11;

    const AxisMapping mapping = axisMapping(calibration, kLinearChannel);
    EXPECT_EQ(mapping.in_mid, kLinearRest);
    EXPECT_EQ(mapping.deadband_counts, 11);

    // An out-of-range channel falls back to the nominal endpoints rather than reading past the
    // array; the node rejects such a channel number at configure time.
    const AxisMapping fallback = axisMapping(calibration, 0);
    EXPECT_EQ(fallback.in_mid, kDefaultCrsfChannelMid);
}

TEST(RcCalibrationTest, CentreIsTheMeanOfTheRestSamplesAndTheDeadbandCoversTheirSpread)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());

    constexpr int kJitter = 6;
    feedRest(calibrator, kCenterSampleTarget, kJitter);

    // Equal numbers of +jitter and -jitter samples, so the mean is exactly the rest value - the
    // measured offset from the nominal 992 is recovered, which is the whole point.
    EXPECT_EQ(at(calibrator.measured().in_mid, kLinearChannel), kLinearRest);
    EXPECT_EQ(at(calibrator.measured().in_mid, kAngularChannel), kAngularRest);

    // Spread is 2 * jitter, so the deadband is half of that plus the margin.
    EXPECT_EQ(at(calibrator.measured().deadband, kLinearChannel), kJitter + kDeadbandMargin);
}

TEST(RcCalibrationTest, ADeadStillChannelStillGetsTheMinimumDeadband)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());

    feedRest(calibrator, kCenterSampleTarget, 0);

    // Zero measured spread would otherwise give a zero deadband, and the first count of drift
    // would become a permanent creep command.
    EXPECT_EQ(at(calibrator.measured().deadband, kLinearChannel), kMinDeadband);
}

TEST(RcCalibrationTest, TheSweepHoldsTheExtremesEvenFromASingleFrame)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());
    feedRest(calibrator, kCenterSampleTarget, 4);
    ASSERT_TRUE(calibrator.beginSweep());

    // One frame at each extreme, then back to centre - the peak hold is exactly what a browser
    // sampling a best-effort topic would miss.
    sweepChannel(calibrator, kLinearChannel, 180, 1800, kLinearRest);

    EXPECT_EQ(at(calibrator.measured().in_min, kLinearChannel), 180);
    EXPECT_EQ(at(calibrator.measured().in_max, kLinearChannel), 1800);
    EXPECT_TRUE(calibrator.channelsMoved()[kLinearChannel - 1]);
}

TEST(RcCalibrationTest, AChannelThatNeverMovedKeepsItsPreviousCalibration)
{
    ChannelCalibration previous = defaultCalibration();
    previous.in_min[kAngularChannel - 1] = 200;
    previous.in_mid[kAngularChannel - 1] = 1000;
    previous.in_max[kAngularChannel - 1] = 1790;
    previous.deadband[kAngularChannel - 1] = 17;

    RcCalibrator calibrator;
    calibrator.start(previous);
    feedRest(calibrator, kCenterSampleTarget, 4);
    ASSERT_TRUE(calibrator.beginSweep());

    // Only the linear stick is swept.
    sweepChannel(calibrator, kLinearChannel, 180, 1800, kLinearRest);
    ASSERT_TRUE(calibrator.finish());

    EXPECT_FALSE(calibrator.channelsMoved()[kAngularChannel - 1]);
    EXPECT_EQ(at(calibrator.measured().in_min, kAngularChannel), 200);
    EXPECT_EQ(at(calibrator.measured().in_mid, kAngularChannel), 1000);
    EXPECT_EQ(at(calibrator.measured().in_max, kAngularChannel), 1790);
    EXPECT_EQ(at(calibrator.measured().deadband, kAngularChannel), 17);
}

TEST(RcCalibrationTest, TheSweepMustSeeRealTravelBeforeAChannelCounts)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());
    feedRest(calibrator, kCenterSampleTarget, 4);
    ASSERT_TRUE(calibrator.beginSweep());

    // Less than kMinTravel: an unassigned channel drifting, not a stick being moved.
    sweepChannel(calibrator, kLinearChannel, kLinearRest - 20, kLinearRest + 20, kLinearRest);

    EXPECT_FALSE(calibrator.channelsMoved()[kLinearChannel - 1]);
    EXPECT_FALSE(calibrator.finish()) << "a sweep where nothing moved is not a calibration";
}

TEST(RcCalibrationTest, PhaseTransitionsAreOrdered)
{
    RcCalibrator calibrator;
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kIdle);
    EXPECT_FALSE(calibrator.beginSweep());
    EXPECT_FALSE(calibrator.finish());

    calibrator.start(defaultCalibration());
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kCenter);

    // The centre is what everything downstream is defined about, so a handful of frames is not
    // enough to leave it.
    feedRest(calibrator, kMinCenterSamples - 1, 4);
    EXPECT_FALSE(calibrator.beginSweep());
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kCenter);

    feedRest(calibrator, 1, 4);
    EXPECT_TRUE(calibrator.beginSweep());
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kSweep);

    sweepChannel(calibrator, kLinearChannel, 180, 1800, kLinearRest);
    EXPECT_TRUE(calibrator.finish());
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kReview);

    calibrator.cancel();
    EXPECT_EQ(calibrator.phase(), CalibrationPhase::kIdle);
}

TEST(RcCalibrationTest, ProgressTracksTheCentreSampleTargetOnly)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());
    EXPECT_DOUBLE_EQ(calibrator.progress(), 0.0);

    feedRest(calibrator, kCenterSampleTarget / 2, 4);
    EXPECT_NEAR(calibrator.progress(), 0.5, 1e-9);

    feedRest(calibrator, kCenterSampleTarget, 4);
    EXPECT_DOUBLE_EQ(calibrator.progress(), 1.0) << "progress is clamped once the target is met";

    ASSERT_TRUE(calibrator.beginSweep());
    EXPECT_DOUBLE_EQ(calibrator.progress(), 0.0) << "the sweep ends when the operator says so";
}

TEST(RcCalibrationTest, ASwitchRestingAtAnEndpointIsNotAProblem)
{
    ChannelCalibration calibration = defaultCalibration();
    // What a correctly measured two-position switch looks like: it rests at one end of its
    // travel, so its centre IS its maximum.
    calibration.in_min[kSwitchChannel - 1] = 172;
    calibration.in_mid[kSwitchChannel - 1] = 1811;
    calibration.in_max[kSwitchChannel - 1] = 1811;

    EXPECT_TRUE(calibrationProblems(calibration, axisMask({kLinearChannel, kAngularChannel})).empty())
        << "switches are not proportional axes and must not be checked as if they were";

    EXPECT_FALSE(calibrationProblems(calibration, axisMask({kSwitchChannel})).empty())
        << "checked as an axis, the same switch is degenerate";
}

TEST(RcCalibrationTest, ADeadbandThatSwallowsAHalfThrowIsRejected)
{
    ChannelCalibration calibration = defaultCalibration();
    calibration.in_min[kLinearChannel - 1] = 900;
    calibration.in_mid[kLinearChannel - 1] = 1004;
    calibration.in_max[kLinearChannel - 1] = 1800;
    calibration.deadband[kLinearChannel - 1] = 200;

    const auto problems = calibrationProblems(calibration, axisMask({kLinearChannel}));
    ASSERT_EQ(problems.size(), 1u);

    // mapAxis() returns 0.0 for a non-positive half-span, so without this check the axis would be
    // silently dead in one direction with no error anywhere.
    const AxisMapping mapping = axisMapping(calibration, kLinearChannel);
    EXPECT_DOUBLE_EQ(mapAxis(900, mapping), 0.0);
}

TEST(RcCalibrationTest, ACentreOutsideTheMeasuredRangeIsRejected)
{
    ChannelCalibration calibration = defaultCalibration();
    calibration.in_min[kLinearChannel - 1] = 1100;
    calibration.in_mid[kLinearChannel - 1] = 1004;
    calibration.in_max[kLinearChannel - 1] = 1800;

    EXPECT_EQ(calibrationProblems(calibration, axisMask({kLinearChannel})).size(), 1u);
}

TEST(RcCalibrationTest, AMeasuredCalibrationMapsItsOwnCentreToExactlyZero)
{
    RcCalibrator calibrator;
    calibrator.start(defaultCalibration());
    feedRest(calibrator, kCenterSampleTarget, 6);
    ASSERT_TRUE(calibrator.beginSweep());
    sweepChannel(calibrator, kLinearChannel, 180, 1800, kLinearRest);
    ASSERT_TRUE(calibrator.finish());

    AxisMapping mapping = axisMapping(calibrator.measured(), kLinearChannel);
    mapping.out_min = -1.2;
    mapping.out_max = 1.2;

    // The bug the global midpoint caused: a stick resting 12 counts off 992 commanded a permanent
    // creep, which made the hardware interface's E-Stop reset invariant unsatisfiable.
    EXPECT_DOUBLE_EQ(mapAxis(kLinearRest, mapping), 0.0);
    EXPECT_DOUBLE_EQ(mapAxis(1800, mapping), 1.2);
    EXPECT_DOUBLE_EQ(mapAxis(180, mapping), -1.2);
}

}  // namespace
}  // namespace rover_crsf_teleop
