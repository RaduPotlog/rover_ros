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

#include "rover_crsf_teleop/domain/rc_calibration.hpp"

#include <algorithm>
#include <limits>

namespace rover_crsf_teleop
{

namespace
{

std::string channelLabel(const std::size_t index)
{
    return "Channel " + std::to_string(index + 1);
}

}  // namespace

ChannelCalibration defaultCalibration()
{
    ChannelCalibration calibration;
    calibration.in_min.fill(kDefaultCrsfChannelMin);
    calibration.in_mid.fill(kDefaultCrsfChannelMid);
    calibration.in_max.fill(kDefaultCrsfChannelMax);
    calibration.deadband.fill(kDefaultChannelDeadband);
    return calibration;
}

AxisMapping axisMapping(const ChannelCalibration & calibration, const int channel_number)
{
    AxisMapping mapping;

    if (channel_number < 1 ||
        static_cast<std::size_t>(channel_number) > RcFrame::kChannelCount)
    {
        return mapping;
    }

    const std::size_t index = static_cast<std::size_t>(channel_number - 1);
    mapping.in_min = calibration.in_min[index];
    mapping.in_mid = calibration.in_mid[index];
    mapping.in_max = calibration.in_max[index];
    mapping.deadband_counts = calibration.deadband[index];
    return mapping;
}

std::vector<std::string> calibrationProblems(
    const ChannelCalibration & calibration,
    const std::array<bool, RcFrame::kChannelCount> & axis_channels)
{
    std::vector<std::string> problems;

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        if (!axis_channels[i]) {
            continue;
        }

        const int min = calibration.in_min[i];
        const int mid = calibration.in_mid[i];
        const int max = calibration.in_max[i];
        const int deadband = calibration.deadband[i];

        if (deadband < 0) {
            problems.push_back(
                channelLabel(i) + ": deadband " + std::to_string(deadband) + " is negative.");
            continue;
        }

        if (mid < min || mid > max) {
            problems.push_back(
                channelLabel(i) + ": centre " + std::to_string(mid) + " is outside the measured "
                "range " + std::to_string(min) + "-" + std::to_string(max) +
                ". Was the stick held while the centre was sampled?");
            continue;
        }

        // mapAxis() returns 0.0 when either half-span is non-positive, so this is an axis that
        // would be silently dead in one direction.
        if (max - mid - deadband <= 0 || mid - min - deadband <= 0) {
            problems.push_back(
                channelLabel(i) + ": the " + std::to_string(deadband) +
                "-count deadband swallows one side of the throw (" + std::to_string(min) + " / " +
                std::to_string(mid) + " / " + std::to_string(max) +
                "), which would leave that direction dead.");
        }
    }

    return problems;
}

void RcCalibrator::start(const ChannelCalibration & previous)
{
    phase_ = CalibrationPhase::kCenter;
    samples_ = 0;
    previous_ = previous;
    measured_ = previous;
    moved_.fill(false);

    center_sum_.fill(0);
    center_low_.fill(std::numeric_limits<int>::max());
    center_high_.fill(std::numeric_limits<int>::min());
    sweep_low_.fill(std::numeric_limits<int>::max());
    sweep_high_.fill(std::numeric_limits<int>::min());
}

void RcCalibrator::onFrame(const RcFrame & frame)
{
    if (phase_ == CalibrationPhase::kCenter) {
        for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
            const int value = frame.channels[i];
            center_sum_[i] += value;
            center_low_[i] = std::min(center_low_[i], value);
            center_high_[i] = std::max(center_high_[i], value);
        }

        ++samples_;
        computeCenter();
        return;
    }

    if (phase_ == CalibrationPhase::kSweep) {
        for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
            const int value = frame.channels[i];
            sweep_low_[i] = std::min(sweep_low_[i], value);
            sweep_high_[i] = std::max(sweep_high_[i], value);

            // Live, so the UI can show the range filling in while the operator sweeps.
            measured_.in_min[i] = sweep_low_[i];
            measured_.in_max[i] = sweep_high_[i];
            moved_[i] = (sweep_high_[i] - sweep_low_[i]) >= kMinTravel;
        }

        ++samples_;
    }
}

void RcCalibrator::computeCenter()
{
    if (samples_ == 0) {
        return;
    }

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        // Rounded, not truncated: a centre one count low is a permanent bias on that axis.
        const long total = center_sum_[i];
        const long count = static_cast<long>(samples_);
        measured_.in_mid[i] = static_cast<int>((total + count / 2) / count);

        // Half the peak-to-peak rest spread, plus a margin, floored. This is the hand procedure
        // config/rover_crsf_teleop.yaml documents ("size it against where the sticks actually
        // REST, not against frame noise"), done from the frames themselves.
        const int spread = center_high_[i] - center_low_[i];
        measured_.deadband[i] = std::max(kMinDeadband, (spread + 1) / 2 + kDeadbandMargin);
    }
}

bool RcCalibrator::beginSweep()
{
    if (phase_ != CalibrationPhase::kCenter || samples_ < kMinCenterSamples) {
        return false;
    }

    // Seed the sweep from where the sticks rest, so a channel that is never touched still spans a
    // self-consistent (if unusable) range instead of an inverted one.
    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        sweep_low_[i] = center_low_[i];
        sweep_high_[i] = center_high_[i];
        measured_.in_min[i] = sweep_low_[i];
        measured_.in_max[i] = sweep_high_[i];
    }

    phase_ = CalibrationPhase::kSweep;
    samples_ = 0;
    return true;
}

bool RcCalibrator::finish()
{
    if (phase_ != CalibrationPhase::kSweep) {
        return false;
    }

    const bool any_moved = std::any_of(moved_.cbegin(), moved_.cend(), [](const bool m) { return m; });
    if (!any_moved) {
        return false;
    }

    // A channel the sweep never saw move keeps whatever it had: overwriting an unassigned
    // channel - or one the operator forgot - with a range a few counts wide would produce a
    // mapping that saturates on noise.
    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        if (!moved_[i]) {
            measured_.in_min[i] = previous_.in_min[i];
            measured_.in_mid[i] = previous_.in_mid[i];
            measured_.in_max[i] = previous_.in_max[i];
            measured_.deadband[i] = previous_.deadband[i];
        }
    }

    phase_ = CalibrationPhase::kReview;
    return true;
}

void RcCalibrator::cancel()
{
    phase_ = CalibrationPhase::kIdle;
    samples_ = 0;
}

double RcCalibrator::progress() const
{
    if (phase_ != CalibrationPhase::kCenter) {
        return 0.0;
    }

    return std::min(1.0, static_cast<double>(samples_) / static_cast<double>(kCenterSampleTarget));
}

}  // namespace rover_crsf_teleop
