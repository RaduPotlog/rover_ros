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

#include "rover_crsf_teleop/infrastructure/rc_message_conversions.hpp"

#include <algorithm>
#include <cstdint>

namespace rover_crsf_teleop
{

rover_msgs::msg::RcChannels toRcChannelsMsg(const RcFrame & frame, const rclcpp::Time & stamp)
{
    rover_msgs::msg::RcChannels message;
    message.header.stamp = stamp;

    // frame_id stays empty: RC input is not expressed in any TF frame.
    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        // Channel values are 11-bit unsigned by construction; clamp anyway so a hand-built
        // RcFrame in a test or a future non-CRSF source cannot wrap the uint16 field silently.
        const int value = std::clamp(frame.channels[i], 0, 2047);
        message.channels[i] = static_cast<std::uint16_t>(value);
    }

    return message;
}

rover_msgs::msg::RcLinkStatus toRcLinkStatusMsg(const RcLinkStats & stats, const rclcpp::Time & stamp)
{
    rover_msgs::msg::RcLinkStatus message;
    message.header.stamp = stamp;

    message.uplink_rssi_ant1 = stats.uplink_rssi_ant1;
    message.uplink_rssi_ant2 = stats.uplink_rssi_ant2;
    message.uplink_link_quality = stats.uplink_link_quality;
    message.uplink_snr = stats.uplink_snr;
    message.active_antenna = stats.active_antenna;
    message.rf_mode = stats.rf_mode;
    message.uplink_tx_power = stats.uplink_tx_power;
    message.downlink_rssi = stats.downlink_rssi;
    message.downlink_link_quality = stats.downlink_link_quality;
    message.downlink_snr = stats.downlink_snr;

    return message;
}

namespace
{

std::array<std::uint16_t, RcFrame::kChannelCount> toUnsigned(
    const std::array<int, RcFrame::kChannelCount> & values)
{
    std::array<std::uint16_t, RcFrame::kChannelCount> out{};

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        out[i] = static_cast<std::uint16_t>(std::clamp(values[i], 0, 2047));
    }

    return out;
}

std::uint8_t toPhaseField(const CalibrationPhase phase)
{
    switch (phase) {
        case CalibrationPhase::kCenter:
            return rover_msgs::msg::RcCalibrationState::PHASE_CENTER;
        case CalibrationPhase::kSweep:
            return rover_msgs::msg::RcCalibrationState::PHASE_SWEEP;
        case CalibrationPhase::kReview:
            return rover_msgs::msg::RcCalibrationState::PHASE_REVIEW;
        case CalibrationPhase::kIdle:
            break;
    }

    return rover_msgs::msg::RcCalibrationState::PHASE_IDLE;
}

}  // namespace

rover_msgs::msg::RcCalibration toRcCalibrationMsg(const ChannelCalibration & calibration)
{
    rover_msgs::msg::RcCalibration message;
    message.channel_min = toUnsigned(calibration.in_min);
    message.channel_mid = toUnsigned(calibration.in_mid);
    message.channel_max = toUnsigned(calibration.in_max);
    message.channel_deadband = toUnsigned(calibration.deadband);
    return message;
}

rover_msgs::msg::RcCalibrationState toRcCalibrationStateMsg(
    const CalibrationSnapshot & snapshot, const rclcpp::Time & stamp)
{
    rover_msgs::msg::RcCalibrationState message;
    message.header.stamp = stamp;

    message.phase = toPhaseField(snapshot.phase);
    message.samples = snapshot.samples;
    message.progress = static_cast<float>(snapshot.progress);
    message.teleop_inhibited = snapshot.teleop_inhibited;
    message.active = toRcCalibrationMsg(snapshot.active);
    message.measured = toRcCalibrationMsg(snapshot.measured);
    message.channel_moved = snapshot.channel_moved;
    message.problems = snapshot.problems;
    message.message = snapshot.message;

    return message;
}

ChannelCalibration fromRcCalibrationMsg(const rover_msgs::msg::RcCalibration & message)
{
    ChannelCalibration calibration;

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        calibration.in_min[i] = static_cast<int>(message.channel_min[i]);
        calibration.in_mid[i] = static_cast<int>(message.channel_mid[i]);
        calibration.in_max[i] = static_cast<int>(message.channel_max[i]);
        calibration.deadband[i] = static_cast<int>(message.channel_deadband[i]);
    }

    return calibration;
}

std::vector<int64_t> toParameterArray(const std::array<int, RcFrame::kChannelCount> & values)
{
    return std::vector<int64_t>(values.cbegin(), values.cend());
}

}  // namespace rover_crsf_teleop
