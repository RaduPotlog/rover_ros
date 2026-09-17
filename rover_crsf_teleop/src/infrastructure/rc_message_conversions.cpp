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

}  // namespace rover_crsf_teleop
