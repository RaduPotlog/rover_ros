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

// Unit tests for the domain -> rover_msgs conversions used by the rc/channels and rc/link echoes.

#include <gtest/gtest.h>

#include <cstddef>

#include <rclcpp/time.hpp>

#include "rover_crsf_teleop/infrastructure/rc_message_conversions.hpp"

namespace rover_crsf_teleop
{
namespace
{

TEST(RcMessageConversionsTest, ChannelsAreCopiedInOrder)
{
    RcFrame frame;

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        frame.channels[i] = static_cast<int>(100U + i * 111U);
    }

    const auto message = toRcChannelsMsg(frame, rclcpp::Time(7, 250000000));

    ASSERT_EQ(message.channels.size(), RcFrame::kChannelCount);

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        EXPECT_EQ(message.channels[i], 100U + i * 111U) << "channel " << i;
    }

    EXPECT_EQ(message.header.stamp.sec, 7);
    EXPECT_EQ(message.header.stamp.nanosec, 250000000U);

    // RC input is not expressed in any TF frame.
    EXPECT_TRUE(message.header.frame_id.empty());
}

TEST(RcMessageConversionsTest, ChannelsAreClampedToElevenBits)
{
    // A CRSF frame can never produce these, but a hand-built RcFrame can - and a silent wrap of
    // the uint16 field would be worse than a clamp.
    RcFrame frame;
    frame.channels[0] = -5;
    frame.channels[1] = 99999;

    const auto message = toRcChannelsMsg(frame, rclcpp::Time(0, 0));

    EXPECT_EQ(message.channels[0], 0U);
    EXPECT_EQ(message.channels[1], 2047U);
}

TEST(RcMessageConversionsTest, LinkStatsRoundTripIncludingSignedSnr)
{
    RcLinkStats stats;
    stats.uplink_rssi_ant1 = 45;
    stats.uplink_rssi_ant2 = 50;
    stats.uplink_link_quality = 100;
    stats.uplink_snr = -7;
    stats.active_antenna = 1;
    stats.rf_mode = 2;
    stats.uplink_tx_power = 3;
    stats.downlink_rssi = 60;
    stats.downlink_link_quality = 99;
    stats.downlink_snr = -12;

    const auto message = toRcLinkStatusMsg(stats, rclcpp::Time(11, 0));

    EXPECT_EQ(message.uplink_rssi_ant1, 45);
    EXPECT_EQ(message.uplink_rssi_ant2, 50);
    EXPECT_EQ(message.uplink_link_quality, 100);
    // The field this test exists for: the message it replaced typed uplink_snr unsigned, so a
    // negative uplink SNR - a marginal link, the case you actually care about - was published
    // as 249 rather than -7.
    EXPECT_EQ(message.uplink_snr, -7);
    EXPECT_EQ(message.active_antenna, 1);
    EXPECT_EQ(message.rf_mode, 2);
    EXPECT_EQ(message.uplink_tx_power, 3);
    EXPECT_EQ(message.downlink_rssi, 60);
    EXPECT_EQ(message.downlink_link_quality, 99);
    EXPECT_EQ(message.downlink_snr, -12);
    EXPECT_EQ(message.header.stamp.sec, 11);
}

}  // namespace
}  // namespace rover_crsf_teleop
