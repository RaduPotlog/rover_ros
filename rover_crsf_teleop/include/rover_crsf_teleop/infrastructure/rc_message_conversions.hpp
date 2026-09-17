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

#ifndef ROVER_CRSF_TELEOP_INFRASTRUCTURE_RC_MESSAGE_CONVERSIONS_HPP_
#define ROVER_CRSF_TELEOP_INFRASTRUCTURE_RC_MESSAGE_CONVERSIONS_HPP_

#include <rclcpp/time.hpp>

#include "rover_msgs/msg/rc_channels.hpp"
#include "rover_msgs/msg/rc_link_status.hpp"

#include "rover_crsf_teleop/domain/rc_frame.hpp"

namespace rover_crsf_teleop
{

// Domain -> ROS, at the publishing boundary only. These exist so decoded RC input can be echoed
// on rc/channels and rc/link for tuning and for bags; nothing in the teleop rules reads them,
// which is why the domain types stay free of rover_msgs.

rover_msgs::msg::RcChannels toRcChannelsMsg(const RcFrame & frame, const rclcpp::Time & stamp);

rover_msgs::msg::RcLinkStatus toRcLinkStatusMsg(const RcLinkStats & stats, const rclcpp::Time & stamp);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_INFRASTRUCTURE_RC_MESSAGE_CONVERSIONS_HPP_
