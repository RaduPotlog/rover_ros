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

#include <array>
#include <cstdint>
#include <vector>

#include <rclcpp/time.hpp>

#include "rover_msgs/msg/safety_status.hpp"
#include "rover_msgs/msg/rc_calibration.hpp"
#include "rover_msgs/msg/rc_calibration_state.hpp"
#include "rover_msgs/msg/rc_channels.hpp"
#include "rover_msgs/msg/rc_link_status.hpp"

#include "rover_crsf_teleop/application/calibration_use_case.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"
#include "rover_crsf_teleop/domain/safety_io_flags.hpp"

namespace rover_crsf_teleop
{

// Domain -> ROS, at the publishing boundary only. These exist so decoded RC input can be echoed
// on rc/channels and rc/link for tuning and for bags; nothing in the teleop rules reads them,
// which is why the domain types stay free of rover_msgs.

rover_msgs::msg::RcChannels toRcChannelsMsg(const RcFrame & frame, const rclcpp::Time & stamp);

rover_msgs::msg::RcLinkStatus toRcLinkStatusMsg(const RcLinkStats & stats, const rclcpp::Time & stamp);

rover_msgs::msg::RcCalibration toRcCalibrationMsg(const ChannelCalibration & calibration);

rover_msgs::msg::RcCalibrationState toRcCalibrationStateMsg(
    const CalibrationSnapshot & snapshot, const rclcpp::Time & stamp);

// ROS -> domain, for the apply service. Out-of-range values are clamped rather than rejected
// here; whether the result is usable is the domain's judgement (calibrationProblems), not the
// message layer's.
ChannelCalibration fromRcCalibrationMsg(const rover_msgs::msg::RcCalibration & message);

// The per-channel array as a ROS integer-array parameter value.
std::vector<int64_t> toParameterArray(const std::array<int, RcFrame::kChannelCount> & values);

// The motion-inhibiting subset of SafetyStatus, as plain bools.
//
// Field for field, no inversion: every *_e_stop_* pin is true when that stop is active. Two
// fields are deliberately dropped - see domain/safety_io_flags.hpp for why including either the
// watchdog heartbeat or the latch-reset pulse would be a bug.
SafetyIoFlags toSafetyIoFlags(const rover_msgs::msg::SafetyStatus & message);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_INFRASTRUCTURE_RC_MESSAGE_CONVERSIONS_HPP_
