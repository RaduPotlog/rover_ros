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

#include "rover_crsf_teleop/application/teleop_config_validation.hpp"

#include <array>
#include <cstddef>
#include <string>
#include <utility>
#include <vector>

#include "rover_crsf_teleop/domain/rc_frame.hpp"

namespace rover_crsf_teleop
{

namespace
{

// The top of the 11-bit CRSF wire domain (crsf_protocol.hpp).
constexpr int kMaxWireValue = 2047;

// The four channel roles, in the order the rules check and report them.
std::array<std::pair<const char *, int>, 4> channelRoles(const TeleopConfig & config)
{
    return {{
        {"linear_x_channel", config.linear_x_channel},
        {"angular_z_channel", config.angular_z_channel},
        {"e_stop_channel", config.e_stop_channel},
        {"e_stop_latch_reset_channel", config.e_stop_latch_reset_channel}}};
}

}  // namespace

TeleopConfigCheck validateTeleopConfig(
    const TeleopConfig & config,
    const TeleopIntegerParameters & integers,
    const ChannelCalibration & calibration)
{
    TeleopConfigCheck check;
    std::vector<std::string> & problems = check.problems;
    const int threshold = config.channel_switch_threshold;

    // Same 0-2047 wire domain the endpoint arrays are held to when the node reads them. The
    // calibrated-range check further down is only a warning and is gated on having a calibration
    // at all, so without this a negative or out-of-wire-range threshold configured silently and
    // pinned both switches to one position for the life of the node.
    if (threshold < 0 || threshold > kMaxWireValue) {
        problems.push_back(
            "Parameter channel_switch_threshold = " + std::to_string(threshold) + " is outside 0-" +
            std::to_string(kMaxWireValue) + ".");
    }

    // Fail configure on a bad channel number instead of the previous behaviour of reading it as
    // 0 - which the stick mapping clamps to full negative deflection.
    const auto roles = channelRoles(config);
    for (const auto & [name, channel] : roles) {
        if (!RcFrame::isValidChannel(channel)) {
            problems.push_back(
                std::string("Parameter ") + name + " = " + std::to_string(channel) +
                " is outside 1-" + std::to_string(RcFrame::kChannelCount) + ".");
        }
    }

    // Two roles on one channel would drive e.g. the E-Stop from a stick, so fail configure.
    for (std::size_t i = 0; i < roles.size(); ++i) {
        for (std::size_t j = i + 1; j < roles.size(); ++j) {
            if (roles[i].second == roles[j].second) {
                problems.push_back(
                    std::string("Parameters ") + roles[i].first + " and " + roles[j].first +
                    " both use channel " + std::to_string(roles[i].second) + ".");
            }
        }
    }

    for (const std::string & problem : calibrationProblems(calibration, axisChannels(config))) {
        problems.push_back("Unusable stick calibration: " + problem);
    }

    // channel_switch_threshold stays an absolute raw value: a switch sits at the ends of its
    // travel, so comparing raw counts is right, and re-deriving a safety-critical threshold from
    // a measurement an operator just took is a worse failure mode than leaving it explicit. What
    // the calibration does buy is being able to notice when the threshold has fallen outside a
    // switch's actual range - which would leave that switch stuck reading one position forever.
    //
    // Only with every check above passed: the channels index the calibration, so they have to be
    // valid first, and a calibration that is itself unusable has no range worth comparing against.
    if (problems.empty()) {
        // The two switch roles: e_stop_channel, then e_stop_latch_reset_channel.
        for (const auto & [name, channel] : {roles[2], roles[3]}) {
            const std::size_t index = static_cast<std::size_t>(channel - 1);
            const int low = calibration.in_min[index];
            const int high = calibration.in_max[index];

            if (high > low && (threshold <= low || threshold >= high)) {
                check.warnings.push_back(
                    "channel_switch_threshold " + std::to_string(threshold) +
                    " is outside channel " + std::to_string(channel) + "'s calibrated range " +
                    std::to_string(low) + "-" + std::to_string(high) + " (" + name +
                    "), so that switch will always read the same position. Re-measure the "
                    "channel or move the threshold.");
            }
        }
    }

    if (integers.zero_burst_duration_ms < 0) {
        problems.push_back("zero_burst_duration_ms must be >= 0.");
    }

    if (integers.switch_settle_frames < 0 || integers.channel_timeout_ms <= 0 ||
        integers.link_stats_timeout_ms <= 0)
    {
        problems.push_back(
            "switch_settle_frames must be >= 0 and channel_timeout_ms / link_stats_timeout_ms "
            "> 0.");
    }

    const std::int64_t lost = integers.link_quality_lost_below;
    const std::int64_t recovered = integers.link_quality_recovered_at;

    if (lost < 0 || recovered > 100 || recovered < lost) {
        problems.push_back(
            "Link quality thresholds must satisfy 0 <= link_quality_lost_below <= "
            "link_quality_recovered_at <= 100 (got " + std::to_string(lost) + " and " +
            std::to_string(recovered) + ").");
    }

    return check;
}

}  // namespace rover_crsf_teleop
