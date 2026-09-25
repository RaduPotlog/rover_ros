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

#ifndef ROVER_CRSF_TELEOP_APPLICATION_TELEOP_CONFIG_VALIDATION_HPP_
#define ROVER_CRSF_TELEOP_APPLICATION_TELEOP_CONFIG_VALIDATION_HPP_

#include <cstdint>
#include <string>
#include <vector>

#include "rover_crsf_teleop/application/teleop_use_case.hpp"
#include "rover_crsf_teleop/domain/link_monitor.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/switch_debouncer.hpp"

namespace rover_crsf_teleop
{

// The six integer parameters TeleopConfig stores narrower than ROS reads them, kept at the
// int64_t they are read as so the checks see the configured value (-1 frames is refused, not
// wrapped to 4294967295). Defaults = the node's declared defaults.
//
// LinkMonitorConfig and kDefaultSwitchSettleFrames are named directly by the defaults below (not
// just transitively through TeleopConfig), so their headers are included explicitly rather than
// relied on by accident.
struct TeleopIntegerParameters
{
    std::int64_t switch_settle_frames{kDefaultSwitchSettleFrames};
    std::int64_t channel_timeout_ms{LinkMonitorConfig{}.channel_timeout.count()};
    std::int64_t link_stats_timeout_ms{LinkMonitorConfig{}.link_stats_timeout.count()};
    std::int64_t link_quality_lost_below{LinkMonitorConfig{}.lq_lost_below};
    std::int64_t link_quality_recovered_at{LinkMonitorConfig{}.lq_recovered_at};
    std::int64_t zero_burst_duration_ms{0};
};

struct TeleopConfigCheck
{
    // Configure-fatal, in the node's historical check order; front() is what gets logged.
    std::vector<std::string> problems;
    // Non-fatal; logged, and configure proceeds.
    std::vector<std::string> warnings;
};

// Every rule that decides whether the parameters make a usable TeleopConfig, one human-readable
// sentence per violation. Empty `problems` means configure may proceed.
//
// `config` is as read from the parameters (before applyCalibration()), `calibration` is read
// alongside it, and `integers` carries the values TeleopConfig would narrow. Problems are
// collected in a fixed rule order - the switch threshold's wire range, channel numbers 1-16, no
// two roles on one channel, the stick calibration, the zero burst, the settle frames and
// timeouts, the link-quality band - so front() is always the first rule that failed. The
// threshold-outside-a-switch's-calibrated-range warning is only computed once the channels and
// the calibration have passed, since it indexes the calibration by those channels.
TeleopConfigCheck validateTeleopConfig(
    const TeleopConfig & config,
    const TeleopIntegerParameters & integers,
    const ChannelCalibration & calibration);

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_APPLICATION_TELEOP_CONFIG_VALIDATION_HPP_
