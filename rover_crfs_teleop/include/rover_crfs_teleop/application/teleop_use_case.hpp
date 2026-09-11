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

#ifndef ROVER_CRFS_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_
#define ROVER_CRFS_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_

#include <cstdint>
#include <memory>
#include <optional>

#include "rover_crfs_teleop/domain/link_monitor.hpp"
#include "rover_crfs_teleop/domain/ports.hpp"
#include "rover_crfs_teleop/domain/rc_frame.hpp"
#include "rover_crfs_teleop/domain/stick_mapping.hpp"
#include "rover_crfs_teleop/domain/switch_debouncer.hpp"

namespace rover_crfs_teleop
{

struct TeleopConfig
{
    AxisMapping linear_x_mapping;
    AxisMapping angular_z_mapping;

    // RC channel numbers, 1-16.
    int linear_x_channel{3};
    int angular_z_channel{1};
    int e_stop_channel{5};
    int e_stop_latch_reset_channel{4};

    // A switch channel below this raw value is "low".
    int channel_switch_threshold{500};
    unsigned int switch_settle_frames{kDefaultSwitchSettleFrames};

    LinkMonitorConfig link;
};

enum class TickStatus
{
    kWaitingForFirstFrame,
    kLinkLost,
    kActive,
};

// Turns RC input into velocity commands and E-Stop requests, once per control tick.
//
// Per tick:
//   1. Before the first frame nothing is published: a default frame would map to full negative
//      deflection on both axes.
//   2. If the link is unhealthy (see LinkMonitor), one zero command is published and then
//      nothing - so the rover stops at once instead of after twist_mux's timeout, and twist_mux
//      then falls through to its next source. The switches are not evaluated either: their values
//      can't be trusted any more than the sticks'.
//   3. Otherwise the sticks are mapped and published - except that a zero command is published
//      only once, so a centred stick doesn't hold twist_mux on this source and block autonomy.
//   4. Switch position changes are turned into E-Stop requests. A switch flipped while the link
//      was lost fires on recovery, which is what the operator asked for.
//
// Not thread-safe: the node calls it from a single-threaded executor.
class TeleopUseCase
{

public:

    TeleopUseCase(
        const TeleopConfig & config,
        std::shared_ptr<VelocityCommandPort> velocity_port,
        std::shared_ptr<SafetySwitchPort> safety_switch_port);

    void onChannels(const RcFrame & frame, SteadyTime now);

    void onLinkStats(std::uint8_t link_quality, SteadyTime now);

    TickStatus tick(SteadyTime now);

    // Stops commanding: publishes one zero unless the last command already was zero. Called when
    // teleop is being deactivated.
    void stop();

private:

    // Publishes `command`, except a zero that has already been published.
    void publish(const VelocityCommand & command);

    double mapChannel(int channel_number, const AxisMapping & mapping) const;

    void evaluateSwitches();

    TeleopConfig config_;

    std::shared_ptr<VelocityCommandPort> velocity_port_;
    std::shared_ptr<SafetySwitchPort> safety_switch_port_;

    LinkMonitor link_monitor_;
    SwitchDebouncer e_stop_switch_;
    SwitchDebouncer latch_reset_switch_;

    std::optional<RcFrame> last_frame_;

    // Shared by the link-lost and centred-stick paths, so going from one to the other doesn't
    // publish a second zero.
    bool zero_sent_{false};
};

}  // namespace rover_crfs_teleop

#endif  // ROVER_CRFS_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_
