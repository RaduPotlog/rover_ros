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

#ifndef ROVER_CRSF_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_
#define ROVER_CRSF_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_

#include <cstdint>
#include <memory>
#include <optional>

#include "rover_crsf_teleop/domain/link_monitor.hpp"
#include "rover_crsf_teleop/domain/ports.hpp"
#include "rover_crsf_teleop/domain/rc_calibration.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"
#include "rover_crsf_teleop/domain/rim_speed_limit.hpp"
#include "rover_crsf_teleop/domain/stick_mapping.hpp"
#include "rover_crsf_teleop/domain/switch_debouncer.hpp"
#include "rover_crsf_teleop/domain/teleop_health.hpp"

namespace rover_crsf_teleop
{

struct TeleopConfig
{
    AxisMapping linear_x_mapping;
    AxisMapping angular_z_mapping;

    // Every channel's measured endpoints. Only the two entries the mappings above are built from
    // are read per tick; the rest are carried so a calibration can be applied, persisted and
    // displayed whole rather than only for the channels that happen to drive something.
    ChannelCalibration calibration{defaultCalibration()};

    // RC channel numbers, 1-16.
    int linear_x_channel{3};
    int angular_z_channel{1};

    // Outer-wheel rim speed budget, m/s, and half the effective track width, m (see
    // domain/rim_speed_limit.hpp). Either <= 0 disables the limit.
    double max_wheel_rim_speed{0.0};
    double half_track_width{0.0};
    int e_stop_channel{5};
    int e_stop_latch_reset_channel{4};

    // A switch channel below this raw value is "low".
    int channel_switch_threshold{500};
    unsigned int switch_settle_frames{kDefaultSwitchSettleFrames};

    LinkMonitorConfig link;
};

// `config` with both stick axes re-anchored on `calibration`, and `calibration` recorded on it.
//
// This is the single definition of what "applying a calibration" means, and every path that has
// one goes through it: the parameters read at configure, a calibration loaded from the store, and
// one handed over at run time by the apply service. Written once because the alternative - the
// same three lines at each site - is how one path quietly keeps driving on stale endpoints when a
// third axis is added.
//
// `config` is the base to apply onto, not somewhere to accumulate: a run-time apply passes the
// pre-calibration config, so applying twice is the same as applying once.
TeleopConfig applyCalibration(const TeleopConfig & config, const ChannelCalibration & calibration);

enum class TickStatus
{
    kWaitingForFirstFrame,
    kLinkLost,
    kInhibited,
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
// An RC calibration session inhibits all of this (see setCommandInhibited): one zero is published
// and then nothing, and the switches are not evaluated - the sweep walks the E-Stop switch
// through both ends on purpose, and that must not reach the hardware interface.
//
// Not thread-safe: the node calls it from a single-threaded executor.
// Read-only snapshot for diagnostics; building it has no side effects on teleop.
struct TeleopDiagnostics
{
    bool first_frame_received{false};
    bool inhibited{false};
    LinkHealthSnapshot link;
    HealthReport health;
    VelocityCommand last_command;
    std::optional<SwitchPosition> e_stop_switch;
    std::optional<SwitchPosition> latch_reset_switch;
};

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

    // Holds teleop off while something else owns the sticks - today, an RC calibration session.
    // Checked before every other branch of tick(), so there is no path that can command while it
    // is set. Independent of the lifecycle state: deactivating is the operator's interlock, this
    // is the node's.
    void setCommandInhibited(bool inhibited);

    // Restarts both switch debouncers' settle periods. Call when releasing an inhibit: no frames
    // reached the debouncers while it was set, so their recorded positions are from before it.
    void rearmSwitches();

    TeleopDiagnostics diagnostics(SteadyTime now) const;

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

    VelocityCommand last_command_;

    bool inhibited_{false};
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_APPLICATION_TELEOP_USE_CASE_HPP_
