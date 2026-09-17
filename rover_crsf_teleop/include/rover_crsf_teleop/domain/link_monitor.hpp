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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_LINK_MONITOR_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_LINK_MONITOR_HPP_

#include <chrono>
#include <cstdint>
#include <optional>

namespace rover_crsf_teleop
{

// Monotonic, so the watchdog neither jumps with wall-clock corrections nor follows use_sim_time.
using SteadyTime = std::chrono::steady_clock::time_point;

struct LinkMonitorConfig
{
    // A decoded RC frame older than this means the receiver has stopped delivering frames (or
    // the serial bridge has). At the receiver's 50 Hz packet rate, 200 ms is 10 missed frames.
    std::chrono::milliseconds channel_timeout{200};

    // rc/link older than this means the link quality below can no longer be trusted. Link stats
    // arrive at the ELRS telemetry rate, far slower than channels - measure with
    // `ros2 topic hz rc/link` before tightening.
    std::chrono::milliseconds link_stats_timeout{1000};

    // Uplink link quality (CRSF LQ, 0-100 %) hysteresis band: the link is lost once LQ drops
    // below `lq_lost_below` and only counts as recovered at `lq_recovered_at` or above, so a value
    // hovering at the edge can't make teleop - and twist_mux's source selection - flap.
    std::uint8_t lq_lost_below{30};
    std::uint8_t lq_recovered_at{50};

    // Freshness of rc/channels alone can't catch a receiver in failsafe that keeps sending held
    // (or preset) channel values after losing the transmitter; link quality can. There is no
    // reason to disable this on ELRS hardware: the receiver always sends LINK_STATISTICS.
    bool require_link_stats{true};
};

// Decides whether the RC link is healthy enough for its stick values to command motion.
//
// Healthy means all of:
//   - an rc/channels frame arrived within `channel_timeout`;
//   - when `require_link_stats`: an rc/link report arrived within `link_stats_timeout`, and the
//     link quality is in its "ok" hysteresis state.
//
// Before the first frame (and the first link report, when required) the link is unhealthy.
// Why the link counts as lost, checked in this order; kNone means healthy.
enum class LinkLossReason
{
    kNone,
    kNoChannels,
    kChannelsStale,
    kNoLinkStats,
    kLinkStatsStale,
    kLowLinkQuality,
};

const char * toString(LinkLossReason reason);

// Read-only view of the monitor for diagnostics. Ages are nullopt until the first message.
struct LinkHealthSnapshot
{
    std::optional<std::chrono::milliseconds> channels_age;
    std::optional<std::chrono::milliseconds> link_stats_age;
    std::optional<std::uint8_t> link_quality;
    bool link_quality_ok{false};
    bool require_link_stats{true};
    LinkLossReason loss_reason{LinkLossReason::kNoChannels};
};

class LinkMonitor
{

public:

    explicit LinkMonitor(const LinkMonitorConfig & config);

    void onChannels(SteadyTime now);

    void onLinkStats(SteadyTime now, std::uint8_t link_quality);

    bool isHealthy(SteadyTime now) const;

    LinkHealthSnapshot snapshot(SteadyTime now) const;

private:

    LinkLossReason lossReason(SteadyTime now) const;

    LinkMonitorConfig config_;

    std::optional<SteadyTime> last_channels_;
    std::optional<SteadyTime> last_link_stats_;
    std::optional<std::uint8_t> last_link_quality_;

    // Starts false: nothing has proven the link good yet.
    bool link_quality_ok_{false};
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_DOMAIN_LINK_MONITOR_HPP_
