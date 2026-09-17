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

#include "rover_crsf_teleop/domain/link_monitor.hpp"

namespace rover_crsf_teleop
{

namespace
{

bool isFresh(
    const std::optional<SteadyTime> & last, const SteadyTime now,
    const std::chrono::milliseconds timeout)
{
    return last.has_value() && (now - *last) <= timeout;
}

std::optional<std::chrono::milliseconds> ageOf(
    const std::optional<SteadyTime> & last, const SteadyTime now)
{
    if (!last.has_value()) {
        return std::nullopt;
    }
    return std::chrono::duration_cast<std::chrono::milliseconds>(now - *last);
}

}  // namespace

const char * toString(const LinkLossReason reason)
{
    switch (reason) {
        case LinkLossReason::kNone: return "none";
        case LinkLossReason::kNoChannels: return "no rc/channels received";
        case LinkLossReason::kChannelsStale: return "rc/channels stale";
        case LinkLossReason::kNoLinkStats: return "no rc/link received";
        case LinkLossReason::kLinkStatsStale: return "rc/link stale";
        case LinkLossReason::kLowLinkQuality: return "link quality below threshold";
    }
    return "unknown";
}

LinkMonitor::LinkMonitor(const LinkMonitorConfig & config)
: config_(config)
{
}

void LinkMonitor::onChannels(const SteadyTime now)
{
    last_channels_ = now;
}

void LinkMonitor::onLinkStats(const SteadyTime now, const std::uint8_t link_quality)
{
    last_link_stats_ = now;
    last_link_quality_ = link_quality;

    if (link_quality < config_.lq_lost_below) {
        link_quality_ok_ = false;
    } else if (link_quality >= config_.lq_recovered_at) {
        link_quality_ok_ = true;
    }
    // Inside the band: keep the previous state.
}

bool LinkMonitor::isHealthy(const SteadyTime now) const
{
    return lossReason(now) == LinkLossReason::kNone;
}

LinkLossReason LinkMonitor::lossReason(const SteadyTime now) const
{
    if (!last_channels_.has_value()) {
        return LinkLossReason::kNoChannels;
    }

    if (!isFresh(last_channels_, now, config_.channel_timeout)) {
        return LinkLossReason::kChannelsStale;
    }

    if (!config_.require_link_stats) {
        return LinkLossReason::kNone;
    }

    if (!last_link_stats_.has_value()) {
        return LinkLossReason::kNoLinkStats;
    }

    if (!isFresh(last_link_stats_, now, config_.link_stats_timeout)) {
        return LinkLossReason::kLinkStatsStale;
    }

    return link_quality_ok_ ? LinkLossReason::kNone : LinkLossReason::kLowLinkQuality;
}

LinkHealthSnapshot LinkMonitor::snapshot(const SteadyTime now) const
{
    LinkHealthSnapshot snapshot;
    snapshot.channels_age = ageOf(last_channels_, now);
    snapshot.link_stats_age = ageOf(last_link_stats_, now);
    snapshot.link_quality = last_link_quality_;
    snapshot.link_quality_ok = link_quality_ok_;
    snapshot.require_link_stats = config_.require_link_stats;
    snapshot.loss_reason = lossReason(now);
    return snapshot;
}

}  // namespace rover_crsf_teleop
