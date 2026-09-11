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

#include "rover_crfs_teleop/domain/link_monitor.hpp"

namespace rover_crfs_teleop
{

namespace
{

bool isFresh(
    const std::optional<SteadyTime> & last, const SteadyTime now,
    const std::chrono::milliseconds timeout)
{
    return last.has_value() && (now - *last) <= timeout;
}

}  // namespace

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

    if (link_quality < config_.lq_lost_below) {
        link_quality_ok_ = false;
    } else if (link_quality >= config_.lq_recovered_at) {
        link_quality_ok_ = true;
    }
    // Inside the band: keep the previous state.
}

bool LinkMonitor::isHealthy(const SteadyTime now) const
{
    if (!isFresh(last_channels_, now, config_.channel_timeout)) {
        return false;
    }

    if (!config_.require_link_stats) {
        return true;
    }

    return isFresh(last_link_stats_, now, config_.link_stats_timeout) && link_quality_ok_;
}

}  // namespace rover_crfs_teleop
