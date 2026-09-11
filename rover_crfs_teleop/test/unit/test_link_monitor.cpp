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

#include <gtest/gtest.h>

#include <chrono>

#include "rover_crfs_teleop/domain/link_monitor.hpp"

namespace rover_crfs_teleop
{
namespace
{

using namespace std::chrono_literals;

LinkMonitorConfig testConfig()
{
    LinkMonitorConfig config;
    config.channel_timeout = 200ms;
    config.link_stats_timeout = 1000ms;
    config.lq_lost_below = 30;
    config.lq_recovered_at = 50;
    config.require_link_stats = true;
    return config;
}

const SteadyTime kT0{};

}  // namespace

TEST(LinkMonitorTest, UnhealthyBeforeAnyInput)
{
    const LinkMonitor monitor(testConfig());

    EXPECT_FALSE(monitor.isHealthy(kT0));
}

TEST(LinkMonitorTest, HealthyWithFreshChannelsAndGoodLinkQuality)
{
    LinkMonitor monitor(testConfig());
    monitor.onChannels(kT0);
    monitor.onLinkStats(kT0, 100);

    EXPECT_TRUE(monitor.isHealthy(kT0 + 100ms));
}

TEST(LinkMonitorTest, ChannelsAloneAreNotEnoughWhenLinkStatsRequired)
{
    LinkMonitor monitor(testConfig());
    monitor.onChannels(kT0);

    EXPECT_FALSE(monitor.isHealthy(kT0));
}

TEST(LinkMonitorTest, ChannelTimeoutMakesTheLinkUnhealthy)
{
    // The stale-sticks regression: channels that stop arriving must not keep commanding motion,
    // whatever the last link stats said.
    LinkMonitor monitor(testConfig());
    monitor.onChannels(kT0);
    monitor.onLinkStats(kT0, 100);

    EXPECT_TRUE(monitor.isHealthy(kT0 + 200ms));
    EXPECT_FALSE(monitor.isHealthy(kT0 + 201ms));
}

TEST(LinkMonitorTest, LinkStatsTimeoutMakesTheLinkUnhealthy)
{
    LinkMonitor monitor(testConfig());
    monitor.onLinkStats(kT0, 100);
    monitor.onChannels(kT0 + 1001ms);

    EXPECT_FALSE(monitor.isHealthy(kT0 + 1001ms));
}

TEST(LinkMonitorTest, LinkStatsIgnoredWhenNotRequired)
{
    LinkMonitorConfig config = testConfig();
    config.require_link_stats = false;
    LinkMonitor monitor(config);
    monitor.onChannels(kT0);

    EXPECT_TRUE(monitor.isHealthy(kT0));

    monitor.onLinkStats(kT0, 0);
    EXPECT_TRUE(monitor.isHealthy(kT0));
}

TEST(LinkMonitorTest, LowLinkQualityMakesTheLinkUnhealthyDespiteFreshChannels)
{
    // A receiver in failsafe can keep sending held channel values; LQ is what catches it.
    LinkMonitor monitor(testConfig());
    monitor.onChannels(kT0);
    monitor.onLinkStats(kT0, 100);
    monitor.onLinkStats(kT0, 29);

    EXPECT_FALSE(monitor.isHealthy(kT0));
}

TEST(LinkMonitorTest, LinkQualityHysteresisKeepsThePreviousState)
{
    LinkMonitor monitor(testConfig());
    monitor.onChannels(kT0);

    // Inside the band from the start: never proven good.
    monitor.onLinkStats(kT0, 40);
    EXPECT_FALSE(monitor.isHealthy(kT0));

    monitor.onLinkStats(kT0, 50);
    EXPECT_TRUE(monitor.isHealthy(kT0));

    // Dropping into the band keeps it good...
    monitor.onLinkStats(kT0, 30);
    EXPECT_TRUE(monitor.isHealthy(kT0));

    // ...below it loses it...
    monitor.onLinkStats(kT0, 29);
    EXPECT_FALSE(monitor.isHealthy(kT0));

    // ...and climbing back into the band does not recover it.
    monitor.onLinkStats(kT0, 49);
    EXPECT_FALSE(monitor.isHealthy(kT0));
}

}  // namespace rover_crfs_teleop
