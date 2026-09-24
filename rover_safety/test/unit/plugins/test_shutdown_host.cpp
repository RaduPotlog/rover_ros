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

#include <openssl/hmac.h>

#include <algorithm>
#include <chrono>
#include <iomanip>
#include <optional>
#include <regex>
#include <sstream>
#include <string>

#include "rover_safety/plugins/shutdown_host.hpp"

#include "plugin_test_utils.hpp"

using rover_safety::ShutdownHost;
using rover_safety::ShutdownHostState;
using rover_safety::test::OneShotHttpServer;

namespace
{

class ShutdownHostWrapper : public ShutdownHost
{
public:
    using ShutdownHost::ShutdownHost;
    using ShutdownHost::getTimeSinceEpoch;
    using ShutdownHost::pollAvailability;

    /** Polls until the background ping has an answer. */
    std::optional<bool> waitForAvailability(std::chrono::seconds deadline = std::chrono::seconds(5))
    {
        const auto start = std::chrono::steady_clock::now();
        auto available = pollAvailability();
        while (!available && std::chrono::steady_clock::now() - start < deadline) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
            available = pollAvailability();
        }
        return available;
    }
};

/** Calls the host until it leaves the given transient states or `deadline` passes. */
ShutdownHostState callUntilSettled(
    ShutdownHost & host, std::chrono::seconds deadline = std::chrono::seconds(20))
{
    const auto start = std::chrono::steady_clock::now();
    do {
        host.call();
        const auto state = host.getState();
        if (state == ShutdownHostState::PINGING || state == ShutdownHostState::SUCCESS ||
            state == ShutdownHostState::FAILURE || state == ShutdownHostState::SKIPPED) {
            return state;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } while (std::chrono::steady_clock::now() - start < deadline);
    return host.getState();
}

std::string hmacSha256Hex(const std::string & key, const std::string & data)
{
    unsigned int length = 0;
    const unsigned char * digest = HMAC(
        EVP_sha256(), key.data(), static_cast<int>(key.size()),
        reinterpret_cast<const unsigned char *>(data.data()), data.size(), nullptr, &length);

    std::stringstream hex;
    for (unsigned int i = 0; i < length; ++i) {
        hex << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(digest[i]);
    }
    return hex.str();
}

}  // namespace

TEST(ShutdownHostTest, LocalhostIsAvailable)
{
    ShutdownHostWrapper host("127.0.0.1", "3003", "secret", 1.0);
    EXPECT_EQ(host.waitForAvailability(), std::optional<bool>(true));
}

TEST(ShutdownHostTest, UnreachableHostIsSkipped)
{
    ShutdownHostWrapper host(rover_safety::test::kUnreachableIp, "3003", "secret", 1.0);

    EXPECT_EQ(host.waitForAvailability(), std::optional<bool>(false));
    EXPECT_EQ(callUntilSettled(host), ShutdownHostState::SKIPPED);
}

// The shutdown tree ticks every host from the safety node's only executor thread, so call() must
// return at once even while a ping to a dead host takes its full second.
TEST(ShutdownHostTest, CallNeverBlocks)
{
    ShutdownHost host(rover_safety::test::kUnreachableIp, "3003", "secret", 1.0);

    auto slowest = std::chrono::steady_clock::duration::zero();
    const auto start = std::chrono::steady_clock::now();
    while (host.getState() == ShutdownHostState::IDLE &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(5)) {
        const auto before = std::chrono::steady_clock::now();
        host.call();
        slowest = std::max(slowest, std::chrono::steady_clock::now() - before);
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    EXPECT_EQ(host.getState(), ShutdownHostState::SKIPPED);
    EXPECT_LT(slowest, std::chrono::milliseconds(50));
}

TEST(ShutdownHostTest, TimeSinceEpochIsCurrent)
{
    ShutdownHostWrapper host("127.0.0.1", "3003", "secret", 1.0);
    const auto now = std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();

    EXPECT_NEAR(static_cast<double>(host.getTimeSinceEpoch()), static_cast<double>(now), 1.0);
}

TEST(ShutdownHostTest, ServerNotRunningFails)
{
    ShutdownHost host("127.0.0.1", OneShotHttpServer::closedPort(), "secret", 1.0);

    EXPECT_EQ(callUntilSettled(host), ShutdownHostState::FAILURE);
    EXPECT_FALSE(host.getError().empty());
}

TEST(ShutdownHostTest, ServerErrorFails)
{
    OneShotHttpServer server("500 Internal Server Error");
    ShutdownHost host("127.0.0.1", server.port(), "secret", 1.0);

    EXPECT_EQ(callUntilSettled(host), ShutdownHostState::FAILURE);
    EXPECT_NE(host.getError().find("500"), std::string::npos) << host.getError();
}

TEST(ShutdownHostTest, AcceptedRequestIsSignedThenWaitsForHostToGoDown)
{
    const std::string secret = "rover-secret";
    OneShotHttpServer server("200 OK");
    ShutdownHost host("127.0.0.1", server.port(), secret, 1.0);

    EXPECT_EQ(callUntilSettled(host), ShutdownHostState::PINGING) << host.getError();

    const std::regex request_line(R"(GET /shutdown\?ts=(\d+)&sig=([0-9a-f]{64}) HTTP)");
    std::smatch match;
    const auto request = server.request();
    ASSERT_TRUE(std::regex_search(request, match, request_line)) << request;
    EXPECT_EQ(match[2].str(), hmacSha256Hex(secret, "/shutdown|" + match[1].str()));

    // 127.0.0.1 keeps answering ping, so the host never "goes down".
    const auto start = std::chrono::steady_clock::now();
    while (host.getState() == ShutdownHostState::PINGING &&
           std::chrono::steady_clock::now() - start < std::chrono::seconds(10)) {
        host.call();
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    EXPECT_EQ(host.getState(), ShutdownHostState::FAILURE);
    EXPECT_EQ(host.getError(), "Timeout waiting for host to shutdown");
}
