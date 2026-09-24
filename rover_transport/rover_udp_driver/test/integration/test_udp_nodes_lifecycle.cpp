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
//
// Drives UdpSenderNode and UdpReceiverNode through their lifecycle transitions against plain
// POSIX sockets on localhost. The ACTIVE gate itself is unit-tested in rover_io_context
// (test_byte_bridges.cpp).

#include <gtest/gtest.h>

#include <arpa/inet.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <udp_msgs/msg/udp_packet.hpp>

#include "rover_udp_driver/infrastructure/udp_receiver_node.hpp"
#include "rover_udp_driver/infrastructure/udp_sender_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using rover::transport::udp::UdpReceiverNode;
using rover::transport::udp::UdpSenderNode;
using UdpPacket = udp_msgs::msg::UdpPacket;

namespace
{

constexpr const char * kIp = "127.0.0.1";

// Unique per process and per case, like test_asio_udp_socket.cpp (whose offsets stay below 10).
int testPort(int offset)
{
    return 20000 + (getpid() % 10000) + 20 + offset;
}

sockaddr_in localhost(int port)
{
    sockaddr_in address{};
    address.sin_family = AF_INET;
    address.sin_port = htons(static_cast<std::uint16_t>(port));
    inet_pton(AF_INET, kIp, &address.sin_addr);
    return address;
}

rclcpp::NodeOptions endpointOptions(const std::string & ip, int port)
{
    rclcpp::NodeOptions options;
    options.parameter_overrides({{"ip", ip}, {"port", port}});
    return options;
}

class UdpNodesLifecycleTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        test_node_ = std::make_shared<rclcpp::Node>("udp_nodes_test_peer");
        socket_fd_ = socket(AF_INET, SOCK_DGRAM, 0);
        ASSERT_GE(socket_fd_, 0);
        ASSERT_EQ(fcntl(socket_fd_, F_SETFL, fcntl(socket_fd_, F_GETFL) | O_NONBLOCK), 0);
    }

    void TearDown() override
    {
        close(socket_fd_);
    }

    // Spins `node` and the test peer until `condition` holds or `timeout` passes.
    bool spinUntil(
        const rclcpp_lifecycle::LifecycleNode::SharedPtr & node,
        const std::function<bool()> & condition, std::chrono::seconds timeout = 10s)
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node->get_node_base_interface());
        executor.add_node(test_node_);
        const auto deadline = std::chrono::steady_clock::now() + timeout;
        while (std::chrono::steady_clock::now() < deadline) {
            if (condition()) {
                return true;
            }
            executor.spin_some(20ms);
        }
        return condition();
    }

    rclcpp::Node::SharedPtr test_node_;
    int socket_fd_{-1};
};

}  // namespace

TEST_F(UdpNodesLifecycleTest, SenderWithoutAPortFailsToConfigure)
{
    auto node = std::make_shared<UdpSenderNode>(endpointOptions(kIp, 0));
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(UdpNodesLifecycleTest, SenderWithAnUnparsableIpFailsToConfigure)
{
    auto node = std::make_shared<UdpSenderNode>(endpointOptions("not-an-ip", testPort(0)));
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(UdpNodesLifecycleTest, ActiveSenderForwardsUdpWriteToTheEndpoint)
{
    const int port = testPort(1);
    const auto address = localhost(port);
    ASSERT_EQ(bind(socket_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address)), 0);

    auto node = std::make_shared<UdpSenderNode>(endpointOptions(kIp, port));
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

    auto pub = test_node_->create_publisher<UdpPacket>("udp_write", rclcpp::QoS(10));
    UdpPacket packet;
    packet.data = {0xE1, 0x00, 0xFF, 0x7F};
    std::vector<std::uint8_t> received(64);
    ssize_t length = -1;
    // Republished until it arrives: the subscription may not be matched yet.
    EXPECT_TRUE(spinUntil(node, [&]() {
        length = recv(socket_fd_, received.data(), received.size(), 0);
        if (length < 0) {
            pub->publish(packet);
        }
        return length >= 0;
    }));
    ASSERT_GE(length, 0);
    received.resize(static_cast<std::size_t>(length));
    EXPECT_EQ(received, packet.data);

    EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(UdpNodesLifecycleTest, ReceiverWithoutAPortFailsToConfigure)
{
    auto node = std::make_shared<UdpReceiverNode>(endpointOptions(kIp, 0));
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(UdpNodesLifecycleTest, ActiveReceiverPublishesDatagramsOnUdpRead)
{
    const int port = testPort(2);
    auto node = std::make_shared<UdpReceiverNode>(endpointOptions(kIp, port));
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

    std::vector<std::uint8_t> received;
    auto sub = test_node_->create_subscription<UdpPacket>(
        "udp_read", rclcpp::QoS(100),
        [&received](const UdpPacket::SharedPtr msg) {received = msg->data;});

    const std::vector<std::uint8_t> datagram = {0x10, 0x20, 0x30};
    const auto address = localhost(port);
    // Resent until it is published: the test subscription may not be matched yet.
    EXPECT_TRUE(spinUntil(node, [&]() {
        if (received.empty()) {
            sendto(
                socket_fd_, datagram.data(), datagram.size(), 0,
                reinterpret_cast<const sockaddr *>(&address), sizeof(address));
        }
        return !received.empty();
    }));
    EXPECT_EQ(received, datagram);

    EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(UdpNodesLifecycleTest, ReceiverCleanupReleasesItsPort)
{
    const int port = testPort(3);
    const auto address = localhost(port);
    const auto bind_test_socket = [&]() {
        return bind(socket_fd_, reinterpret_cast<const sockaddr *>(&address), sizeof(address));
    };
    auto node = std::make_shared<UdpReceiverNode>(endpointOptions(kIp, port));
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_NE(bind_test_socket(), 0) << "the configured receiver should hold the port";

    ASSERT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(bind_test_socket(), 0);
}

TEST_F(UdpNodesLifecycleTest, ReceiverSurvivesTeardownWhileDatagramsArrive)
{
    // Regression test: close() used to return while a receive handler was still delivering a
    // datagram on the io thread, which then used the packet publisher cleanup had just freed.
    const int port = testPort(4);
    auto node = std::make_shared<UdpReceiverNode>(endpointOptions(kIp, port));

    std::atomic<bool> stop{false};
    std::thread flood([&]() {
        const auto address = localhost(port);
        const std::vector<std::uint8_t> datagram(64, 0xAB);
        while (!stop) {
            sendto(
                socket_fd_, datagram.data(), datagram.size(), 0,
                reinterpret_cast<const sockaddr *>(&address), sizeof(address));
        }
    });

    for (int cycle = 0; cycle < 50; ++cycle) {
        ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE) << "cycle " << cycle;
        ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE) << "cycle " << cycle;
        ASSERT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE) << "cycle " << cycle;
        ASSERT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED) << "cycle " << cycle;
    }

    stop = true;
    flood.join();
}
