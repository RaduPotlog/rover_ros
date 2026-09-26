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
// Drives SerialBridgeNode through its lifecycle transitions. A pty pair stands in for the UART:
// the node opens the slave end with the real ASIO adapter and the test talks to the master end.
// The ACTIVE gate itself is unit-tested in rover_io_context (test_byte_bridges.cpp).

#include <gtest/gtest.h>

#include <fcntl.h>
#include <poll.h>
#include <pty.h>
#include <termios.h>
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
#include <std_msgs/msg/u_int8_multi_array.hpp>

#include "rover_serial_driver/infrastructure/serial_bridge_node.hpp"

using namespace std::chrono_literals;
using lifecycle_msgs::msg::State;
using rover::transport::serial::SerialBridgeNode;
using UInt8MultiArray = std_msgs::msg::UInt8MultiArray;

namespace
{

class SerialBridgeNodeLifecycleTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() { rclcpp::init(0, nullptr); }
    static void TearDownTestSuite() { rclcpp::shutdown(); }

    void SetUp() override
    {
        char slave_name[128] = {};
        ASSERT_EQ(openpty(&master_fd_, &slave_fd_, slave_name, nullptr, nullptr), 0);
        slave_path_ = slave_name;

        termios raw{};
        ASSERT_EQ(tcgetattr(master_fd_, &raw), 0);
        cfmakeraw(&raw);
        ASSERT_EQ(tcsetattr(master_fd_, TCSANOW, &raw), 0);
        ASSERT_EQ(fcntl(master_fd_, F_SETFL, fcntl(master_fd_, F_GETFL) | O_NONBLOCK), 0);

        test_node_ = std::make_shared<rclcpp::Node>("serial_bridge_test_peer");
    }

    void TearDown() override
    {
        node_.reset();
        close(slave_fd_);
        close(master_fd_);
    }

    std::shared_ptr<SerialBridgeNode> makeNode(
        const std::string & device_name, const std::string & parity = "none")
    {
        rclcpp::NodeOptions options;
        options.parameter_overrides({
            {"device_name", device_name},
            {"baud_rate", 115200},
            {"flow_control", "none"},
            {"parity", parity},
            {"stop_bits", "1"},
        });
        node_ = std::make_shared<SerialBridgeNode>(options);
        return node_;
    }

    // Spins both nodes until `condition` holds or `timeout` passes.
    bool spinUntil(const std::function<bool()> & condition, std::chrono::seconds timeout = 10s)
    {
        rclcpp::executors::SingleThreadedExecutor executor;
        executor.add_node(node_->get_node_base_interface());
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

    std::vector<std::uint8_t> readMaster()
    {
        std::vector<std::uint8_t> buffer(256);
        const auto n = read(master_fd_, buffer.data(), buffer.size());
        buffer.resize(n > 0 ? static_cast<std::size_t>(n) : 0);
        return buffer;
    }

    int master_fd_{-1};
    int slave_fd_{-1};
    std::string slave_path_;
    rclcpp::Node::SharedPtr test_node_;
    std::shared_ptr<SerialBridgeNode> node_;
};

}  // namespace

TEST_F(SerialBridgeNodeLifecycleTest, EmptyDeviceNameFailsToConfigure)
{
    EXPECT_EQ(makeNode("")->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SerialBridgeNodeLifecycleTest, InvalidSerialOptionFailsToConfigure)
{
    EXPECT_EQ(makeNode(slave_path_, "sometimes")->configure().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SerialBridgeNodeLifecycleTest, MissingDeviceFailsToConfigure)
{
    EXPECT_EQ(
        makeNode("/dev/rover-serial-test-no-such-tty")->configure().id(),
        State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SerialBridgeNodeLifecycleTest, ActiveNodeBridgesBytesBothWays)
{
    auto node = makeNode(slave_path_);
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->activate().id(), State::PRIMARY_STATE_ACTIVE);

    // Device -> serial_read.
    std::vector<std::uint8_t> read_bytes;
    auto sub = test_node_->create_subscription<UInt8MultiArray>(
        "serial_read", rclcpp::QoS(100),
        [&read_bytes](const UInt8MultiArray::SharedPtr msg) {
            read_bytes.insert(read_bytes.end(), msg->data.begin(), msg->data.end());
        });
    const std::vector<std::uint8_t> from_device = {0xC8, 0x18, 0x16};
    ASSERT_EQ(write(master_fd_, from_device.data(), from_device.size()),
              static_cast<ssize_t>(from_device.size()));
    EXPECT_TRUE(spinUntil([&]() { return read_bytes.size() >= from_device.size(); }));
    EXPECT_EQ(read_bytes, from_device);

    // serial_write -> device. Republished until it arrives: the subscription may not be
    // matched yet.
    auto pub = test_node_->create_publisher<UInt8MultiArray>("serial_write", rclcpp::QoS(10));
    UInt8MultiArray to_device;
    to_device.data = {0x01, 0x02, 0x03};
    std::vector<std::uint8_t> written;
    EXPECT_TRUE(spinUntil([&]() {
        const auto chunk = readMaster();
        written.insert(written.end(), chunk.begin(), chunk.end());
        if (written.empty()) {
            pub->publish(to_device);
        }
        return written.size() >= to_device.data.size();
    }));
    ASSERT_GE(written.size(), to_device.data.size());
    EXPECT_EQ(
        std::vector<std::uint8_t>(written.begin(), written.begin() + 3), to_device.data);

    EXPECT_EQ(node->deactivate().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
}

TEST_F(SerialBridgeNodeLifecycleTest, CanBeConfiguredAgainAfterCleanup)
{
    auto node = makeNode(slave_path_);
    ASSERT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    ASSERT_EQ(node->cleanup().id(), State::PRIMARY_STATE_UNCONFIGURED);
    EXPECT_EQ(node->configure().id(), State::PRIMARY_STATE_INACTIVE);
    EXPECT_EQ(node->shutdown().id(), State::PRIMARY_STATE_FINALIZED);
}

TEST_F(SerialBridgeNodeLifecycleTest, SurvivesTeardownWhileBytesArrive)
{
    // Regression test: close() used to return while a read handler was still delivering bytes
    // on the io thread, which then used the publisher cleanup had just freed.
    auto node = makeNode(slave_path_);

    std::atomic<bool> stop{false};
    std::thread flood([&]() {
        const std::vector<std::uint8_t> chunk(64, 0xAB);
        while (!stop) {
            // (void) doesn't silence warn_unused_result, so test the result instead.
            if (write(master_fd_, chunk.data(), chunk.size()) < 0) {
                continue;  // EAGAIN when the pty buffer is full: fine, keep flooding.
            }
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
