// Copyright 2021 the Autoware Foundation
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
// Modified 2026 by Mechatronics Academy: relayouted from
// udp_driver/src/udp_bridge_node.cpp (ros-drivers/transport_drivers v1.2.0).

#include <memory>
#include <stdexcept>

#include <lifecycle_msgs/msg/state.hpp>
#include <rclcpp/rclcpp.hpp>

#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_udp_driver/infrastructure/udp_receiver_node.hpp"
#include "rover_udp_driver/infrastructure/udp_sender_node.hpp"

using lifecycle_msgs::msg::State;
using rover::transport::IoContext;
using rover::transport::udp::UdpReceiverNode;
using rover::transport::udp::UdpSenderNode;

// Provides both UDP sending and receiving from one process, on a shared IoContext.
int main(int argc, char ** argv)
{
    setvbuf(stdout, nullptr, _IONBF, BUFSIZ);

    rclcpp::init(argc, argv);
    rclcpp::executors::SingleThreadedExecutor executor;
    rclcpp::NodeOptions options;
    IoContext ctx{};

    auto receiver_node = std::make_shared<UdpReceiverNode>(options, ctx);
    auto sender_node = std::make_shared<UdpSenderNode>(options, ctx);

    executor.add_node(receiver_node->get_node_base_interface());
    executor.add_node(sender_node->get_node_base_interface());

    if (receiver_node->configure().id() != State::PRIMARY_STATE_INACTIVE) {
        throw std::runtime_error{"Failed to configure UDP receiver."};
    }
    if (receiver_node->activate().id() != State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error{"Failed to activate UDP receiver."};
    }

    if (sender_node->configure().id() != State::PRIMARY_STATE_INACTIVE) {
        throw std::runtime_error{"Failed to configure UDP sender."};
    }
    if (sender_node->activate().id() != State::PRIMARY_STATE_ACTIVE) {
        throw std::runtime_error{"Failed to activate UDP sender."};
    }

    executor.spin();

    rclcpp::shutdown();

    return 0;
}
