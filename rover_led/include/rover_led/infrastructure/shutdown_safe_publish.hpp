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

#ifndef ROVER_LED_INFRASTRUCTURE_SHUTDOWN_SAFE_PUBLISH_HPP_
#define ROVER_LED_INFRASTRUCTURE_SHUTDOWN_SAFE_PUBLISH_HPP_

#include <utility>

#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"

#include "rover_utils/shutdown_gate.hpp"

namespace rover_led
{

// Publishes unless shutdown has started, and never lets a failed publish escape.
//
// rmw_zenoh closes its session as soon as rcl_shutdown() starts, but the context stays valid
// until the session is fully closed (~2 s), so rclcpp::ok() alone lets timers keep publishing
// into a closed session. The gate closes in a pre-shutdown callback, before that window.
// A publish can still fail (generic error, so rclcpp throws RCLError), and an uncaught RCLError
// aborts the whole component container, so the message is dropped instead: a lost LED frame is
// never worth a crash.
template<typename PublisherPtrT, typename MessageT>
void publishUnlessShutdown(
    const rover_utils::ros::ShutdownGate & gate, const rclcpp::Logger & logger,
    const PublisherPtrT & publisher, MessageT && msg)
{
    if (!gate.isOpen()) {
        return;
    }

    try {
        publisher->publish(std::forward<MessageT>(msg));
    } catch (const rclcpp::exceptions::RCLError & e) {
        if (gate.isOpen()) {
            RCLCPP_WARN(logger, "Dropped message, publish failed: %s", e.what());
        }
    }
}

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_SHUTDOWN_SAFE_PUBLISH_HPP_
