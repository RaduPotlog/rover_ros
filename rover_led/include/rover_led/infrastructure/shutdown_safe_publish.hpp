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

#include "rclcpp/context.hpp"
#include "rclcpp/exceptions.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/logging.hpp"
#include "rclcpp/utilities.hpp"

namespace rover_led
{

// Publishes unless the context has been shut down, and never lets a failed publish escape.
//
// On Ctrl-C rmw_zenoh starts refusing publishes (generic error, so rclcpp throws RCLError)
// while timer callbacks are still running - and even from pre-shutdown callbacks, where the
// context is still valid. An uncaught RCLError aborts the whole component container, so the
// message is dropped instead: a lost LED frame is never worth a crash.
template<typename PublisherPtrT, typename MessageT>
void publishUnlessShutdown(
    const rclcpp::Context::SharedPtr & context, const rclcpp::Logger & logger,
    const PublisherPtrT & publisher, MessageT && msg)
{
    if (!rclcpp::ok(context)) {
        return;
    }

    try {
        publisher->publish(std::forward<MessageT>(msg));
    } catch (const rclcpp::exceptions::RCLError & e) {
        if (rclcpp::ok(context)) {
            RCLCPP_WARN(logger, "Dropped message, publish failed: %s", e.what());
        }
    }
}

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_SHUTDOWN_SAFE_PUBLISH_HPP_
