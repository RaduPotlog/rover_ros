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

#include "rover_safety/infrastructure/configure_retry.hpp"

#include <string>
#include <utility>

#include <lifecycle_msgs/msg/state.hpp>

namespace rover_safety::infrastructure
{

ConfigureRetry::ConfigureRetry(nav2::LifecycleNode & node, const std::chrono::duration<double> period)
: node_(node)
, period_(period)
{
}

void ConfigureRetry::onFailure(const std::string & error)
{
    ++failed_attempts_;
    last_error_ = error;

    if (!node_.get_parameter("autostart_node").as_bool()) {
        return;
    }

    RCLCPP_WARN(
        node_.get_logger(), "Configure attempt %u failed; retrying in %.1f s.", failed_attempts_,
        period_.count());

    timer_ = node_.create_wall_timer(period_, [this]() { retry(); });
}

void ConfigureRetry::onSuccess()
{
    if (failed_attempts_ > 0) {
        RCLCPP_INFO(
            node_.get_logger(), "Configured after %u failed attempt(s).", failed_attempts_);
    }
    last_error_.clear();
    if (timer_) {
        timer_->cancel();
    }
}

void ConfigureRetry::retry()
{
    // One-shot. Keep the timer alive for the rest of this callback: a failed configure below
    // re-enters onFailure(), which replaces timer_.
    const auto self = std::move(timer_);
    self->cancel();

    using lifecycle_msgs::msg::State;

    // Something else (a manual transition) may have moved the node on meanwhile.
    if (node_.get_current_state().id() != State::PRIMARY_STATE_UNCONFIGURED) {
        return;
    }

    // Blocks this executor thread: building the tree waits up to
    // ros_communication_timeout.availability for each service a BT node needs. Accepted because
    // an unconfigured node has no subscriptions or timers of its own yet, so there is nothing
    // else on this thread to starve.
    if (node_.configure().id() != State::PRIMARY_STATE_INACTIVE) {
        return;
    }

    if (node_.activate().id() != State::PRIMARY_STATE_ACTIVE) {
        RCLCPP_ERROR(node_.get_logger(), "Configured on retry but failed to activate.");
    }
}

}  // namespace rover_safety::infrastructure
