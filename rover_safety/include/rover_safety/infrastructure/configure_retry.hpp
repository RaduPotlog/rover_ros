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

#ifndef ROVER_SAFETY_INFRASTRUCTURE_CONFIGURE_RETRY_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_CONFIGURE_RETRY_HPP_

#include <chrono>
#include <string>

#include <nav2_ros_common/lifecycle_node.hpp>
#include <rclcpp/rclcpp.hpp>

namespace rover_safety::infrastructure
{

/**
 * Retries a failed configure of an autostarted lifecycle node.
 *
 * nav2::LifecycleNode::autostart() tries configure() exactly once. A safety tree whose service
 * server is not up yet (led/set_animation, hardware_interface/sw_user_e_stop_set) throws while it
 * is built, so without a retry the node stays unconfigured - and unsubscribed - until restarted.
 *
 * Only an autostarted node (autostart_node = true) is retried: with a lifecycle manager in charge,
 * retrying is the manager's decision.
 */
class ConfigureRetry
{
public:
    ConfigureRetry(nav2::LifecycleNode & node, std::chrono::duration<double> period);

    /** Called from on_configure when it fails; arms a one-shot configure + activate retry. */
    void onFailure(const std::string & error);

    /** Called from on_configure when it succeeds; drops the recorded error. */
    void onSuccess();

    unsigned failedAttempts() const { return failed_attempts_; }
    const std::string & lastError() const { return last_error_; }

private:
    void retry();

    nav2::LifecycleNode & node_;
    const std::chrono::duration<double> period_;
    unsigned failed_attempts_{0};
    std::string last_error_;
    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_CONFIGURE_RETRY_HPP_
