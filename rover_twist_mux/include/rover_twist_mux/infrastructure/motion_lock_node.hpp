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

#ifndef ROVER_TWIST_MUX_INFRASTRUCTURE_MOTION_LOCK_NODE_HPP_
#define ROVER_TWIST_MUX_INFRASTRUCTURE_MOTION_LOCK_NODE_HPP_

#include <memory>
#include <optional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rover_msgs/msg/gpio_state.hpp"

#include "rover_twist_mux/domain/safety_io_flags.hpp"
#include "rover_twist_mux/motion_lock_params.hpp"

namespace rover_twist_mux
{

/**
 * @brief Translates rover_msgs/GpioState into the std_msgs/Bool lock topic twist_mux consumes.
 * @details twist_mux locks are typed std_msgs/Bool and nothing in the stack published one, so
 *          the safety IO could not gate the mux. This node is that adapter and nothing more:
 *          the decision itself lives in domain::isMotionInhibited().
 *
 *          Plain (non-lifecycle) node on purpose - it owns no hardware resource, only a
 *          subscription and a timer.
 *
 *          Two failure modes are handled explicitly, both by asserting the lock:
 *            - no gpio_state received yet (startup), and
 *            - gpio_state gone stale beyond `gpio_timeout` (hardware interface died).
 *          The lock is republished on a timer rather than on message arrival, because twist_mux
 *          also treats a stale lock *topic* as locked; a silent node must never be mistaken for
 *          a permissive one.
 */
class MotionLockNode : public rclcpp::Node
{
public:
    /** @throws rclcpp::exceptions::InvalidParameterValueException on an invalid override. */
    explicit MotionLockNode(
        const std::string & node_name,
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
    void gpioStateCallback(const rover_msgs::msg::GpioState::SharedPtr msg);

    void timerCallback();

    /** @brief Current lock state, accounting for missing and stale gpio_state. */
    bool evaluateLock();

    std::shared_ptr<motion_lock::ParamListener> param_listener_;

    std::optional<domain::SafetyIoFlags> flags_;

    rclcpp::Time last_gpio_stamp_;

    // Latches the transition so a held lock does not spam the log at publish_frequency.
    std::optional<bool> last_logged_lock_;

    rclcpp::Subscription<rover_msgs::msg::GpioState>::SharedPtr gpio_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_lock_pub_;

    rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace rover_twist_mux

#endif  // ROVER_TWIST_MUX_INFRASTRUCTURE_MOTION_LOCK_NODE_HPP_
