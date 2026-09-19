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

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"

#include "rover_msgs/msg/gpio_state.hpp"

#include "rover_twist_mux/domain/motion_lock_health.hpp"
#include "rover_twist_mux/domain/safety_io_flags.hpp"
#include "rover_twist_mux/motion_lock_params.hpp"
#include "rover_utils/shutdown_gate.hpp"

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
 *          Both are reported on the "Motion lock" diagnostic as ERROR; a lock held by a stop
 *          condition is WARN with its reasons.
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

    /** @brief Current lock decision and its reasons, accounting for missing and stale gpio_state. */
    domain::MotionLockHealth evaluateLock();

    /** @brief "Motion lock" task: formats the decision last published, never re-evaluates. */
    void diagnoseMotionLock(diagnostic_updater::DiagnosticStatusWrapper & status);

    std::shared_ptr<motion_lock::ParamListener> param_listener_;

    std::optional<domain::SafetyIoFlags> flags_;

    rclcpp::Time last_gpio_stamp_;

    // What the timer last published; the diagnostic reports exactly this.
    std::optional<domain::MotionLockHealth> last_health_;

    // Latches the transition so a held lock does not spam the log at publish_frequency.
    std::optional<bool> last_logged_lock_;

    rclcpp::Subscription<rover_msgs::msg::GpioState>::SharedPtr gpio_state_sub_;

    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr motion_lock_pub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // Cancels timer_ once shutdown starts, before rmw_zenoh closes its session.
    rover_utils::ros::ShutdownGate shutdown_gate_;

    // Last member: its timer must not fire into a partially destroyed node.
    diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace rover_twist_mux

#endif  // ROVER_TWIST_MUX_INFRASTRUCTURE_MOTION_LOCK_NODE_HPP_
