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

#include "rover_twist_mux/infrastructure/motion_lock_node.hpp"

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rover_twist_mux/domain/motion_lock_policy.hpp"

namespace rover_twist_mux
{

namespace
{

/** @brief Maps the published GpioState onto the domain's flags, preserving pin polarity. */
domain::SafetyIoFlags toSafetyIoFlags(const rover_msgs::msg::GpioState & msg)
{
    domain::SafetyIoFlags flags;

    flags.hw_e_stop_user_button = msg.gpio_pin_hw_e_stop_user_button;
    flags.sw_e_stop_user_button = msg.gpio_pin_sw_e_stop_user_button;
    flags.sw_e_stop_motor_driver_fault = msg.gpio_pin_sw_e_stop_motor_driver_fault;
    flags.sw_e_stop_latch_status = msg.gpio_pin_sw_e_stop_latch_status;
    flags.motor_contactor_engaged = msg.gpio_pin_motor_contactor_engaged;

    return flags;
}

domain::MotionLockPolicy toPolicy(const motion_lock::Params & params)
{
    domain::MotionLockPolicy policy;

    policy.use_hw_e_stop_user_button = params.use_hw_e_stop_user_button;
    policy.use_sw_e_stop_user_button = params.use_sw_e_stop_user_button;
    policy.use_sw_e_stop_motor_driver_fault = params.use_sw_e_stop_motor_driver_fault;
    policy.use_sw_e_stop_latch_status = params.use_sw_e_stop_latch_status;
    policy.require_motor_contactor_engaged = params.require_motor_contactor_engaged;

    return policy;
}

unsigned char toDiagnosticLevel(domain::HealthLevel level)
{
    using diagnostic_msgs::msg::DiagnosticStatus;

    switch (level) {
        case domain::HealthLevel::Error: return DiagnosticStatus::ERROR;
        case domain::HealthLevel::Warn: return DiagnosticStatus::WARN;
        case domain::HealthLevel::Ok:
        default: return DiagnosticStatus::OK;
    }
}

}  // namespace

MotionLockNode::MotionLockNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp::Node(node_name, options)
, last_gpio_stamp_(0, 0, this->get_clock()->get_clock_type())
, shutdown_gate_(this->get_node_base_interface()->get_context(), [this]() {
    if (timer_) {
        timer_->cancel();
    }
})
, diagnostic_updater_(this)
{
    param_listener_ = std::make_shared<motion_lock::ParamListener>(
        this->get_node_parameters_interface());

    const auto params = param_listener_->get_params();

    // Matches the publisher in rover_hardware_interface SystemROSInterface: KeepLast(1),
    // transient_local, reliable. Transient-local matters here - the hardware interface publishes
    // gpio_state on change, so a late-joining subscriber would otherwise wait for the next edge.
    gpio_state_sub_ = this->create_subscription<rover_msgs::msg::GpioState>(
        "hardware_interface/gpio_state",
        rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable(),
        std::bind(&MotionLockNode::gpioStateCallback, this, std::placeholders::_1));

    // twist_mux subscribes its lock topics with SystemDefaultsQoS (reliable, volatile), which a
    // reliable volatile publisher satisfies.
    motion_lock_pub_ = this->create_publisher<std_msgs::msg::Bool>(
        "motion_lock", rclcpp::QoS(rclcpp::KeepLast(1)).reliable());

    const auto period = std::chrono::duration<double>(1.0 / params.publish_frequency);
    timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&MotionLockNode::timerCallback, this));

    diagnostic_updater_.setHardwareID("Motion Lock");
    diagnostic_updater_.add("Motion lock", this, &MotionLockNode::diagnoseMotionLock);

    RCLCPP_INFO(
        this->get_logger(),
        "Motion lock publishing on '%s' at %.1f Hz; locked until gpio_state arrives.",
        motion_lock_pub_->get_topic_name(), params.publish_frequency);
}

void MotionLockNode::gpioStateCallback(const rover_msgs::msg::GpioState::SharedPtr msg)
{
    flags_ = toSafetyIoFlags(*msg);
    last_gpio_stamp_ = this->now();
}

domain::MotionLockHealth MotionLockNode::evaluateLock()
{
    const auto params = param_listener_->get_params();

    const double age_s = flags_.has_value() ? (this->now() - last_gpio_stamp_).seconds() : 0.0;

    return domain::evaluateMotionLockHealth(flags_, age_s, params.gpio_timeout, toPolicy(params));
}

void MotionLockNode::timerCallback()
{
    last_health_ = evaluateLock();
    const bool locked = last_health_->locked;

    std_msgs::msg::Bool msg;
    msg.data = locked;
    motion_lock_pub_->publish(msg);

    if (!last_logged_lock_.has_value() || *last_logged_lock_ != locked) {
        if (locked) {
            RCLCPP_WARN(
                this->get_logger(), "Motion LOCKED: velocity commands are inhibited (%s)",
                last_health_->message.c_str());
        } else {
            RCLCPP_INFO(this->get_logger(), "Motion unlocked: velocity commands are permitted.");
        }

        last_logged_lock_ = locked;
    }
}

void MotionLockNode::diagnoseMotionLock(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!last_health_.has_value()) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::STALE, "Lock not evaluated yet.");
        return;
    }

    const auto params = param_listener_->get_params();

    status.add("Locked", last_health_->locked);

    if (flags_.has_value()) {
        status.add("gpio_state age (s)", (this->now() - last_gpio_stamp_).seconds());
        status.add("HW E-Stop user button", flags_->hw_e_stop_user_button);
        status.add("SW E-Stop user button", flags_->sw_e_stop_user_button);
        status.add("SW E-Stop motor driver fault", flags_->sw_e_stop_motor_driver_fault);
        status.add("SW E-Stop latch status", flags_->sw_e_stop_latch_status);
        status.add("Motor contactor engaged", flags_->motor_contactor_engaged);
    }

    status.add("gpio_state timeout (s)", params.gpio_timeout);
    status.add("Policy: use_hw_e_stop_user_button", params.use_hw_e_stop_user_button);
    status.add("Policy: use_sw_e_stop_user_button", params.use_sw_e_stop_user_button);
    status.add("Policy: use_sw_e_stop_motor_driver_fault", params.use_sw_e_stop_motor_driver_fault);
    status.add("Policy: use_sw_e_stop_latch_status", params.use_sw_e_stop_latch_status);
    status.add("Policy: require_motor_contactor_engaged", params.require_motor_contactor_engaged);

    status.summary(toDiagnosticLevel(last_health_->level), last_health_->message);
}

}  // namespace rover_twist_mux
