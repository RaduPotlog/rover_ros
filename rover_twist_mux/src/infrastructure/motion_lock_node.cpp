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

#include <algorithm>
#include <optional>

#include <chrono>
#include <memory>
#include <string>

#include "diagnostic_msgs/msg/diagnostic_status.hpp"

#include "rover_twist_mux/domain/motion_lock_policy.hpp"

namespace rover_twist_mux
{

namespace
{

/**
 * @brief Maps the two safety messages onto the domain's flags, preserving pin polarity.
 * @details Which field comes from which topic is the point of the split: the plant readings come
 *          from SafetyStatus, the two `sw_*` stop requests are read-backs of coils this system
 *          drives and come from SafetyCommandEcho. Reading a request as a reason to inhibit is
 *          sound and is what this package does; reading one as evidence of plant state is not.
 */
domain::SafetyIoFlags toSafetyIoFlags(
    const rover_msgs::msg::SafetyStatus & status,
    const rover_msgs::msg::SafetyCommandEcho & echo)
{
    domain::SafetyIoFlags flags;

    flags.hw_e_stop_user_button = status.hw_e_stop_user_button;
    flags.sw_e_stop_latch_status = status.latch_active;
    flags.motor_contactor_engaged = status.motor_contactor_engaged;

    flags.sw_e_stop_user_button = echo.sw_e_stop_user_button;
    flags.sw_e_stop_motor_driver_fault = echo.sw_e_stop_motor_driver_fault;

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
, last_status_stamp_(0, 0, this->get_clock()->get_clock_type())
, last_echo_stamp_(0, 0, this->get_clock()->get_clock_type())
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
    // reliable, volatile. Volatile, not transient_local: these are periodic 20 Hz status streams,
    // and a latched last sample told late joiners what was true when the publisher last ran
    // rather than that it is still running - which is what led two other consumers to skip their
    // staleness checks entirely. This node times both topics out instead.
    const auto safety_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable().durability_volatile();

    safety_status_sub_ = this->create_subscription<rover_msgs::msg::SafetyStatus>(
        "hardware_interface/safety_status", safety_qos,
        std::bind(&MotionLockNode::safetyStatusCallback, this, std::placeholders::_1));

    safety_command_echo_sub_ = this->create_subscription<rover_msgs::msg::SafetyCommandEcho>(
        "hardware_interface/safety_command_echo", safety_qos,
        std::bind(&MotionLockNode::safetyCommandEchoCallback, this, std::placeholders::_1));

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
        "Motion lock publishing on '%s' at %.1f Hz; locked until safety state arrives.",
        motion_lock_pub_->get_topic_name(), params.publish_frequency);
}

void MotionLockNode::safetyStatusCallback(const rover_msgs::msg::SafetyStatus::SharedPtr msg)
{
    last_status_ = *msg;
    last_status_stamp_ = this->now();
}

void MotionLockNode::safetyCommandEchoCallback(
    const rover_msgs::msg::SafetyCommandEcho::SharedPtr msg)
{
    last_echo_ = *msg;
    last_echo_stamp_ = this->now();
}

domain::MotionLockHealth MotionLockNode::evaluateLock()
{
    const auto params = param_listener_->get_params();

    // Both halves are required: acting on one alone would silently read the missing half's stop
    // conditions as "not active", which is the fail-open this node exists to prevent.
    std::optional<domain::SafetyIoFlags> flags;

    if (last_status_.has_value() && last_echo_.has_value()) {
        flags = toSafetyIoFlags(*last_status_, *last_echo_);
    }

    // The older of the two ages, so neither topic going quiet on its own can hide behind the
    // other still arriving.
    const double age_s = flags.has_value()
        ? std::max(
              (this->now() - last_status_stamp_).seconds(),
              (this->now() - last_echo_stamp_).seconds())
        : 0.0;

    const bool link_healthy = last_status_.has_value() && last_status_->link_healthy;

    return domain::evaluateMotionLockHealth(
        flags, age_s, params.gpio_timeout, toPolicy(params), link_healthy);
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

    if (last_status_.has_value()) {
        status.add("safety_status age (s)", (this->now() - last_status_stamp_).seconds());
        status.add("Safety PLC link healthy", last_status_->link_healthy);
        status.add("HW E-Stop user button", last_status_->hw_e_stop_user_button);
        status.add("SW E-Stop latch status", last_status_->latch_active);
        status.add("Motor contactor engaged", last_status_->motor_contactor_engaged);
    }

    if (last_echo_.has_value()) {
        status.add("safety_command_echo age (s)", (this->now() - last_echo_stamp_).seconds());
        status.add("SW E-Stop user button", last_echo_->sw_e_stop_user_button);
        status.add("SW E-Stop motor driver fault", last_echo_->sw_e_stop_motor_driver_fault);
    }

    status.add("Safety state timeout (s)", params.gpio_timeout);
    status.add("Policy: use_hw_e_stop_user_button", params.use_hw_e_stop_user_button);
    status.add("Policy: use_sw_e_stop_user_button", params.use_sw_e_stop_user_button);
    status.add("Policy: use_sw_e_stop_motor_driver_fault", params.use_sw_e_stop_motor_driver_fault);
    status.add("Policy: use_sw_e_stop_latch_status", params.use_sw_e_stop_latch_status);
    status.add("Policy: require_motor_contactor_engaged", params.require_motor_contactor_engaged);

    status.summary(toDiagnosticLevel(last_health_->level), last_health_->message);
}

}  // namespace rover_twist_mux
