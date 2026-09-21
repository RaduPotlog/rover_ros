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

#ifndef ROVER_UTILS_ROVER_UTILS_SHUTDOWN_GATE_HPP_
#define ROVER_UTILS_ROVER_UTILS_SHUTDOWN_GATE_HPP_

#include <atomic>
#include <functional>
#include <utility>

#include "rclcpp/context.hpp"
#include "rclcpp/utilities.hpp"

namespace rover_utils::ros
{

// Closes as soon as shutdown of the context starts, before the middleware goes away.
//
// rcl_shutdown() shuts the rmw down first and only then invalidates the context, so for the
// whole time rmw_zenoh needs to close its session (~2 s) rclcpp::ok() is still true while every
// publish fails. Pre-shutdown callbacks run before that window, so publishing is gated here.
//
// on_close runs first - while publishing still works - and the gate closes after it returns.
// It runs on the thread calling rclcpp::shutdown(), not on the executor.
//
// Declare it after the timers/publishers on_close touches, so it is destroyed (and its callback
// unregistered) before them.
class ShutdownGate
{
public:
    explicit ShutdownGate(rclcpp::Context::SharedPtr context, std::function<void()> on_close = {})
    : context_(std::move(context))
    , on_close_(std::move(on_close))
    {
        handle_ = context_->add_pre_shutdown_callback([this]() {
            if (on_close_) {
                on_close_();
            }

            closed_ = true;
        });
    }

    ~ShutdownGate() { context_->remove_pre_shutdown_callback(handle_); }

    ShutdownGate(const ShutdownGate &) = delete;
    ShutdownGate & operator=(const ShutdownGate &) = delete;

    bool isOpen() const { return !closed_ && rclcpp::ok(context_); }

private:
    rclcpp::Context::SharedPtr context_;
    std::function<void()> on_close_;
    std::atomic<bool> closed_{false};
    rclcpp::PreShutdownCallbackHandle handle_;
};

}  // namespace rover_utils::ros

#endif  // ROVER_UTILS_ROVER_UTILS_SHUTDOWN_GATE_HPP_
