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

#include "rover_led/infrastructure/led_driver_node.hpp"

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <memory>
#include <future>
#include <string>
#include <utility>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_srvs/srv/set_bool.hpp"

#include "rover_msgs/srv/set_led_brightness.hpp"

#include "rover_led/infrastructure/shutdown_safe_publish.hpp"
#include "rover_led/led_driver_parameters.hpp"

namespace rover_led
{

using std::placeholders::_1;
using std::placeholders::_2;

LedDriverNode::LedDriverNode(const rclcpp::NodeOptions & options)
: LifecycleNode("rover_led_driver", options)
, shutdown_gate_(this->get_node_base_interface()->get_context(), [this]() { finalizeOnExecutor(); })
, diagnostic_updater_(this)
{
    RCLCPP_INFO(this->get_logger(), "Constructing node.");

    this->param_listener_ =
        std::make_shared<led_driver::ParamListener>(this->get_node_parameters_interface());

    diagnostic_updater_.setHardwareID("Bumper Led");
    diagnostic_updater_.add("Led driver status", this, &LedDriverNode::diagnoseLeds);

    // Self-driven rather than launch_ros ComposableLifecycleNode autostart,
    // which (Jazzy) mis-builds the node name in a namespace and breaks when
    // the node's launch condition is false.
    if (this->param_listener_->get_params().autostart) {
        autostart_timer_ = this->create_wall_timer(std::chrono::milliseconds(0), [this]() {
            autostart_timer_->cancel();

            if (this->configure().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_INACTIVE) {
                this->activate();
            }
        });
    }

    RCLCPP_INFO(this->get_logger(), "Node constructed successfully.");
}

LedDriverNode::~LedDriverNode() = default;

LedDriverNode::CallbackReturn LedDriverNode::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(this->get_logger(), "Configuring.");

    this->params_ = this->param_listener_->get_params();

    const auto frame_timeout_ns = static_cast<std::int64_t>(std::llround(this->params_.frame_timeout * 1e9));
    const auto now_ns = this->get_clock()->now().nanoseconds();

    const std::vector<std::pair<std::string, std::size_t>> channel_layout = {
        {"channel_1", static_cast<std::size_t>(this->params_.channel_1_num_led)},
        {"channel_2", static_cast<std::size_t>(this->params_.channel_2_num_led)},
    };

    channels_.clear();
    channels_.reserve(channel_layout.size());

    std::vector<std::shared_ptr<SK9822FrameEncoder>> encoders;

    for (const auto & [name, num_led] : channel_layout) {
        Channel channel;
        channel.name = name;
        channel.encoder = std::make_shared<SK9822FrameEncoder>();
        channel.encoder->setGlobalBrightness(static_cast<float>(this->params_.global_brightness));
        channel.encode_frame = std::make_unique<EncodeFrameUseCase>(
            channel.encoder, num_led, frame_timeout_ns, now_ns);
        channel.publisher = this->create_publisher<UdpPacketMsg>("udp_write/led_" + name, 5);

        encoders.push_back(channel.encoder);
        channels_.push_back(std::move(channel));
    }

    // Subscriptions capture the channel by pointer, so create them once the
    // vector is no longer resized.
    for (auto & channel : channels_) {
        Channel * channel_ptr = &channel;
        channel.subscription = this->create_subscription<ImageMsg>(
            "led/" + channel.name + "_frame", 5, [this, channel_ptr](const ImageMsg::UniquePtr & msg) {
                frameCallback(msg, *channel_ptr);
            });
    }

    set_brightness_use_case_ = std::make_unique<SetBrightnessUseCase>(encoders);

    // Default (mutually exclusive) callback group: the reply is handled
    // serially with transitions, timers and frames.
    enable_led_control_client_ = this->create_client<SetBoolSrv>(
        "hardware/led_control_enable", rclcpp::ServicesQoS());

    set_brightness_server_ = this->create_service<SetLedBrightnessSrv>(
        "led/set_brightness", std::bind(&LedDriverNode::setBrightnessCallback, this, _1, _2));

    brightness_publisher_ = this->create_publisher<Float32Msg>(
        "led/brightness", rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(this->get_logger(), "Configured.");

    return CallbackReturn::SUCCESS;
}

LedDriverNode::CallbackReturn LedDriverNode::on_activate(const rclcpp_lifecycle::State & previous_state)
{
    // Activates the lifecycle publishers.
    const auto result = LifecycleNode::on_activate(previous_state);

    if (result != CallbackReturn::SUCCESS) {
        return result;
    }

    publishBrightness();

    // Replies to requests of an earlier activation are stale from now on.
    control_epoch_++;
    led_control_pending_ = false;
    led_control_failed_ = false;
    control_request_attempt_ = 0;

    if (!this->params_.led_control_handshake) {
        led_control_granted_ = true;
        clearLeds();
        RCLCPP_INFO(this->get_logger(), "Activated.");
        return CallbackReturn::SUCCESS;
    }

    // The single-threaded container can't block here waiting for the reply,
    // so control is requested asynchronously and retried from a timer.
    led_control_granted_ = false;
    control_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100), std::bind(&LedDriverNode::controlTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Activated; requesting LED control.");

    return CallbackReturn::SUCCESS;
}

LedDriverNode::CallbackReturn LedDriverNode::on_deactivate(const rclcpp_lifecycle::State & previous_state)
{
    if (control_timer_) {
        control_timer_->cancel();
        control_timer_.reset();
    }

    control_epoch_++;
    led_control_pending_ = false;

    if (led_control_granted_) {
        clearLeds();

        if (this->params_.led_control_handshake) {
            sendLedControlRequest(false);
        }
    }

    led_control_granted_ = false;

    RCLCPP_INFO(this->get_logger(), "Deactivated.");

    // Deactivates the lifecycle publishers.
    return LifecycleNode::on_deactivate(previous_state);
}

LedDriverNode::CallbackReturn LedDriverNode::on_cleanup(const rclcpp_lifecycle::State & /*previous_state*/)
{
    releaseResources();

    RCLCPP_INFO(this->get_logger(), "Cleaned up.");

    return CallbackReturn::SUCCESS;
}

LedDriverNode::CallbackReturn LedDriverNode::on_shutdown(const rclcpp_lifecycle::State & previous_state)
{
    if (previous_state.id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE &&
        on_deactivate(previous_state) != CallbackReturn::SUCCESS) {
        RCLCPP_WARN(this->get_logger(), "Deactivation during shutdown failed.");
    }

    releaseResources();

    RCLCPP_INFO(this->get_logger(), "Shut down.");

    return CallbackReturn::SUCCESS;
}

void LedDriverNode::finalizeOnExecutor()
{
    // Runs on the thread calling rclcpp::shutdown(), while publishing still works. The
    // transition is handed to the executor so it is serialized with the frame, timer and
    // service callbacks; on_shutdown() then clears the LEDs and hands back LED control, and
    // the node is Finalized before it is destroyed.
    auto finalized = std::make_shared<std::promise<void>>();
    auto finalized_future = finalized->get_future();
    auto fired = std::make_shared<std::atomic<bool>>(false);

    const auto timer = this->create_wall_timer(std::chrono::milliseconds(0), [this, finalized, fired]() {
        if (fired->exchange(true)) {
            return;
        }

        if (this->get_current_state().id() != lifecycle_msgs::msg::State::PRIMARY_STATE_FINALIZED) {
            this->shutdown();
        }

        finalized->set_value();
    });

    // Bounded: the executor may not be spinning (or this may be called from one of its callbacks).
    if (finalized_future.wait_for(kFinalizeTimeout) != std::future_status::ready) {
        RCLCPP_WARN(this->get_logger(), "Executor did not finalize the node on shutdown; LEDs may stay lit.");
    }

    timer->cancel();
}

bool LedDriverNode::isActive() const
{
    return this->get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void LedDriverNode::releaseResources()
{
    if (control_timer_) {
        control_timer_->cancel();
        control_timer_.reset();
    }

    control_epoch_++;

    if (enable_led_control_client_) {
        enable_led_control_client_->prune_pending_requests();
    }

    set_brightness_server_.reset();
    brightness_publisher_.reset();
    enable_led_control_client_.reset();
    set_brightness_use_case_.reset();
    channels_.clear();
}

void LedDriverNode::clearLeds()
{
    for (auto & channel : channels_) {
        publishPayload(channel, channel.encode_frame->encodeBlank());
    }
}

void LedDriverNode::publishPayload(Channel & channel, std::vector<std::uint8_t> payload)
{
    UdpPacketMsg udp_msg;
    udp_msg.header.stamp = this->now();
    udp_msg.header.frame_id = "";
    udp_msg.data = std::move(payload);

    publishUnlessShutdown(shutdown_gate_, this->get_logger(), channel.publisher, udp_msg);
}

void LedDriverNode::frameCallback(const ImageMsg::UniquePtr & msg, Channel & channel)
{
    if (!isActive()) {
        throttledWarn("Driver is not active. Ignoring frame for " + channel.name + "!");
        return;
    }

    if (!led_control_granted_) {
        throttledWarn("Waiting for LED control to be granted. Ignoring frame for " + channel.name + "!");
        return;
    }

    RgbaFrame frame;
    frame.stamp_ns = rclcpp::Time(msg->header.stamp).nanoseconds();
    frame.encoding = msg->encoding;
    frame.height = msg->height;
    frame.width = msg->width;
    frame.data = std::move(msg->data);

    auto result = channel.encode_frame->execute(frame, this->get_clock()->now().nanoseconds());

    if (!result.accepted) {
        const auto warn_msg = result.error + " on " + channel.name + "!";
        throttledWarn(warn_msg);
        diagnostic_updater_.broadcast(diagnostic_msgs::msg::DiagnosticStatus::WARN, warn_msg);
        return;
    }

    publishPayload(channel, std::move(result.payload));
}

void LedDriverNode::controlTimerCallback()
{
    if (led_control_granted_ || led_control_failed_) {
        control_timer_->cancel();
        return;
    }

    if (led_control_pending_) {
        if (this->now() - led_control_call_time_ <= rclcpp::Duration::from_seconds(kServiceResponseTimeout)) {
            return;
        }

        RCLCPP_WARN(this->get_logger(), "LED control service response timeout.");
        led_control_pending_ = false;
    }

    if (control_request_attempt_ >= kMaxControlRequestAttempts) {
        led_control_failed_ = true;
        control_timer_->cancel();
        RCLCPP_ERROR(
            this->get_logger(), "Failed to obtain LED control after %u attempts; frames will be ignored.",
            kMaxControlRequestAttempts);
        return;
    }

    control_request_attempt_++;

    if (sendLedControlRequest(true)) {
        led_control_pending_ = true;
        led_control_call_time_ = this->now();
    }
}

bool LedDriverNode::sendLedControlRequest(const bool enable)
{
    if (!enable_led_control_client_ || !enable_led_control_client_->service_is_ready()) {
        RCLCPP_WARN(this->get_logger(), "Service 'hardware/led_control_enable' is not available.");
        return false;
    }

    RCLCPP_DEBUG(this->get_logger(), "Requesting LED control '%s'.", enable ? "enable" : "disable");

    auto request = std::make_shared<SetBoolSrv::Request>();
    request->data = enable;

    enable_led_control_client_->async_send_request(
        request, [this, epoch = control_epoch_](rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future) {
            ledControlResponseCallback(future, epoch);
        });

    return true;
}

void LedDriverNode::ledControlResponseCallback(
    rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future,
    const std::uint64_t epoch)
{
    const auto result = future.get();
    const auto request = result.first;
    const auto response = result.second;

    if (epoch != control_epoch_) {
        // Reply to a request of an earlier activation: never adopt it, and
        // hand back control that was granted too late.
        if (request->data && response->success) {
            RCLCPP_INFO(this->get_logger(), "Releasing LED control granted to a previous activation.");
            sendLedControlRequest(false);
        }

        return;
    }

    led_control_pending_ = false;

    if (!response->success) {
        RCLCPP_ERROR(this->get_logger(), "Failed to toggle LED control.");
        return;
    }

    if (!request->data) {
        RCLCPP_INFO(this->get_logger(), "LED control revoked.");
        return;
    }

    led_control_granted_ = true;
    clearLeds();
    RCLCPP_INFO(this->get_logger(), "LED control granted.");
}

void LedDriverNode::setBrightnessCallback(
    const SetLedBrightnessSrv::Request::SharedPtr & req,
    SetLedBrightnessSrv::Response::SharedPtr res)
{
    const float brightness = req->data;

    try {
        set_brightness_use_case_->execute(brightness);
    } catch (const std::out_of_range & e) {
        res->success = false;
        res->message = "Failed to set brightness: " + std::string(e.what());
        return;
    }

    // Kept as the parameter too, so a reconfigure doesn't fall back to the
    // brightness from the launch configuration.
    this->params_.global_brightness = brightness;
    this->set_parameter(rclcpp::Parameter("global_brightness", static_cast<double>(brightness)));
    publishBrightness();

    auto str_bright = std::to_string(brightness);
    str_bright = str_bright.substr(0, str_bright.find(".") + 3);
    res->success = true;
    res->message = "Changed brightness to " + str_bright;
}

void LedDriverNode::publishBrightness()
{
    if (!brightness_publisher_ || !brightness_publisher_->is_activated()) {
        return;
    }

    Float32Msg msg;
    msg.data = static_cast<float>(this->params_.global_brightness);
    publishUnlessShutdown(shutdown_gate_, this->get_logger(), brightness_publisher_, msg);
}

void LedDriverNode::throttledWarn(const std::string & message)
{
    RCLCPP_WARN_STREAM_THROTTLE(this->get_logger(), *this->get_clock(), 5000, message);
}

void LedDriverNode::diagnoseLeds(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    unsigned char error_level{diagnostic_updater::DiagnosticStatusWrapper::ERROR};
    std::string message{"Driver is not functional!"};
    std::string led_control_status{"NOT_GRANTED"};

    if (!isActive()) {
        message = "Driver is not active!";
    } else if (led_control_granted_) {
        error_level = diagnostic_updater::DiagnosticStatusWrapper::OK;
        message = "Driver is fully functional.";
        led_control_status = "GRANTED";
    } else if (!led_control_failed_) {
        error_level = diagnostic_updater::DiagnosticStatusWrapper::WARN;
        message = "Driver is not yet functional!";
        led_control_status = "PENDING";
    }

    status.add("Lifecycle state", this->get_current_state().label());
    status.add("LED control status", led_control_status);
    status.summary(error_level, message);
}

}  // namespace rover_led

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rover_led::LedDriverNode)
