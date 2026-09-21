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

#ifndef ROVER_LED_INFRASTRUCTURE_LED_DRIVER_NODE_HPP_
#define ROVER_LED_INFRASTRUCTURE_LED_DRIVER_NODE_HPP_

#include <cstdint>
#include <memory>
#include <chrono>
#include <string>
#include <vector>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/lifecycle_publisher.hpp"

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "udp_msgs/msg/udp_packet.hpp"

#include "rover_msgs/srv/set_led_brightness.hpp"

#include "rover_led/application/encode_frame_use_case.hpp"
#include "rover_led/application/set_brightness_use_case.hpp"
#include "rover_led/domain/sk9822_frame_encoder.hpp"
#include "rover_led/led_driver_parameters.hpp"
#include "rover_utils/shutdown_gate.hpp"

namespace rover_led
{

using ImageMsg = sensor_msgs::msg::Image;
using Float32Msg = std_msgs::msg::Float32;
using UdpPacketMsg = udp_msgs::msg::UdpPacket;
using SetBoolSrv = std_srvs::srv::SetBool;
using SetLedBrightnessSrv = rover_msgs::srv::SetLedBrightness;

// ROS adapter of the hardware side: turns led/channel_<n>_frame images into
// SK9822 UDP packets on udp_write/led_channel_<n>.
//
// Lifecycle (with the autostart parameter the node configures and activates
// itself once the executor spins):
//   configure  - read parameters, create encoders, pub/sub/services.
//   activate   - obtain LED control, clear the LEDs, start forwarding frames.
//                With led_control_handshake, control is requested from
//                hardware/led_control_enable asynchronously (bounded retries);
//                frames are ignored until it is granted.
//   deactivate - clear the LEDs, release LED control, stop forwarding.
class LedDriverNode : public rclcpp_lifecycle::LifecycleNode
{

public:

    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    LedDriverNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~LedDriverNode() override;

protected:

    CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state) override;

private:

    struct Channel
    {
        std::string name;
        std::shared_ptr<SK9822FrameEncoder> encoder;
        std::unique_ptr<EncodeFrameUseCase> encode_frame;
        rclcpp::Subscription<ImageMsg>::SharedPtr subscription;
        rclcpp_lifecycle::LifecyclePublisher<UdpPacketMsg>::SharedPtr publisher;
    };

    // Pre-shutdown: runs the lifecycle shutdown on the executor and waits for it.
    void finalizeOnExecutor();

    bool isActive() const;

    void releaseResources();

    void clearLeds();

    void publishPayload(Channel & channel, std::vector<std::uint8_t> payload);

    void frameCallback(const ImageMsg::UniquePtr & msg, Channel & channel);

    void controlTimerCallback();

    // Fire-and-forget; false if the service is unavailable.
    bool sendLedControlRequest(const bool enable);

    void ledControlResponseCallback(
        rclcpp::Client<SetBoolSrv>::SharedFutureWithRequest future,
        const std::uint64_t epoch);

    void setBrightnessCallback(
        const SetLedBrightnessSrv::Request::SharedPtr & request,
        SetLedBrightnessSrv::Response::SharedPtr response);

    // Latched on led/brightness, so dashboards show the brightness in effect.
    void publishBrightness();

    void throttledWarn(const std::string & message);

    void diagnoseLeds(diagnostic_updater::DiagnosticStatusWrapper & status);

    static constexpr unsigned kMaxControlRequestAttempts = 3;
    static constexpr double kServiceResponseTimeout = 3.0;
    static constexpr std::chrono::seconds kFinalizeTimeout{1};

    std::shared_ptr<led_driver::ParamListener> param_listener_;

    led_driver::Params params_;

    std::vector<Channel> channels_;

    std::unique_ptr<SetBrightnessUseCase> set_brightness_use_case_;

    bool led_control_granted_ = false;

    bool led_control_pending_ = false;

    bool led_control_failed_ = false;

    unsigned control_request_attempt_ = 0;

    // Bumped on every activation, deactivation and cleanup; LED control
    // replies carrying an older value are stale.
    std::uint64_t control_epoch_ = 0;

    rclcpp::Time led_control_call_time_;

    rclcpp::TimerBase::SharedPtr autostart_timer_;

    rclcpp::TimerBase::SharedPtr control_timer_;

    rclcpp::Client<SetBoolSrv>::SharedPtr enable_led_control_client_;

    rclcpp::Service<SetLedBrightnessSrv>::SharedPtr set_brightness_server_;

    rclcpp_lifecycle::LifecyclePublisher<Float32Msg>::SharedPtr brightness_publisher_;

    // After everything on_shutdown() touches, so it is unregistered first.
    rover_utils::ros::ShutdownGate shutdown_gate_;

    diagnostic_updater::Updater diagnostic_updater_;
};

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_LED_DRIVER_NODE_HPP_
