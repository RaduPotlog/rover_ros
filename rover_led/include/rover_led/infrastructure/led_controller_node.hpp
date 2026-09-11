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

#ifndef ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_
#define ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_

#include <cstddef>
#include <memory>
#include <unordered_map>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_led/application/render_tick_use_case.hpp"
#include "rover_led/application/set_animation_use_case.hpp"
#include "rover_led/infrastructure/pluginlib_animation_factory.hpp"
#include "rover_led/led_controller_parameters.hpp"

namespace rover_led
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLedAnimationSrv = rover_msgs::srv::SetLedAnimation;

// ROS adapter of the animation side: loads the LED configuration, serves
// led/set_animation and publishes one RGBA8 frame per panel on
// led/channel_<n>_frame at controller_frequency.
class LedControllerNode : public rclcpp::Node
{

public:

    LedControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~LedControllerNode() {}

private:

    void checkAnimationTypes(const std::vector<LedAnimationDescription> & animations);

    void publishPanelFrame(const std::size_t channel, std::vector<std::uint8_t> frame);

    void setLedAnimationCallback(
        const SetLedAnimationSrv::Request::SharedPtr & request,
        SetLedAnimationSrv::Response::SharedPtr response);

    void controllerTimerCallback();

    // Declared first: animations created by the factory must be destroyed
    // before its class loader.
    std::shared_ptr<PluginlibAnimationFactory> animation_factory_;

    std::unordered_map<std::size_t, rclcpp::Publisher<ImageMsg>::SharedPtr> panel_publishers_;

    std::unique_ptr<SetAnimationUseCase> set_animation_use_case_;

    std::unique_ptr<RenderTickUseCase> render_tick_use_case_;

    std::shared_ptr<led_controller::ParamListener> param_listener_;

    led_controller::Params params_;

    rclcpp::Service<SetLedAnimationSrv>::SharedPtr set_led_animation_server_;

    rclcpp::TimerBase::SharedPtr controller_timer_;
};

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_LED_CONTROLLER_NODE_HPP_
