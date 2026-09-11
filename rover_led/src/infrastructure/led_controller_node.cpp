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

#include "rover_led/infrastructure/led_controller_node.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_led/application/led_types.hpp"
#include "rover_led/domain/led_components/led_panel.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"
#include "rover_led/infrastructure/yaml_led_config.hpp"
#include "rover_led/led_controller_parameters.hpp"
#include "rover_utils/ros_utils.hpp"
#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

LedControllerNode::LedControllerNode(const rclcpp::NodeOptions & options)
: Node("led_controller", options)
, animation_factory_(std::make_shared<PluginlibAnimationFactory>())
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    using namespace std::placeholders;

    this->param_listener_ =
        std::make_shared<led_controller::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    const float controller_freq = static_cast<float>(this->params_.controller_frequency);
    const YAML::Node led_config_desc = YAML::LoadFile(this->params_.animations_config_path);

    const auto layout = parseLedLayout(led_config_desc);

    PanelMap panels;

    for (const auto & panel : layout.panels) {
        panels.emplace(panel.channel, std::make_shared<LedPanel>(panel.number_of_leds));
        panel_publishers_.emplace(
            panel.channel,
            this->create_publisher<ImageMsg>("led/channel_" + std::to_string(panel.channel) + "_frame", 10));

        RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized panel with channel no. " << panel.channel << ".");
    }

    SegmentMap segments;

    for (const auto & segment : layout.segments) {
        segments.emplace(segment.name, std::make_shared<LedSegment>(segment.config));
        RCLCPP_DEBUG_STREAM(this->get_logger(), "Initialized '" << segment.name << "' segment.");
    }

    const auto catalog = std::make_shared<YamlAnimationCatalog>(
        rover_utils::getYAMLKeyValue<YAML::Node>(led_config_desc, "led_animations"), layout.segments_map);

    for (const auto & warning : catalog->warnings()) {
        RCLCPP_WARN(this->get_logger(), "%s", warning.c_str());
    }

    checkAnimationTypes(catalog->getAll());

    RCLCPP_INFO(this->get_logger(), "Loaded default animations.");

    set_animation_use_case_ = std::make_unique<SetAnimationUseCase>(
        catalog, animation_factory_, segments, controller_freq);
    render_tick_use_case_ = std::make_unique<RenderTickUseCase>(segments, panels);

    set_led_animation_server_ = this->create_service<SetLedAnimationSrv>(
        "led/set_animation", std::bind(&LedControllerNode::setLedAnimationCallback, this, _1, _2));

    controller_timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / controller_freq)),
        std::bind(&LedControllerNode::controllerTimerCallback, this));

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
}

void LedControllerNode::checkAnimationTypes(const std::vector<LedAnimationDescription> & animations)
{
    for (const auto & led_animation : animations) {
        for (const auto & animation : led_animation.animations) {
            try {
                animation_factory_->create(animation.type);
            } catch (const std::runtime_error & e) {
                RCLCPP_WARN(
                    this->get_logger(), "Animation '%s' (id %zu) uses unavailable type '%s'; it can't be displayed.",
                    led_animation.name.c_str(), led_animation.id, animation.type.c_str());
            }
        }
    }
}

void LedControllerNode::setLedAnimationCallback(
    const SetLedAnimationSrv::Request::SharedPtr & request,
    SetLedAnimationSrv::Response::SharedPtr response)
{
    LedAnimationRequest animation_request;
    animation_request.id = request->animation.id;
    animation_request.param = request->animation.param;
    animation_request.repeating = request->repeating;

    try {
        const auto result = set_animation_use_case_->execute(animation_request);
        response->success = true;

        if (!result.rejected_segments.empty()) {
            response->message = "Animation queue full on " +
                std::to_string(result.rejected_segments.size()) + " segment(s); '" + result.name +
                "' dropped there.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
        }
    } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
    }
}

void LedControllerNode::publishPanelFrame(const std::size_t channel, std::vector<std::uint8_t> frame)
{
    const auto number_of_leds = frame.size() / 4;

    ImageMsg::UniquePtr image(new ImageMsg);
    image->header.frame_id = rover_utils::ros::addNamespaceToFrameID(
        "led_channel_" + std::to_string(channel) + "_link", std::string(this->get_namespace()));
    image->header.stamp = this->get_clock()->now();
    image->encoding = "rgba8";
    image->height = 1;
    image->width = number_of_leds;
    image->step = number_of_leds * 4;
    image->data = std::move(frame);

    panel_publishers_.at(channel)->publish(std::move(image));
}

void LedControllerNode::controllerTimerCallback()
{
    auto result = render_tick_use_case_->execute();

    for (const auto & error : result.segment_errors) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to update animation on segment " << error);
    }

    if (result.error) {
        RCLCPP_ERROR(this->get_logger(), "%s", result.error->c_str());
        return;
    }

    for (auto & [channel, frame] : result.frames) {
        publishPanelFrame(channel, std::move(frame));
    }
}

}  // namespace rover_led

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rover_led::LedControllerNode)
