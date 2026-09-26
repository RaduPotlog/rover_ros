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

#include <algorithm>
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

#include "rover_msgs/msg/led_animation_catalog.hpp"
#include "rover_msgs/msg/led_animation_info.hpp"
#include "rover_msgs/msg/led_layer_state.hpp"
#include "rover_msgs/msg/led_segment_state.hpp"
#include "rover_msgs/msg/led_state.hpp"
#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_led/application/led_types.hpp"
#include "rover_led/application/validate_animation_catalog_use_case.hpp"
#include "rover_led/domain/led_components/led_panel.hpp"
#include "rover_led/domain/led_components/led_segment.hpp"
#include "rover_led/infrastructure/shutdown_safe_publish.hpp"
#include "rover_led/infrastructure/yaml_led_config.hpp"
#include "rover_led/led_controller_parameters.hpp"
#include "rover_utils/ros_utils.hpp"
#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

LedControllerNode::LedControllerNode(const rclcpp::NodeOptions & options)
: Node("rover_led_controller", options)
, animation_factory_(std::make_shared<PluginlibAnimationFactory>())
, shutdown_gate_(this->get_node_base_interface()->get_context(), [this]() { stopTimers(); })
{
    RCLCPP_INFO(this->get_logger(), "Initializing.");

    using namespace std::placeholders;

    this->param_listener_ =
        std::make_shared<led_controller::ParamListener>(this->get_node_parameters_interface());
    this->params_ = this->param_listener_->get_params();

    const float controller_freq = static_cast<float>(this->params_.controller_frequency);
    if (this->params_.preview_publish_rate > 0.0) {
        preview_period_ = std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double>(1.0 / this->params_.preview_publish_rate));
    }
    const YAML::Node led_config_desc = YAML::LoadFile(this->params_.animations_config_path);

    const auto layout = parseLedLayout(led_config_desc);

    PanelMap panels;

    for (const auto & panel : layout.panels) {
        panels.emplace(panel.channel, std::make_shared<LedPanel>(panel.number_of_leds, panel.rows));
        panel_publishers_.emplace(
            panel.channel,
            this->create_publisher<ImageMsg>("led/channel_" + std::to_string(panel.channel) + "_frame", 10));
        if (this->params_.preview_publish_rate > 0.0) {
            // For UIs: a web page redraws a few times a second, and the full-rate frame would
            // cross the Zenoh router and the websocket 50 times a second per panel.
            preview_publishers_.emplace(
                panel.channel,
                this->create_publisher<ImageMsg>(
                    "led/channel_" + std::to_string(panel.channel) + "_preview", rclcpp::QoS(1).best_effort()));
        }

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
    diagnostics_.catalog_warnings = catalog->warnings().size();

    const auto animations = catalog->getAll();
    diagnostics_.animations_loaded = animations.size();

    const auto validation = ValidateAnimationCatalogUseCase(animation_factory_).execute(animations);
    for (const auto & unavailable : validation.unavailable_types) {
        RCLCPP_WARN(
            this->get_logger(), "Animation '%s' (id %zu) uses unavailable type '%s'; it can't be displayed.",
            unavailable.name.c_str(), unavailable.id, unavailable.type.c_str());
    }
    diagnostics_.unavailable_animations = validation.unavailable_animations;

    RCLCPP_INFO(this->get_logger(), "Loaded default animations.");

    set_animation_use_case_ = std::make_unique<SetAnimationUseCase>(
        catalog, animation_factory_, segments, controller_freq);
    stop_animation_use_case_ = std::make_unique<StopAnimationUseCase>(catalog, segments);
    render_tick_use_case_ = std::make_unique<RenderTickUseCase>(segments, panels);
    get_led_state_use_case_ = std::make_unique<GetLedStateUseCase>(segments);

    // Latched, so late subscribers (e.g. rosbridge) get the last value.
    const auto latched_qos = rclcpp::QoS(1).reliable().transient_local();

    animation_catalog_publisher_ =
        this->create_publisher<LedAnimationCatalogMsg>("led/animations", latched_qos);
    state_publisher_ = this->create_publisher<LedStateMsg>("led/state", latched_qos);

    publishAnimationCatalog(animations);

    set_led_animation_server_ = this->create_service<SetLedAnimationSrv>(
        "led/set_animation", std::bind(&LedControllerNode::setLedAnimationCallback, this, _1, _2));
    stop_led_animation_server_ = this->create_service<StopLedAnimationSrv>(
        "led/stop_animation", std::bind(&LedControllerNode::stopLedAnimationCallback, this, _1, _2));

    controller_timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / controller_freq)),
        std::bind(&LedControllerNode::controllerTimerCallback, this));

    // Same (default, mutually exclusive) callback group as the render timer
    // and the service, so the segments are never read while they change.
    state_timer_ = this->create_wall_timer(
        std::chrono::microseconds(static_cast<std::uint64_t>(1e6 / this->params_.state_publish_rate)),
        std::bind(&LedControllerNode::stateTimerCallback, this));

    render_min_hz_ = this->params_.controller_frequency;
    render_max_hz_ = this->params_.controller_frequency;
    render_rate_ = std::make_unique<diagnostic_updater::FrequencyStatus>(
        diagnostic_updater::FrequencyStatusParam(&render_min_hz_, &render_max_hz_, 0.1, 10),
        "Led render rate", this->get_clock());

    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("Bumper Led");
    diagnostic_updater_->add("Led controller status", this, &LedControllerNode::diagnoseController);
    diagnostic_updater_->add(*render_rate_);

    RCLCPP_INFO(this->get_logger(), "Initialized successfully.");
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
            diagnostics_.last_rejected_request = response->message;
        }
    } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
        diagnostics_.last_rejected_request =
            "id " + std::to_string(request->animation.id) + ": " + response->message;
    }
}

void LedControllerNode::stopLedAnimationCallback(
    const StopLedAnimationSrv::Request::SharedPtr & request,
    StopLedAnimationSrv::Response::SharedPtr response)
{
    try {
        const auto result = stop_animation_use_case_->execute(request->id);

        if (result.stopped_segments.empty()) {
            response->success = false;
            response->message = "'" + result.name + "' is not playing.";
            return;
        }

        std::string segments;

        for (const auto & segment : result.stopped_segments) {
            segments += (segments.empty() ? "" : ", ") + segment;
        }

        response->success = true;
        response->message = "Stopped '" + result.name + "' on " + segments + ".";
    } catch (const std::exception & e) {
        response->success = false;
        response->message = e.what();
    }
}

// One image row per serpentine row of the panel; data stays in wire order.
void LedControllerNode::publishPanelFrame(
    const std::size_t channel, std::vector<std::uint8_t> frame, const std::size_t rows, const bool preview_due)
{
    const auto leds_per_row = frame.size() / 4 / rows;

    ImageMsg::UniquePtr image(new ImageMsg);
    image->header.frame_id = rover_utils::ros::addNamespaceToFrameID(
        "led_channel_" + std::to_string(channel) + "_link", std::string(this->get_namespace()));
    image->header.stamp = this->get_clock()->now();
    image->encoding = "rgba8";
    image->height = rows;
    image->width = leds_per_row;
    image->step = leds_per_row * 4;
    image->data = std::move(frame);

    if (preview_due) {
        const auto preview = preview_publishers_.find(channel);
        if (preview != preview_publishers_.end()) {
            publishUnlessShutdown(
                shutdown_gate_, this->get_logger(), preview->second, std::make_unique<ImageMsg>(*image));
        }
    }

    publishUnlessShutdown(shutdown_gate_, this->get_logger(), panel_publishers_.at(channel), std::move(image));
}

void LedControllerNode::controllerTimerCallback()
{
    auto result = render_tick_use_case_->execute();

    for (const auto & error : result.segment_errors) {
        RCLCPP_WARN_STREAM(this->get_logger(), "Failed to update animation on segment " << error);
    }
    diagnostics_.segment_errors = result.segment_errors.size();
    diagnostics_.render_error = result.error;

    if (result.error) {
        RCLCPP_ERROR(this->get_logger(), "%s", result.error->c_str());
        return;
    }

    render_rate_->tick();

    // One decision per tick, so every panel's preview shows the same render.
    bool preview_due = false;
    if (!preview_publishers_.empty()) {
        const auto now = std::chrono::steady_clock::now();
        if (!last_preview_ || now - *last_preview_ >= preview_period_) {
            last_preview_ = now;
            preview_due = true;
        }
    }

    for (auto & [channel, frame] : result.frames) {
        publishPanelFrame(channel, std::move(frame), result.rows.at(channel), preview_due);
    }
}

void LedControllerNode::publishAnimationCatalog(const std::vector<LedAnimationDescription> & animations)
{
    LedAnimationCatalogMsg msg;

    for (const auto & animation : animations) {
        rover_msgs::msg::LedAnimationInfo info;
        info.id = static_cast<std::uint16_t>(animation.id);
        info.name = animation.name;
        info.priority = animation.priority;
        msg.animations.push_back(std::move(info));
    }

    std::sort(msg.animations.begin(), msg.animations.end(), [](const auto & a, const auto & b) {
        return a.id < b.id;
    });

    animation_catalog_publisher_->publish(msg);
}

void LedControllerNode::stateTimerCallback()
{
    const auto snapshot = get_led_state_use_case_->execute();

    LedStateMsg msg;
    msg.header.stamp = this->get_clock()->now();

    for (const auto & segment : snapshot.segments) {
        rover_msgs::msg::LedSegmentState segment_msg;
        segment_msg.name = segment.name;
        segment_msg.channel = static_cast<std::uint16_t>(segment.channel);

        for (const auto & layer : segment.layers) {
            rover_msgs::msg::LedLayerState layer_msg;
            layer_msg.priority = static_cast<std::uint8_t>(layer.priority);
            layer_msg.active = layer.status.has_value();

            if (layer.status) {
                layer_msg.id = static_cast<std::uint16_t>(layer.status->info.id);
                layer_msg.name = layer.status->info.name;
                layer_msg.param = layer.status->info.param;
                layer_msg.repeating = layer.status->repeating;
                layer_msg.progress = layer.status->progress;
                layer_msg.queued = static_cast<std::uint8_t>(layer.status->queued);
            }

            segment_msg.layers.push_back(std::move(layer_msg));
        }

        msg.segments.push_back(std::move(segment_msg));
    }

    publishUnlessShutdown(shutdown_gate_, this->get_logger(), state_publisher_, msg);
}

void LedControllerNode::stopTimers()
{
    // Stop rendering at the source; the driver then has no frames to forward.
    if (controller_timer_) {
        controller_timer_->cancel();
    }

    if (state_timer_) {
        state_timer_->cancel();
    }
}

void LedControllerNode::diagnoseController(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    fillLedControllerStatus(get_led_state_use_case_->execute(), diagnostics_, status);
}

}  // namespace rover_led

#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(rover_led::LedControllerNode)
