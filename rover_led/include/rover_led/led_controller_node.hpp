#ifndef ROVER_LED_LED_CONTROLLER_NODE_HPP_
#define ROVER_LED_LED_CONTROLLER_NODE_HPP_

#include <memory>
#include <string>
#include <unordered_map>

#include "yaml-cpp/yaml.h"
#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/image.hpp"

#include "rover_msgs/srv/set_led_animation.hpp"

#include "rover_led/animation/animation.hpp"
#include "rover_led/led_components/led_animation.hpp"
#include "rover_led/led_components/segment_converter.hpp"
#include "rover_led/led_controller_parameters.hpp"
#include "rover_utils/yaml_utils.hpp"

namespace rover_led
{

using ImageMsg = sensor_msgs::msg::Image;
using SetLedAnimationSrv = rover_msgs::srv::SetLedAnimation;

class LedControllerNode : public rclcpp::Node
{

public:

    LedControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~LedControllerNode() {}

protected:
 
    void initializeLedPanels(const YAML::Node & panels_description);

    void initializeLedSegments(
        const YAML::Node & segments_description, 
        const float controller_freq);

    void initializeLedSegmentsMap(const YAML::Node & segments_map_description);

    void loadDefaultAnimations(const YAML::Node & animations_description);

    void loadUserAnimations(const std::string & user_led_animations_path);

    void loadAnimation(const YAML::Node & animation_description);

    void updateAndPublishAnimation();

    void addAnimationToLayer(
        const std::size_t animation_id, 
        const bool repeating, 
        const std::string & param);

    void setLedAnimation(const std::shared_ptr<LedAnimation> & led_animation);

private:

    void publishPanelFrame(const std::size_t channel);
    
    void setLedAnimationCallback(
        const SetLedAnimationSrv::Request::SharedPtr & request,
        SetLedAnimationSrv::Response::SharedPtr response);
  
    void controllerTimerCallback();

    std::unordered_map<std::size_t, std::shared_ptr<LedPanel>> led_panels_;
    std::unordered_map<std::size_t, rclcpp::Publisher<ImageMsg>::SharedPtr> panel_publishers_;
    std::unordered_map<std::string, std::shared_ptr<LedSegment>> segments_;
    std::unordered_map<std::string, std::vector<std::string>> segments_map_;
    std::unordered_map<std::size_t, LedAnimationDescription> animations_descriptions_;
    std::shared_ptr<SegmentConverter> segment_converter_;

    std::shared_ptr<led_controller::ParamListener> param_listener_;
    led_controller::Params params_;

    rclcpp::Service<SetLedAnimationSrv>::SharedPtr set_led_animation_server_;
    rclcpp::TimerBase::SharedPtr controller_timer_;

    bool animation_finished_ = true;
};

}  // namespace rover_led

#endif  // ROVER_LED_LED_CONTROLLER_NODE_HPP_