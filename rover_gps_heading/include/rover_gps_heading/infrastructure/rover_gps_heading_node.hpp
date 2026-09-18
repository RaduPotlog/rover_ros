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

#ifndef ROVER_GPS_HEADING_INFRASTRUCTURE_ROVER_GPS_HEADING_NODE_HPP_
#define ROVER_GPS_HEADING_INFRASTRUCTURE_ROVER_GPS_HEADING_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rover_gps_heading/application/align_heading_use_case.hpp"
#include "rover_gps_heading/domain/heading_alignment_estimator.hpp"
#include "rover_gps_heading/infrastructure/gps_msg_conversions.hpp"

namespace rover_gps_heading
{

/**
 * @brief Composition root: the odom->ENU heading alignment from GNSS course.
 * @details Localization logic, not a sensor driver: it consumes gps/fix from the sensor payload
 *          (rover_sensors/rover_gps) and odom from rover_ekf_node, and publishes gps/heading_imu
 *          for rover_navsat_transform_node. GPS fix health is reported by rover_gps_node.
 */
class RoverGpsHeadingNode : public rclcpp::Node
{
public:
    RoverGpsHeadingNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    void init();

private:
    void fixCallback(const infrastructure::NavSatFixMsg::SharedPtr msg);

    void odometryCallback(const infrastructure::OdometryMsg::SharedPtr msg);

    void tickCallback();

    void resetAlignmentCallback(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> request,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response);

    double nowSeconds() const;

    domain::HeadingAlignmentConfig alignment_config_;
    application::AlignHeadingSettings heading_settings_;
    std::string heading_frame_id_;

    std::unique_ptr<application::AlignHeadingUseCase> align_heading_;

    rclcpp::Subscription<infrastructure::NavSatFixMsg>::SharedPtr fix_subscriber_;
    rclcpp::Subscription<infrastructure::OdometryMsg>::SharedPtr odometry_subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_alignment_service_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // Declared last so it is destroyed first: it holds a raw pointer to the publisher's
    // diagnostic callback, and the publisher is owned by the use case.
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_gps_heading

#endif  // ROVER_GPS_HEADING_INFRASTRUCTURE_ROVER_GPS_HEADING_NODE_HPP_
