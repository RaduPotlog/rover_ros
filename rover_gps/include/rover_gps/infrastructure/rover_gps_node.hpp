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

#ifndef ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_NODE_HPP_
#define ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_NODE_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

#include "rover_gps/application/align_heading_use_case.hpp"
#include "rover_gps/application/monitor_gps_use_case.hpp"
#include "rover_gps/domain/gps_health_evaluator.hpp"
#include "rover_gps/domain/heading_alignment_estimator.hpp"
#include "rover_gps/infrastructure/gps_msg_conversions.hpp"

namespace rover_gps
{

/**
 * @brief Composition root: GNSS health diagnostics and the odom->ENU heading alignment.
 * @details Plain (non-lifecycle) node on purpose — the NMEA socket is owned by
 *          nmea_navsat_driver; this node only consumes gps/fix and odom.
 */
class RoverGpsNode : public rclcpp::Node
{
public:
    RoverGpsNode(
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

    domain::GpsHealthThresholds health_thresholds_;
    domain::HeadingAlignmentConfig alignment_config_;
    application::AlignHeadingSettings heading_settings_;
    std::string heading_frame_id_;

    std::unique_ptr<application::MonitorGpsUseCase> monitor_gps_;
    std::unique_ptr<application::AlignHeadingUseCase> align_heading_;

    rclcpp::Subscription<infrastructure::NavSatFixMsg>::SharedPtr fix_subscriber_;
    rclcpp::Subscription<infrastructure::OdometryMsg>::SharedPtr odometry_subscriber_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_alignment_service_;
    rclcpp::TimerBase::SharedPtr tick_timer_;

    // Declared last so it is destroyed first: it holds raw pointers to the publishers' diagnostic
    // callbacks, and the publishers are owned by the use cases.
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_gps

#endif  // ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_NODE_HPP_
