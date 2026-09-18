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

#ifndef ROVER_GPS_HEADING_INFRASTRUCTURE_ROS2_HEADING_PUBLISHER_HPP_
#define ROVER_GPS_HEADING_INFRASTRUCTURE_ROS2_HEADING_PUBLISHER_HPP_

#include <memory>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rover_gps_heading/domain/heading_alignment_estimator.hpp"
#include "rover_gps_heading/domain/ports/heading_publisher_port.hpp"
#include "rover_gps_heading/infrastructure/gps_msg_conversions.hpp"

namespace rover_gps_heading::infrastructure
{

/**
 * @brief Publishes the ENU heading on gps/heading_imu and reports the alignment as the
 *        "Heading alignment" diagnostic task.
 */
class Ros2HeadingPublisher : public domain::HeadingPublisherPort
{
public:
    Ros2HeadingPublisher(
        rclcpp::Node & node,
        const std::shared_ptr<diagnostic_updater::Updater> & diagnostic_updater,
        std::string frame_id, bool heading_output_enabled);

    void publishHeading(const domain::EnuHeading & heading) override;

    void publishAlignmentStatus(const domain::AlignmentStatus & status) override;

private:
    void diagnose(diagnostic_updater::DiagnosticStatusWrapper & status);

    rclcpp::Logger logger_;
    rclcpp::Clock::SharedPtr clock_;
    rclcpp::Publisher<ImuMsg>::SharedPtr heading_pub_;

    std::string frame_id_;
    bool heading_output_enabled_;
    domain::AlignmentStatus status_;
    bool announced_alignment_{false};
};

}  // namespace rover_gps_heading::infrastructure

#endif  // ROVER_GPS_HEADING_INFRASTRUCTURE_ROS2_HEADING_PUBLISHER_HPP_
