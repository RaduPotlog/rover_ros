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

#ifndef ROVER_GPS_INFRASTRUCTURE_ROS2_NMEA_PUBLISHER_HPP_
#define ROVER_GPS_INFRASTRUCTURE_ROS2_NMEA_PUBLISHER_HPP_

#include <string>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rover_gps/domain/ports/nmea_output_port.hpp"
#include "rover_gps/infrastructure/nmea_msg_conversions.hpp"

namespace rover_gps::infrastructure
{

/**
 * @brief Publishes the driver's output on fix / vel / heading / time_reference.
 * @details Topic names are the upstream defaults on purpose: rover_gps.launch.py remaps them into
 *          the `gps` namespace, so renaming them here would silently break that launch file and
 *          every downstream subscriber.
 *          QoS is reliable, depth 10, matching the Python driver. Reliable rather than
 *          sensor-data best-effort because a reliable publisher still satisfies a best-effort
 *          subscriber, while the reverse is not true and would strand reliable consumers such as
 *          navsat_transform_node.
 *          `publish*` is called from the node's receive thread; publishers are thread-safe, and the
 *          node joins that thread before deactivating them.
 */
class Ros2NmeaPublisher : public domain::NmeaOutputPort
{
public:
    Ros2NmeaPublisher(
        rclcpp_lifecycle::LifecycleNode & node, std::string frame_id, std::string time_ref_source);

    void publishFix(const domain::nmea::FixOutput & fix, double stamp_s) override;

    void publishVelocity(const domain::nmea::VelocityOutput & velocity, double stamp_s) override;

    void publishHeading(const domain::nmea::HeadingOutput & heading, double stamp_s) override;

    void publishTimeReference(
        const domain::nmea::TimeRefOutput & time_ref, double stamp_s) override;

    /** @brief Enables the publishers; call from the node's on_activate. */
    void activate();

    /** @brief Disables the publishers; call from the node's on_deactivate. */
    void deactivate();

private:
    rclcpp_lifecycle::LifecyclePublisher<NavSatFixMsg>::SharedPtr fix_pub_;
    rclcpp_lifecycle::LifecyclePublisher<TwistStampedMsg>::SharedPtr velocity_pub_;
    rclcpp_lifecycle::LifecyclePublisher<QuaternionStampedMsg>::SharedPtr heading_pub_;
    rclcpp_lifecycle::LifecyclePublisher<TimeReferenceMsg>::SharedPtr time_reference_pub_;

    std::string frame_id_;
    std::string time_ref_source_;
};

}  // namespace rover_gps::infrastructure

#endif  // ROVER_GPS_INFRASTRUCTURE_ROS2_NMEA_PUBLISHER_HPP_
