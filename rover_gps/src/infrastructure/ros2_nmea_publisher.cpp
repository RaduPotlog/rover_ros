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

#include "rover_gps/infrastructure/ros2_nmea_publisher.hpp"

#include <utility>

namespace rover_gps::infrastructure
{

Ros2NmeaPublisher::Ros2NmeaPublisher(
    rclcpp_lifecycle::LifecycleNode & node, std::string frame_id, std::string time_ref_source)
: frame_id_(std::move(frame_id))
, time_ref_source_(std::move(time_ref_source))
{
    const rclcpp::QoS qos(10);

    fix_pub_ = node.create_publisher<NavSatFixMsg>("fix", qos);
    velocity_pub_ = node.create_publisher<TwistStampedMsg>("vel", qos);
    heading_pub_ = node.create_publisher<QuaternionStampedMsg>("heading", qos);
    time_reference_pub_ = node.create_publisher<TimeReferenceMsg>("time_reference", qos);
}

void Ros2NmeaPublisher::activate()
{
    fix_pub_->on_activate();
    velocity_pub_->on_activate();
    heading_pub_->on_activate();
    time_reference_pub_->on_activate();
}

void Ros2NmeaPublisher::deactivate()
{
    fix_pub_->on_deactivate();
    velocity_pub_->on_deactivate();
    heading_pub_->on_deactivate();
    time_reference_pub_->on_deactivate();
}

void Ros2NmeaPublisher::publishFix(const domain::nmea::FixOutput & fix, double stamp_s)
{
    fix_pub_->publish(toNavSatFixMsg(fix, frame_id_, toRosTime(stamp_s)));
}

void Ros2NmeaPublisher::publishVelocity(
    const domain::nmea::VelocityOutput & velocity, double stamp_s)
{
    velocity_pub_->publish(toTwistStampedMsg(velocity, frame_id_, toRosTime(stamp_s)));
}

void Ros2NmeaPublisher::publishHeading(const domain::nmea::HeadingOutput & heading, double stamp_s)
{
    heading_pub_->publish(toQuaternionStampedMsg(heading, frame_id_, toRosTime(stamp_s)));
}

void Ros2NmeaPublisher::publishTimeReference(
    const domain::nmea::TimeRefOutput & time_ref, double stamp_s)
{
    time_reference_pub_->publish(
        toTimeReferenceMsg(time_ref, frame_id_, time_ref_source_, toRosTime(stamp_s)));
}

}  // namespace rover_gps::infrastructure
