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

#include "rover_rs16_lidar/domain/lidar_settings.hpp"

#include <cmath>
#include <stdexcept>
#include <string>

namespace rover_rs16_lidar::domain
{

namespace
{

void require(bool condition, const std::string & message)
{
    if (!condition) {
        throw std::invalid_argument(message);
    }
}

}  // namespace

LidarInputType parseInputType(const std::string & value)
{
    if (value == "lidar") {
        return LidarInputType::Lidar;
    }
    if (value == "pcap") {
        return LidarInputType::Pcap;
    }
    throw std::invalid_argument("input_type must be 'lidar' or 'pcap', got '" + value + "'.");
}

void LidarSettings::validate(const LidarSettings & settings)
{
    require(!settings.frame_id.empty(), "frame_id must not be empty.");
    require(!settings.point_cloud_topic.empty(), "point_cloud_topic must not be empty.");
    require(settings.publisher_queue_size > 0, "publisher_queue_size must be positive.");

    const auto & sensor = settings.sensor;
    require(sensor.msop_port != 0, "msop_port must not be 0.");
    require(sensor.difop_port != 0, "difop_port must not be 0.");
    require(sensor.msop_port != sensor.difop_port, "msop_port and difop_port must differ.");
    require(sensor.min_distance >= 0.0F, "min_distance must not be negative.");
    require(
        sensor.min_distance < sensor.max_distance, "min_distance must be below max_distance.");
    require(
        sensor.start_angle >= 0.0F && sensor.start_angle <= 360.0F,
        "start_angle must be within [0, 360] degrees.");
    require(
        sensor.end_angle >= 0.0F && sensor.end_angle <= 360.0F,
        "end_angle must be within [0, 360] degrees.");
    if (sensor.input_type == LidarInputType::Pcap) {
        require(!sensor.pcap_path.empty(), "pcap_path is required when input_type is 'pcap'.");
        require(sensor.pcap_rate > 0.0F, "pcap_rate must be positive.");
    }

    const auto & scan = settings.scan;
    if (!scan.enabled) {
        return;
    }
    require(!settings.scan_topic.empty(), "scan_topic must not be empty when publish_scan.");
    require(scan.min_height < scan.max_height, "scan min_height must be below max_height.");
    require(scan.angle_min < scan.angle_max, "scan angle_min must be below angle_max.");
    require(scan.angle_increment > 0.0, "scan angle_increment must be positive.");
    require(
        scan.angle_increment <= scan.angle_max - scan.angle_min,
        "scan angle_increment must fit inside the angle range.");
    require(scan.range_min >= 0.0, "scan range_min must not be negative.");
    require(scan.range_min < scan.range_max, "scan range_min must be below range_max.");
    require(scan.scan_time > 0.0, "scan_time must be positive.");
}

}  // namespace rover_rs16_lidar::domain
