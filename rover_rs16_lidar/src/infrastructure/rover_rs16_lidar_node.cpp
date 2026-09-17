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

#include "rover_rs16_lidar/infrastructure/rover_rs16_lidar_node.hpp"

#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

#include "rover_rs16_lidar/domain/scan_projector.hpp"
#include "rover_rs16_lidar/infrastructure/ros2_laser_scan_publisher.hpp"
#include "rover_rs16_lidar/infrastructure/ros2_lidar_health_publisher.hpp"
#include "rover_rs16_lidar/infrastructure/ros2_point_cloud_publisher.hpp"
#include "rover_rs16_lidar/infrastructure/rs_driver_lidar_source.hpp"

namespace rover_rs16_lidar
{

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeRange(
    const std::string & description, double from_value, double to_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = from_value;
    descriptor.floating_point_range[0].to_value = to_value;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    return describeRange(description, 1.0e-6, max_value);
}

rcl_interfaces::msg::ParameterDescriptor describeIntRange(
    const std::string & description, std::int64_t from_value, std::int64_t to_value)
{
    auto descriptor = describe(description);
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = from_value;
    descriptor.integer_range[0].to_value = to_value;
    return descriptor;
}

}  // namespace

RoverRs16LidarNode::RoverRs16LidarNode(
    const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    declareParameters();

    // Reject inconsistent configuration at startup, not at the first cloud.
    domain::LidarSettings::validate(settings_);
    domain::LidarHealthEvaluator::validate(health_thresholds_);
}

RoverRs16LidarNode::~RoverRs16LidarNode()
{
    // The source runs its own thread and calls back into the publishers, so it has to be
    // stopped before any of them is destroyed.
    if (source_) {
        source_->stop();
    }
}

void RoverRs16LidarNode::declareParameters()
{
    const domain::LidarSettings setting_defaults;
    const domain::LidarHealthThresholds health_defaults;

    settings_.frame_id = declare_parameter(
        "frame_id", std::string("lidar_link"),
        describe("TF frame the cloud and scan are published in; must match rover_description."));
    settings_.point_cloud_topic = declare_parameter(
        "point_cloud_topic", setting_defaults.point_cloud_topic,
        describe("Topic for the sensor_msgs/PointCloud2 output."));
    settings_.scan_topic = declare_parameter(
        "scan_topic", setting_defaults.scan_topic,
        describe("Topic for the flattened sensor_msgs/LaserScan Nav 2 costmaps consume."));
    settings_.scan.enabled = declare_parameter(
        "publish_scan", setting_defaults.scan.enabled,
        describe("False publishes the point cloud only."));
    settings_.publisher_queue_size = static_cast<std::size_t>(declare_parameter<std::int64_t>(
        "publisher_queue_size", static_cast<std::int64_t>(setting_defaults.publisher_queue_size),
        describeIntRange("Publisher queue depth. A PointCloud2 is megabytes - keep it small.",
                         1, 1000)));

    auto & sensor = settings_.sensor;
    const auto & sensor_defaults = setting_defaults.sensor;

    const auto input_type = declare_parameter(
        "input_type", std::string("lidar"),
        describe("'lidar' reads the sensor over UDP, 'pcap' replays a capture."));
    sensor.input_type = domain::parseInputType(input_type);

    sensor.msop_port = static_cast<std::uint16_t>(declare_parameter<std::int64_t>(
        "msop_port", static_cast<std::int64_t>(sensor_defaults.msop_port),
        describeIntRange("UDP port carrying the point data (MSOP).", 1, 65535)));
    sensor.difop_port = static_cast<std::uint16_t>(declare_parameter<std::int64_t>(
        "difop_port", static_cast<std::int64_t>(sensor_defaults.difop_port),
        describeIntRange("UDP port carrying device info and calibration (DIFOP).", 1, 65535)));
    sensor.host_address = declare_parameter(
        "host_address", sensor_defaults.host_address,
        describe("Local address to bind; 0.0.0.0 accepts the stream on any interface."));
    sensor.group_address = declare_parameter(
        "group_address", sensor_defaults.group_address,
        describe("Multicast group to join, when the lidar multicasts."));

    sensor.min_distance = static_cast<float>(declare_parameter(
        "min_distance", static_cast<double>(sensor_defaults.min_distance),
        describeRange("Returns closer than this are discarded by the decoder [m].", 0.0, 1000.0)));
    sensor.max_distance = static_cast<float>(declare_parameter(
        "max_distance", static_cast<double>(sensor_defaults.max_distance),
        describePositive("Returns further than this are discarded by the decoder [m].", 1000.0)));
    sensor.use_lidar_clock = declare_parameter(
        "use_lidar_clock", sensor_defaults.use_lidar_clock,
        describe("True stamps clouds with the lidar's clock, which the rover cannot compare."));
    sensor.dense_points = declare_parameter(
        "dense_points", sensor_defaults.dense_points,
        describe("True drops invalid returns; false keeps them as NaN."));
    sensor.ts_first_point = declare_parameter(
        "ts_first_point", sensor_defaults.ts_first_point,
        describe("True stamps a cloud with its first point, false with its last."));
    sensor.wait_for_difop = declare_parameter(
        "wait_for_difop", sensor_defaults.wait_for_difop,
        describe("True withholds clouds until the calibration DIFOP has been received."));
    sensor.start_angle = static_cast<float>(declare_parameter(
        "start_angle", static_cast<double>(sensor_defaults.start_angle),
        describeRange("First azimuth kept in a revolution [deg].", 0.0, 360.0)));
    sensor.end_angle = static_cast<float>(declare_parameter(
        "end_angle", static_cast<double>(sensor_defaults.end_angle),
        describeRange("Last azimuth kept in a revolution [deg].", 0.0, 360.0)));

    sensor.pcap_path = declare_parameter(
        "pcap_path", std::string(""), describe("Capture to replay when input_type is 'pcap'."));
    sensor.pcap_repeat = declare_parameter(
        "pcap_repeat", sensor_defaults.pcap_repeat, describe("Loop the capture when it ends."));
    sensor.pcap_rate = static_cast<float>(declare_parameter(
        "pcap_rate", static_cast<double>(sensor_defaults.pcap_rate),
        describePositive("Replay speed multiplier.", 100.0)));

    auto & scan = settings_.scan;
    const auto & scan_defaults = setting_defaults.scan;

    scan.min_height = declare_parameter(
        "scan.min_height", scan_defaults.min_height,
        describeRange("Bottom of the slice flattened into the scan [m].", -100.0, 100.0));
    scan.max_height = declare_parameter(
        "scan.max_height", scan_defaults.max_height,
        describeRange("Top of the slice flattened into the scan [m].", -100.0, 100.0));
    scan.angle_min = declare_parameter(
        "scan.angle_min", scan_defaults.angle_min,
        describeRange("First bearing in the scan [rad].", -M_PI, M_PI));
    scan.angle_max = declare_parameter(
        "scan.angle_max", scan_defaults.angle_max,
        describeRange("Last bearing in the scan [rad].", -M_PI, M_PI));
    scan.angle_increment = declare_parameter(
        "scan.angle_increment", scan_defaults.angle_increment,
        describePositive("Angular width of one scan bin [rad].", 2.0 * M_PI));
    scan.scan_time = declare_parameter(
        "scan.scan_time", scan_defaults.scan_time,
        describePositive("Time one revolution takes [s].", 100.0));
    scan.range_min = declare_parameter(
        "scan.range_min", scan_defaults.range_min,
        describeRange("Returns closer than this are left out of the scan [m].", 0.0, 1000.0));
    scan.range_max = declare_parameter(
        "scan.range_max", scan_defaults.range_max,
        describePositive("Returns further than this are left out of the scan [m].", 1000.0));
    scan.use_inf = declare_parameter(
        "scan.use_inf", scan_defaults.use_inf,
        describe("True reports a beam with no return as +inf, false as range_max + 1."));

    health_thresholds_.expected_rate_hz = declare_parameter(
        "expected_rate_hz", health_defaults.expected_rate_hz,
        describePositive("Spin rate the lidar is configured for [Hz].", 100.0));
    health_thresholds_.min_rate_ratio = declare_parameter(
        "min_rate_ratio", health_defaults.min_rate_ratio,
        describePositive("Warn below expected_rate_hz * min_rate_ratio.", 1.0));
    health_thresholds_.cloud_timeout_s = declare_parameter(
        "cloud_timeout_s", health_defaults.cloud_timeout_s,
        describePositive("Report an error when no cloud arrives for this long [s].", 600.0));
    publish_frequency_ = declare_parameter(
        "publish_frequency", 1.0,
        describePositive("Diagnostics evaluation rate [Hz].", 100.0));
    health_thresholds_.min_points_warn = static_cast<std::uint32_t>(
        declare_parameter<std::int64_t>(
            "min_points_warn", static_cast<std::int64_t>(health_defaults.min_points_warn),
            describeIntRange("Warn below this many points per cloud.", 0, 10000000)));
}

void RoverRs16LidarNode::init(std::shared_ptr<domain::LidarSourcePort> source)
{
    diagnostic_updater_->setHardwareID("RoverLidar");

    monitor_lidar_ = std::make_shared<application::MonitorLidarUseCase>(
        health_thresholds_,
        std::make_shared<infrastructure::Ros2LidarHealthPublisher>(diagnostic_updater_));

    auto cloud_publisher = std::make_shared<infrastructure::Ros2PointCloudPublisher>(
        this, settings_.point_cloud_topic, settings_.frame_id, settings_.publisher_queue_size);

    std::shared_ptr<domain::LaserScanPublisherPort> scan_publisher;
    std::shared_ptr<domain::ScanProjector> scan_projector;
    if (settings_.scan.enabled) {
        scan_publisher = std::make_shared<infrastructure::Ros2LaserScanPublisher>(
            this, settings_.scan_topic, settings_.frame_id, settings_.publisher_queue_size);
        scan_projector = std::make_shared<domain::ScanProjector>(settings_.scan);
    }

    stream_lidar_ = std::make_unique<application::StreamLidarUseCase>(
        std::move(cloud_publisher), std::move(scan_publisher), std::move(scan_projector),
        monitor_lidar_);

    source_ = source ? std::move(source)
                     : std::make_shared<infrastructure::RsDriverLidarSource>(settings_.sensor);
    source_->setFrameCallback([this](const domain::PointCloudFrame & cloud) { onFrame(cloud); });
    source_->setErrorCallback([this](const std::string & message) { onSourceError(message); });

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    tick_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        std::bind(&RoverRs16LidarNode::tickCallback, this));

    // Publish the STALE status immediately instead of after the first timer period.
    tickCallback();

    source_->start();

    const std::string scan_note =
        settings_.scan.enabled ? "; scan on '" + settings_.scan_topic + "'" : std::string();
    RCLCPP_INFO(
        get_logger(), "RS16 streaming on '%s' in frame '%s'%s.",
        settings_.point_cloud_topic.c_str(), settings_.frame_id.c_str(), scan_note.c_str());
}

void RoverRs16LidarNode::onFrame(const domain::PointCloudFrame & cloud)
{
    // Runs on the source's thread, not the executor. Arrival time on the rover clock, not
    // cloud.timestamp_s: with use_lidar_clock that stamp comes from the sensor's own clock.
    stream_lidar_->onFrame(cloud, nowSeconds());
}

void RoverRs16LidarNode::onSourceError(const std::string & message)
{
    // Throttled: a lidar on the wrong subnet reports this for every packet it fails to read.
    RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000, "rs_driver: %s", message.c_str());
}

void RoverRs16LidarNode::tickCallback()
{
    monitor_lidar_->publishHealth(nowSeconds());
}

double RoverRs16LidarNode::nowSeconds() const
{
    return get_clock()->now().seconds();
}

}  // namespace rover_rs16_lidar
