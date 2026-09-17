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

#include "rover_gps/infrastructure/rover_gps_driver_node.hpp"

#include <chrono>
#include <cstdint>
#include <ctime>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>

#include "rcl_interfaces/msg/parameter_descriptor.hpp"

namespace rover_gps
{

using namespace std::chrono_literals;

namespace
{

rcl_interfaces::msg::ParameterDescriptor describe(const std::string & description)
{
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.description = description;
    descriptor.read_only = true;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describeIntRange(
    const std::string & description, int64_t from_value, int64_t to_value)
{
    auto descriptor = describe(description);
    descriptor.integer_range.resize(1);
    descriptor.integer_range[0].from_value = from_value;
    descriptor.integer_range[0].to_value = to_value;
    return descriptor;
}

rcl_interfaces::msg::ParameterDescriptor describePositive(
    const std::string & description, double max_value)
{
    auto descriptor = describe(description);
    descriptor.floating_point_range.resize(1);
    descriptor.floating_point_range[0].from_value = 1.0e-9;
    descriptor.floating_point_range[0].to_value = max_value;
    return descriptor;
}

/** @brief Strips CR, LF and surrounding blanks from one line of a datagram. */
std::string_view trimLine(std::string_view line)
{
    const auto is_blank = [](char c) {
        return c == ' ' || c == '\t' || c == '\r' || c == '\n' || c == '\v' || c == '\f';
    };
    while (!line.empty() && is_blank(line.front())) {
        line.remove_prefix(1);
    }
    while (!line.empty() && is_blank(line.back())) {
        line.remove_suffix(1);
    }
    return line;
}

}  // namespace

RoverGpsDriverNode::RoverGpsDriverNode(
    const std::string & node_name, const std::string & ns, const rclcpp::NodeOptions & options)
: LifecycleNode(node_name, ns, options)
{
    listen_ip_ = declare_parameter(
        "ip", std::string("0.0.0.0"),
        describe("Local address to bind; 0.0.0.0 listens on every interface."));
    listen_port_ = static_cast<uint16_t>(declare_parameter(
        "port", static_cast<int64_t>(10110),
        describeIntRange("Local UDP port the receiver forwards NMEA to; 0 binds any free port.",
                         0, 65535)));
    buffer_size_ = static_cast<std::size_t>(declare_parameter(
        "buffer_size", static_cast<int64_t>(4096),
        describeIntRange("Maximum datagram size to accept [bytes].", 64, 1 << 20)));
    // Declared as an integer, as the Python driver did, so an existing `timeout_sec: 2` in YAML
    // still loads: a double-typed parameter would reject that integer override.
    timeout_s_ = static_cast<double>(declare_parameter(
        "timeout_sec", static_cast<int64_t>(2),
        describeIntRange("Receive timeout [s]; also how fast a deactivate is noticed.", 1, 3600)));

    const std::string frame_id = declare_parameter(
        "frame_id", std::string("gps"), describe("frame_id of the published messages."));
    const std::string tf_prefix = declare_parameter(
        "tf_prefix", std::string(""),
        describe("Prefix joined to frame_id with '/'; the launch file passes the namespace."));
    frame_id_ = tf_prefix.empty() ? frame_id : tf_prefix + "/" + frame_id;

    time_ref_source_ = declare_parameter(
        "time_ref_source", std::string("gps"),
        describe("`source` field of the time_reference message; defaults to frame_id when empty."));

    assembler_config_.use_rmc = declare_parameter(
        "useRMC", false,
        describe("Take the fix from RMC instead of GGA. GGA also carries quality and HDOP, which "
                 "set the fix covariance, so it is normally preferred."));

    const domain::nmea::EpeQualityDefaults epe_defaults;
    assembler_config_.epe.quality0 = declare_parameter(
        "epe_quality0", epe_defaults.quality0,
        describePositive("Default position error for an invalid/unknown fix [m].", 1.0e9));
    assembler_config_.epe.quality1 = declare_parameter(
        "epe_quality1", epe_defaults.quality1,
        describePositive("Default position error for an SPS fix [m].", 1.0e9));
    assembler_config_.epe.quality2 = declare_parameter(
        "epe_quality2", epe_defaults.quality2,
        describePositive("Default position error for a DGPS fix [m].", 1.0e9));
    assembler_config_.epe.quality4 = declare_parameter(
        "epe_quality4", epe_defaults.quality4,
        describePositive("Default position error for an RTK fixed solution [m].", 1.0e9));
    assembler_config_.epe.quality5 = declare_parameter(
        "epe_quality5", epe_defaults.quality5,
        describePositive("Default position error for an RTK float solution [m].", 1.0e9));
    assembler_config_.epe.quality9 = declare_parameter(
        "epe_quality9", epe_defaults.quality9,
        describePositive("Default position error for a WAAS fix [m].", 1.0e9));
}

RoverGpsDriverNode::~RoverGpsDriverNode()
{
    stopReceiving();
}

std::string RoverGpsDriverNode::resolveFrameId() const
{
    return frame_id_;
}

application::IngestStatistics RoverGpsDriverNode::statistics() const
{
    return ingest_ ? ingest_->statistics() : application::IngestStatistics{};
}

RoverGpsDriverNode::CallbackReturn RoverGpsDriverNode::on_configure(
    const rclcpp_lifecycle::State & /*state*/)
{
    RCLCPP_INFO(
        get_logger(), "Configuring: ip %s, port %u, buffer_size %zu, timeout_sec %.0f, frame_id %s",
        listen_ip_.c_str(), static_cast<unsigned>(listen_port_), buffer_size_, timeout_s_,
        frame_id_.c_str());

    publisher_ = std::make_shared<infrastructure::Ros2NmeaPublisher>(
        *this, resolveFrameId(), time_ref_source_);

    try {
        ingest_ = std::make_unique<application::IngestNmeaUseCase>(assembler_config_, publisher_);
    } catch (const std::exception & e) {
        RCLCPP_ERROR(get_logger(), "Failed to build the NMEA pipeline: %s", e.what());
        publisher_.reset();
        return CallbackReturn::FAILURE;
    }

    return CallbackReturn::SUCCESS;
}

RoverGpsDriverNode::CallbackReturn RoverGpsDriverNode::on_activate(
    const rclcpp_lifecycle::State & /*state*/)
{
    std::string error;
    if (!receiver_.open(listen_ip_, listen_port_, timeout_s_, buffer_size_, error)) {
        RCLCPP_ERROR(
            get_logger(), "Cannot bind UDP %s:%u - %s", listen_ip_.c_str(),
            static_cast<unsigned>(listen_port_), error.c_str());
        return CallbackReturn::FAILURE;
    }

    publisher_->activate();

    running_ = true;
    receive_thread_ = std::thread(&RoverGpsDriverNode::receiveLoop, this);

    RCLCPP_INFO(
        get_logger(), "Listening for NMEA on UDP %s:%u", listen_ip_.c_str(),
        static_cast<unsigned>(receiver_.boundPort()));

    return CallbackReturn::SUCCESS;
}

RoverGpsDriverNode::CallbackReturn RoverGpsDriverNode::on_deactivate(
    const rclcpp_lifecycle::State & /*state*/)
{
    // Join before disabling the publishers, so the receive thread can never publish into a
    // deactivated publisher.
    stopReceiving();

    if (publisher_) {
        publisher_->deactivate();
    }

    const application::IngestStatistics stats = statistics();
    RCLCPP_INFO(
        get_logger(),
        "Deactivated. Sentences published %zu, no-output %zu, bad checksum %zu, unparsed %zu",
        stats.published, stats.no_output, stats.checksum_failed, stats.unparsed);

    return CallbackReturn::SUCCESS;
}

RoverGpsDriverNode::CallbackReturn RoverGpsDriverNode::on_cleanup(
    const rclcpp_lifecycle::State & /*state*/)
{
    stopReceiving();
    ingest_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

RoverGpsDriverNode::CallbackReturn RoverGpsDriverNode::on_shutdown(
    const rclcpp_lifecycle::State & /*state*/)
{
    stopReceiving();
    ingest_.reset();
    publisher_.reset();
    return CallbackReturn::SUCCESS;
}

void RoverGpsDriverNode::stopReceiving()
{
    running_ = false;
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    receiver_.close();
}

void RoverGpsDriverNode::receiveLoop()
{
    std::string payload;
    std::string error;

    while (running_) {
        // Re-bind after a socket failure, mirroring the Python driver's outer connect loop.
        if (!receiver_.isOpen()) {
            if (!receiver_.open(listen_ip_, listen_port_, timeout_s_, buffer_size_, error)) {
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000, "Cannot re-bind UDP %s:%u - %s",
                    listen_ip_.c_str(), static_cast<unsigned>(listen_port_), error.c_str());
                std::this_thread::sleep_for(1s);
                continue;
            }
            RCLCPP_INFO(get_logger(), "UDP socket re-bound");
        }

        switch (receiver_.receive(payload, error)) {
            case infrastructure::ReceiveStatus::Datagram:
                handleDatagram(payload);
                break;

            case infrastructure::ReceiveStatus::Timeout:
                // A quiet link is normal; rover_gps_node reports the missing fix as a diagnostic.
                break;

            case infrastructure::ReceiveStatus::Error:
                RCLCPP_ERROR_THROTTLE(
                    get_logger(), *get_clock(), 5000, "UDP receive failed: %s", error.c_str());
                receiver_.close();
                break;
        }
    }
}

void RoverGpsDriverNode::handleDatagram(std::string_view payload)
{
    const double stamp_s = now().seconds();
    const std::time_t now_utc = std::time(nullptr);

    std::size_t start = 0;
    while (start <= payload.size()) {
        const std::size_t newline = payload.find('\n', start);
        const std::string_view raw = (newline == std::string_view::npos)
            ? payload.substr(start)
            : payload.substr(start, newline - start);

        // Trim per line, not per datagram: a multi-sentence datagram leaves a CR on every line but
        // the last, and a trailing CR makes the sentence unparseable.
        const std::string_view sentence = trimLine(raw);
        if (!sentence.empty()) {
            const application::IngestResult result = ingest_->onSentence(sentence, stamp_s, now_utc);
            if (result == application::IngestResult::ChecksumFailed) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 5000,
                    "Received a sentence with an invalid checksum: %s",
                    std::string(sentence).c_str());
            }
        }

        if (newline == std::string_view::npos) {
            break;
        }
        start = newline + 1;
    }
}

}  // namespace rover_gps
