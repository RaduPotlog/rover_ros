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

#include "rover_crsf_teleop/infrastructure/rover_crsf_teleop_node.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <utility>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "rover_crsf_teleop/infrastructure/teleop_diagnostics_conversions.hpp"

namespace rover_crsf_teleop
{

using namespace std::chrono_literals;

namespace
{

constexpr char kCmdVelTopic[] = "teleop_elrs_cmd_vel_stamped";
constexpr char kCmdVelFrameId[] = "base_link";
constexpr char kRcChannelsTopic[] = "rc/channels";
constexpr char kRcLinkTopic[] = "rc/link";
constexpr auto kControlPeriod = 20ms;

// Beyond this with no bytes at all, the serial bridge is presumed dead rather than merely quiet.
// A receiver that is powered but out of range still sends LINK_STATISTICS, so silence on the
// byte stream means the bridge or the USB link, not the RC link.
constexpr auto kSerialSilenceTimeout = 1000ms;

bool isValidChannel(const int channel_number)
{
    return channel_number >= 1 && static_cast<std::size_t>(channel_number) <= RcFrame::kChannelCount;
}

}  // namespace

RoverCrsfTeleopNode::RoverCrsfTeleopNode(
    const std::string & node_name, const rclcpp::NodeOptions & options)
: rclcpp_lifecycle::LifecycleNode(node_name, options)
{
    declareParameters();

    // Created here, not in on_configure: the Updater declares diagnostic_updater.period, which a
    // cleanup -> configure cycle would otherwise try to declare twice.
    const double expected_hz = get_parameter("rc_channels_expected_hz").as_double();
    channels_min_hz_ = expected_hz;
    channels_max_hz_ = expected_hz;
    channels_rate_ = std::make_unique<diagnostic_updater::FrequencyStatus>(
        diagnostic_updater::FrequencyStatusParam(
            &channels_min_hz_, &channels_max_hz_,
            get_parameter("rc_channels_rate_tolerance").as_double(), 10),
        "RC channels rate", get_clock());

    diagnostic_updater_ = std::make_unique<diagnostic_updater::Updater>(this);
    diagnostic_updater_->setHardwareID("RC Receiver");
    diagnostic_updater_->add("RC link", this, &RoverCrsfTeleopNode::diagnoseRcLink);
    diagnostic_updater_->add("E-Stop requests", this, &RoverCrsfTeleopNode::diagnoseSafetyRequests);
    diagnostic_updater_->add("RC channels rate", this, &RoverCrsfTeleopNode::diagnoseChannelsRate);
    diagnostic_updater_->add("RC serial link", this, &RoverCrsfTeleopNode::diagnoseSerialLink);
}

void RoverCrsfTeleopNode::declareParameters()
{
    // Every parameter is declared, per .claude/rules/ros2_general.md - a silent get_parameter()
    // on an undeclared name is a bug. Channel defaults are the raw CRSF endpoints from
    // domain/crsf/crsf_protocol.hpp, which is what the decoder actually produces.
    declare_parameter<int>("channel_in_min", kDefaultCrsfChannelMin);
    declare_parameter<int>("channel_in_mid", kDefaultCrsfChannelMid);
    declare_parameter<int>("channel_in_max", kDefaultCrsfChannelMax);
    declare_parameter<int>("channel_deadband", kDefaultChannelDeadband);

    declare_parameter<int>("linear_x_channel", 3);
    declare_parameter<double>("linear_x_out_min", -2.0);
    declare_parameter<double>("linear_x_out_max", 2.0);
    declare_parameter<bool>("linear_x_invert", false);

    declare_parameter<int>("angular_z_channel", 1);
    declare_parameter<double>("angular_z_out_min", -5.0);
    declare_parameter<double>("angular_z_out_max", 5.0);
    // The original implementation inverted this axis by swapping in_min/in_max; the mapping is
    // defined about the midpoint now, so the inversion is explicit.
    declare_parameter<bool>("angular_z_invert", true);

    // Outer-wheel rim speed budget (m/s) and wheel_separation * wheel_separation_multiplier (m).
    // 0.0 disables the limit; see domain/rim_speed_limit.hpp.
    declare_parameter<double>("max_wheel_rim_speed", 0.0);
    declare_parameter<double>("effective_track_width", 0.0);

    declare_parameter<int>("e_stop_channel", 5);
    declare_parameter<int>("e_stop_latch_reset_channel", 4);
    declare_parameter<int>("channel_switch_threshold", 500);
    declare_parameter<int>("switch_settle_frames", static_cast<int>(kDefaultSwitchSettleFrames));

    const LinkMonitorConfig link_defaults;
    declare_parameter<int>("channel_timeout_ms", static_cast<int>(link_defaults.channel_timeout.count()));
    declare_parameter<int>(
        "link_stats_timeout_ms", static_cast<int>(link_defaults.link_stats_timeout.count()));
    declare_parameter<bool>("require_link_stats", link_defaults.require_link_stats);
    declare_parameter<int>("link_quality_lost_below", link_defaults.lq_lost_below);
    declare_parameter<int>("link_quality_recovered_at", link_defaults.lq_recovered_at);

    // The UART is opened by rover_serial_driver's rover_serial_bridge_node, which the launch file starts.
    // These two are read by the LAUNCH FILE and handed to that node; this node declares them so
    // they live in one config file and are introspectable with `ros2 param get`.
    declare_parameter<std::string>("serial_device", "/dev/ttyUSB0");
    declare_parameter<int>("serial_baudrate", 460800);

    // Topic carrying raw bytes from rover_serial_bridge_node (its `serial_read`, remapped by the launch
    // file). Must match that remap.
    declare_parameter<std::string>("serial_topic", "rc/raw");

    // Echo decoded frames on rc/channels and rc/link. Nothing on the rover consumes them.
    declare_parameter<bool>("publish_rc_topics", true);

    // Diagnostics only - read once at construction. Now measures DECODED CRSF frames, i.e. the
    // receiver's real packet rate.
    declare_parameter<double>("rc_channels_expected_hz", 50.0);
    // Fraction the measured rc/channels rate may deviate before "RC channels rate" warns.
    declare_parameter<double>("rc_channels_rate_tolerance", 0.2);
}

std::optional<TeleopConfig> RoverCrsfTeleopNode::readConfig()
{
    TeleopConfig config;

    const int channel_in_min = get_parameter("channel_in_min").as_int();
    const int channel_in_mid = get_parameter("channel_in_mid").as_int();
    const int channel_in_max = get_parameter("channel_in_max").as_int();
    const int channel_deadband = get_parameter("channel_deadband").as_int();

    if (channel_deadband < 0) {
        RCLCPP_ERROR(get_logger(), "channel_deadband must be >= 0 (got %d).", channel_deadband);
        return std::nullopt;
    }

    for (AxisMapping * mapping : {&config.linear_x_mapping, &config.angular_z_mapping}) {
        mapping->in_min = channel_in_min;
        mapping->in_mid = channel_in_mid;
        mapping->in_max = channel_in_max;
        mapping->deadband_counts = channel_deadband;
    }

    config.linear_x_mapping.out_min = get_parameter("linear_x_out_min").as_double();
    config.linear_x_mapping.out_max = get_parameter("linear_x_out_max").as_double();
    config.linear_x_mapping.invert = get_parameter("linear_x_invert").as_bool();

    config.angular_z_mapping.out_min = get_parameter("angular_z_out_min").as_double();
    config.angular_z_mapping.out_max = get_parameter("angular_z_out_max").as_double();
    config.angular_z_mapping.invert = get_parameter("angular_z_invert").as_bool();

    config.max_wheel_rim_speed = get_parameter("max_wheel_rim_speed").as_double();
    config.half_track_width = get_parameter("effective_track_width").as_double() / 2.0;

    config.linear_x_channel = static_cast<int>(get_parameter("linear_x_channel").as_int());
    config.angular_z_channel = static_cast<int>(get_parameter("angular_z_channel").as_int());
    config.e_stop_channel = static_cast<int>(get_parameter("e_stop_channel").as_int());
    config.e_stop_latch_reset_channel =
        static_cast<int>(get_parameter("e_stop_latch_reset_channel").as_int());
    config.channel_switch_threshold =
        static_cast<int>(get_parameter("channel_switch_threshold").as_int());

    // Fail configure on a bad channel number instead of the previous behaviour of reading it as
    // 0 - which the stick mapping clamps to full negative deflection.
    const std::array<std::pair<const char *, int>, 4> channel_roles{{
        {"linear_x_channel", config.linear_x_channel},
        {"angular_z_channel", config.angular_z_channel},
        {"e_stop_channel", config.e_stop_channel},
        {"e_stop_latch_reset_channel", config.e_stop_latch_reset_channel}}};
    for (const auto & [name, channel] : channel_roles) {
        if (!isValidChannel(channel)) {
            RCLCPP_ERROR(
                get_logger(), "Parameter %s = %d is outside 1-%zu.", name, channel,
                RcFrame::kChannelCount);
            return std::nullopt;
        }
    }

    // Two roles on one channel would drive e.g. the E-Stop from a stick, so fail configure.
    for (std::size_t i = 0; i < channel_roles.size(); ++i) {
        for (std::size_t j = i + 1; j < channel_roles.size(); ++j) {
            if (channel_roles[i].second == channel_roles[j].second) {
                RCLCPP_ERROR(
                    get_logger(), "Parameters %s and %s both use channel %d.",
                    channel_roles[i].first, channel_roles[j].first, channel_roles[i].second);
                return std::nullopt;
            }
        }
    }

    const int64_t settle_frames = get_parameter("switch_settle_frames").as_int();
    const int64_t channel_timeout_ms = get_parameter("channel_timeout_ms").as_int();
    const int64_t link_stats_timeout_ms = get_parameter("link_stats_timeout_ms").as_int();
    const int64_t lq_lost_below = get_parameter("link_quality_lost_below").as_int();
    const int64_t lq_recovered_at = get_parameter("link_quality_recovered_at").as_int();

    if (settle_frames < 0 || channel_timeout_ms <= 0 || link_stats_timeout_ms <= 0) {
        RCLCPP_ERROR(
            get_logger(),
            "switch_settle_frames must be >= 0 and channel_timeout_ms / link_stats_timeout_ms > 0.");
        return std::nullopt;
    }

    if (lq_lost_below < 0 || lq_recovered_at > 100 || lq_recovered_at < lq_lost_below) {
        RCLCPP_ERROR(
            get_logger(),
            "Link quality thresholds must satisfy 0 <= link_quality_lost_below <= "
            "link_quality_recovered_at <= 100 (got %ld and %ld).",
            static_cast<long>(lq_lost_below), static_cast<long>(lq_recovered_at));
        return std::nullopt;
    }

    config.switch_settle_frames = static_cast<unsigned int>(settle_frames);
    config.link.channel_timeout = std::chrono::milliseconds(channel_timeout_ms);
    config.link.link_stats_timeout = std::chrono::milliseconds(link_stats_timeout_ms);
    config.link.require_link_stats = get_parameter("require_link_stats").as_bool();
    config.link.lq_lost_below = static_cast<std::uint8_t>(lq_lost_below);
    config.link.lq_recovered_at = static_cast<std::uint8_t>(lq_recovered_at);

    RCLCPP_INFO(
        get_logger(),
        "RC channel range [%d, %d] with midpoint %d and deadband %d counts. Link lost after "
        "%ld ms without channels%s.",
        channel_in_min, channel_in_max, channel_in_mid, channel_deadband,
        static_cast<long>(channel_timeout_ms),
        config.link.require_link_stats ? " or on stale / low-quality link stats" : "");

    return config;
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_configure(const rclcpp_lifecycle::State &)
{
    const auto config = readConfig();

    if (!config.has_value()) {
        return CallbackReturn::FAILURE;
    }

    velocity_publisher_ =
        std::make_shared<Ros2VelocityCommandPublisher>(*this, kCmdVelTopic, kCmdVelFrameId);
    safety_switch_ = std::make_shared<Ros2TriggerSafetySwitch>(*this);
    use_case_ = std::make_unique<TeleopUseCase>(*config, velocity_publisher_, safety_switch_);
    channels_rate_->clear();

    publish_rc_topics_ = get_parameter("publish_rc_topics").as_bool();

    // A cleanup -> configure cycle must not decode a half-frame left over from before.
    parser_.reset();
    last_serial_message_.reset();
    last_decoded_frame_.reset();
    serial_bytes_received_ = 0;
    decoded_frames_ = 0;
    decoded_link_stats_ = 0;

    if (publish_rc_topics_) {
        // Plain (not lifecycle) publishers on purpose - see the header. Best-effort, depth 1:
        // these are echoes for tuning, and a late RC frame is worthless.
        const auto echo_qos = rclcpp::QoS(1).best_effort().durability_volatile();
        rc_channels_publisher_ = create_publisher<rover_msgs::msg::RcChannels>(kRcChannelsTopic, echo_qos);
        rc_link_publisher_ = create_publisher<rover_msgs::msg::RcLinkStatus>(kRcLinkTopic, echo_qos);
    }

    // Matches rover_serial_bridge_node's publisher (rclcpp::QoS{100}, reliable). Reliable is right here:
    // unlike a decoded RC frame, a dropped BYTE chunk desynchronises the parser until the next
    // sync byte, so the transport must not be the thing dropping it.
    //
    // Subscribed in configure, not activate: input is recorded even while inactive, so the link
    // is already known healthy (or not) the moment teleop is activated.
    const std::string serial_topic = get_parameter("serial_topic").as_string();

    serial_subscriber_ = create_subscription<std_msgs::msg::UInt8MultiArray>(
        serial_topic, rclcpp::QoS(100),
        [this](const std_msgs::msg::UInt8MultiArray & msg) {
            last_serial_message_ = std::chrono::steady_clock::now();
            serial_bytes_received_ += msg.data.size();

            // Decodes on the executor thread and calls onRcChannels / onLinkStatistics below,
            // synchronously, before returning.
            parser_.parse(msg.data.data(), msg.data.size(), *this);
        });

    RCLCPP_INFO(
        get_logger(), "Decoding CRSF from '%s' (rover_serial_bridge_node opens %s at %ld baud).",
        serial_topic.c_str(), get_parameter("serial_device").as_string().c_str(),
        static_cast<long>(get_parameter("serial_baudrate").as_int()));

    return CallbackReturn::SUCCESS;
}

void RoverCrsfTeleopNode::onRcChannels(const RcFrame & frame)
{
    const SteadyTime now = std::chrono::steady_clock::now();

    use_case_->onChannels(frame, now);

    last_decoded_frame_ = now;
    ++decoded_frames_;
    channels_rate_->tick();

    if (rc_channels_publisher_) {
        rc_channels_publisher_->publish(toRcChannelsMsg(frame, this->now()));
    }
}

void RoverCrsfTeleopNode::onLinkStatistics(const RcLinkStats & stats)
{
    use_case_->onLinkStats(stats.uplink_link_quality, std::chrono::steady_clock::now());

    ++decoded_link_stats_;

    if (rc_link_publisher_) {
        rc_link_publisher_->publish(toRcLinkStatusMsg(stats, this->now()));
    }
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_activate(const rclcpp_lifecycle::State &)
{
    velocity_publisher_->on_activate();
    last_tick_status_.reset();

    control_timer_ = create_wall_timer(kControlPeriod, [this]() { controlTimerCallback(); });

    RCLCPP_INFO(get_logger(), "RC teleop active.");
    return CallbackReturn::SUCCESS;
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_deactivate(const rclcpp_lifecycle::State &)
{
    control_timer_->cancel();
    control_timer_.reset();

    // Publish the stop before the publisher goes inactive, or it would be dropped.
    use_case_->stop();
    velocity_publisher_->on_deactivate();

    RCLCPP_INFO(get_logger(), "RC teleop inactive.");
    return CallbackReturn::SUCCESS;
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_cleanup(const rclcpp_lifecycle::State &)
{
    releaseResources();
    return CallbackReturn::SUCCESS;
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_shutdown(const rclcpp_lifecycle::State &)
{
    if (control_timer_ && use_case_) {
        control_timer_->cancel();
        use_case_->stop();
    }

    releaseResources();
    return CallbackReturn::SUCCESS;
}

void RoverCrsfTeleopNode::releaseResources()
{
    control_timer_.reset();
    serial_subscriber_.reset();
    rc_channels_publisher_.reset();
    rc_link_publisher_.reset();
    parser_.reset();
    use_case_.reset();
    safety_switch_.reset();
    velocity_publisher_.reset();
}

void RoverCrsfTeleopNode::controlTimerCallback()
{
    const TickStatus status = use_case_->tick(std::chrono::steady_clock::now());

    if (status == last_tick_status_) {
        return;
    }

    // Log transitions only - this runs at 50 Hz.
    switch (status) {
        case TickStatus::kWaitingForFirstFrame:
            RCLCPP_INFO(get_logger(), "Waiting for the first decoded CRSF frame.");
            break;
        case TickStatus::kLinkLost:
            RCLCPP_WARN(get_logger(), "RC link lost: sent a zero command, teleop is silent.");
            break;
        case TickStatus::kActive:
            RCLCPP_INFO(get_logger(), "RC link healthy: teleop commanding.");
            break;
    }

    last_tick_status_ = status;
}

void RoverCrsfTeleopNode::diagnoseRcLink(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    status.add("Lifecycle state", get_current_state().label());

    if (!use_case_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Teleop not configured.");
        return;
    }

    const bool active =
        get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
    fillRcLinkStatus(use_case_->diagnostics(std::chrono::steady_clock::now()), active, status);
}

void RoverCrsfTeleopNode::diagnoseSafetyRequests(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!safety_switch_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Teleop not configured.");
        return;
    }

    fillSafetyRequestsStatus(safety_switch_->requestStatuses(), status);
}

void RoverCrsfTeleopNode::diagnoseChannelsRate(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if (!use_case_) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, "Teleop not configured.");
        return;
    }

    channels_rate_->run(status);

    // FrequencyStatus reports "No events recorded." as ERROR. RC teleop is optional and link loss
    // is already reported by the "RC link" task, so the rate task only warns.
    if (status.level > diagnostic_msgs::msg::DiagnosticStatus::WARN) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::WARN, status.message);
    }
}

void RoverCrsfTeleopNode::diagnoseSerialLink(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    // Distinct from "RC link": that one judges whether the RC signal is good enough to drive on;
    // this one judges whether bytes are reaching us from rover_serial_bridge_node at all. They fail for
    // different reasons - a receiver out of range versus an unplugged dongle or a bridge stuck
    // in `unconfigured` - and the operator needs to tell them apart.
    status.add("Topic", get_parameter("serial_topic").as_string());
    status.add("Device (opened by rover_serial_bridge_node)", get_parameter("serial_device").as_string());
    status.add("Baud rate", get_parameter("serial_baudrate").as_int());
    status.add("Bytes received", static_cast<int>(serial_bytes_received_));
    status.add("CRSF frames decoded", static_cast<int>(decoded_frames_));
    status.add("Link stats decoded", static_cast<int>(decoded_link_stats_));
    status.add("Bytes buffered", static_cast<int>(parser_.bufferedBytes()));

    if (!last_serial_message_.has_value()) {
        // WARN, never ERROR: RC teleop is optional, and the rover must not be held back because
        // nobody plugged the receiver in.
        status.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "No bytes yet - is rover_serial_bridge_node running and active?");
        return;
    }

    const SteadyTime now = std::chrono::steady_clock::now();
    const auto silence =
        std::chrono::duration_cast<std::chrono::milliseconds>(now - *last_serial_message_);
    status.add("Age of last byte chunk (ms)", static_cast<int>(silence.count()));

    if (silence > kSerialSilenceTimeout) {
        status.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "Byte stream silent for " + std::to_string(silence.count()) + " ms.");
        return;
    }

    if (!last_decoded_frame_.has_value()) {
        // Bytes but no frames means the stream is not CRSF, or the baud rate is wrong.
        status.summary(
            diagnostic_msgs::msg::DiagnosticStatus::WARN,
            "Receiving bytes but decoding no CRSF frames - check the baud rate.");
        return;
    }

    status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Receiving and decoding.");
}

}  // namespace rover_crsf_teleop
