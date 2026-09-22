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

#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <diagnostic_msgs/msg/diagnostic_status.hpp>
#include <lifecycle_msgs/msg/state.hpp>

#include "rover_crsf_teleop/infrastructure/teleop_diagnostics_conversions.hpp"
#include "rover_crsf_teleop/infrastructure/yaml_calibration_store.hpp"

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

constexpr char kSafetyStatusTopic[] = "hardware_interface/safety_status";
constexpr char kCalibrationStateTopic[] = "rc/calibration/state";
constexpr char kCalibrationStartService[] = "rc/calibration/start";
constexpr char kCalibrationSweepService[] = "rc/calibration/sweep";
constexpr char kCalibrationFinishService[] = "rc/calibration/finish";
constexpr char kCalibrationCancelService[] = "rc/calibration/cancel";
constexpr char kCalibrationApplyService[] = "rc/calibration/apply";

// The four per-channel calibration parameters. They were scalars shared by both stick axes until
// per-channel calibration; see rejectScalarChannelParameters().
constexpr std::array<const char *, 4> kChannelArrayParameters{
    "channel_in_min", "channel_in_mid", "channel_in_max", "channel_deadband"};

std::vector<int64_t> channelDefaults(const int value)
{
    return std::vector<int64_t>(RcFrame::kChannelCount, static_cast<int64_t>(value));
}

// Beyond this with no bytes at all, the serial bridge is presumed dead rather than merely quiet.
// A receiver that is powered but out of range still sends LINK_STATISTICS, so silence on the
// byte stream means the bridge or the USB link, not the RC link.
constexpr auto kSerialSilenceTimeout = 1000ms;

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
    diagnostic_updater_->add("RC calibration", this, &RoverCrsfTeleopNode::diagnoseCalibration);
}

void RoverCrsfTeleopNode::declareParameters()
{
    // Every parameter is declared, per .claude/rules/ros2_general.md - a silent get_parameter()
    // on an undeclared name is a bug. Channel defaults are the raw CRSF endpoints from
    // domain/crsf/crsf_protocol.hpp, which is what the decoder actually produces.
    rejectScalarChannelParameters();

    // One entry per channel, index N-1 = channel N. Declared with all 16 filled in rather than a
    // single element, so `ros2 param get` and the calibration page always see the whole picture.
    declare_parameter<std::vector<int64_t>>(
        "channel_in_min", channelDefaults(kDefaultCrsfChannelMin));
    declare_parameter<std::vector<int64_t>>(
        "channel_in_mid", channelDefaults(kDefaultCrsfChannelMid));
    declare_parameter<std::vector<int64_t>>(
        "channel_in_max", channelDefaults(kDefaultCrsfChannelMax));
    declare_parameter<std::vector<int64_t>>(
        "channel_deadband", channelDefaults(kDefaultChannelDeadband));

    // Where a measured calibration is kept between runs. Empty turns persistence off: the
    // calibration flow still works, the result is just lost on restart.
    declare_parameter<std::string>("calibration_file", "");
    // A session holds teleop off, so an abandoned one - a closed browser tab - must not be able
    // to do so forever.
    declare_parameter<int>("calibration_timeout_s", 300);
    declare_parameter<double>("calibration_state_rate_hz", 5.0);
    // How long a safety_status sample stays trustworthy. It is published at 20 Hz, so a second is
    // 20 missed messages; matches rover_twist_mux's gpio_timeout.
    declare_parameter<double>("e_stop_state_timeout_s", 1.0);
    // How long the E-Stop must stay un-engaged before a running calibration is cancelled. The
    // driver reports a Modbus read error as "clear" and the underlying IO only refreshes at
    // 2 Hz, so a single not-engaged sample must not throw away a measurement.
    declare_parameter<double>("e_stop_grace_s", 1.0);

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
    // Stick expo, 0 (linear) .. 1 (softest around centre); see AxisMapping::expo.
    declare_parameter<double>("linear_x_expo", 0.0);
    declare_parameter<double>("angular_z_expo", 0.0);

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

void RoverCrsfTeleopNode::rejectScalarChannelParameters()
{
    // channel_in_* used to be plain integers. Declaring them as arrays turns an old override into
    // an InvalidParameterTypeException thrown out of declare_parameter() with an rcl-flavoured
    // message, which is a poor thing to debug on a rover in a field. Catch it first and say what
    // to do instead.
    for (const auto & override : get_node_options().parameter_overrides()) {
        const bool is_channel_parameter =
            std::find_if(
                kChannelArrayParameters.cbegin(), kChannelArrayParameters.cend(),
                [&override](const char * name) { return override.get_name() == name; }) !=
            kChannelArrayParameters.cend();

        if (is_channel_parameter && override.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER)
        {
            RCLCPP_FATAL(
                get_logger(),
                "Parameter '%s' is now an array of %zu values, one per channel (channel N at "
                "index N-1), but this configuration still sets a single number. Write "
                "'%s: [%ld]' - a one-element list is expanded to every channel, which is exactly "
                "the old behaviour.",
                override.get_name().c_str(), RcFrame::kChannelCount, override.get_name().c_str(),
                static_cast<long>(override.as_int()));
            throw std::runtime_error(
                "rover_crsf_teleop: '" + override.get_name() +
                "' must be a list of per-channel values, not a single number");
        }
    }
}

std::optional<std::array<int, RcFrame::kChannelCount>> RoverCrsfTeleopNode::readChannelArray(
    const char * name, const int lower, const int upper)
{
    const std::vector<int64_t> values = get_parameter(name).as_integer_array();

    if (values.size() != 1 && values.size() != RcFrame::kChannelCount) {
        RCLCPP_ERROR(
            get_logger(),
            "Parameter %s must have 1 or %zu entries (got %zu). One entry is expanded to every "
            "channel.",
            name, RcFrame::kChannelCount, values.size());
        return std::nullopt;
    }

    std::array<int, RcFrame::kChannelCount> out{};

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        const int64_t value = (values.size() == 1) ? values[0] : values[i];

        // A typo'd endpoint used to sail straight through and silently kill the stick throw.
        if (value < lower || value > upper) {
            RCLCPP_ERROR(
                get_logger(), "Parameter %s[%zu] = %ld is outside %d-%d.", name, i,
                static_cast<long>(value), lower, upper);
            return std::nullopt;
        }

        out[i] = static_cast<int>(value);
    }

    return out;
}

std::array<bool, RcFrame::kChannelCount> RoverCrsfTeleopNode::axisChannels(
    const TeleopConfig & config) const
{
    std::array<bool, RcFrame::kChannelCount> axes{};

    for (const int channel : {config.linear_x_channel, config.angular_z_channel}) {
        if (RcFrame::isValidChannel(channel)) {
            axes[static_cast<std::size_t>(channel - 1)] = true;
        }
    }

    return axes;
}

std::optional<TeleopConfig> RoverCrsfTeleopNode::readConfig()
{
    TeleopConfig config;

    // 0-2047 is the whole 11-bit wire domain (crsf_protocol.hpp); the deadband is a half-width,
    // so anything approaching the full span is nonsense.
    const auto in_min = readChannelArray("channel_in_min", 0, 2047);
    const auto in_mid = readChannelArray("channel_in_mid", 0, 2047);
    const auto in_max = readChannelArray("channel_in_max", 0, 2047);
    const auto deadband = readChannelArray("channel_deadband", 0, 1023);

    if (!in_min || !in_mid || !in_max || !deadband) {
        return std::nullopt;
    }

    ChannelCalibration calibration;
    calibration.in_min = *in_min;
    calibration.in_mid = *in_mid;
    calibration.in_max = *in_max;
    calibration.deadband = *deadband;

    config.linear_x_mapping.out_min = get_parameter("linear_x_out_min").as_double();
    config.linear_x_mapping.out_max = get_parameter("linear_x_out_max").as_double();
    config.linear_x_mapping.invert = get_parameter("linear_x_invert").as_bool();
    config.linear_x_mapping.expo = get_parameter("linear_x_expo").as_double();

    config.angular_z_mapping.out_min = get_parameter("angular_z_out_min").as_double();
    config.angular_z_mapping.out_max = get_parameter("angular_z_out_max").as_double();
    config.angular_z_mapping.invert = get_parameter("angular_z_invert").as_bool();
    config.angular_z_mapping.expo = get_parameter("angular_z_expo").as_double();

    config.max_wheel_rim_speed = get_parameter("max_wheel_rim_speed").as_double();
    config.half_track_width = get_parameter("effective_track_width").as_double() / 2.0;

    config.linear_x_channel = static_cast<int>(get_parameter("linear_x_channel").as_int());
    config.angular_z_channel = static_cast<int>(get_parameter("angular_z_channel").as_int());
    config.e_stop_channel = static_cast<int>(get_parameter("e_stop_channel").as_int());
    config.e_stop_latch_reset_channel =
        static_cast<int>(get_parameter("e_stop_latch_reset_channel").as_int());
    config.channel_switch_threshold =
        static_cast<int>(get_parameter("channel_switch_threshold").as_int());

    // Same 0-2047 wire domain the endpoint arrays are held to above. The calibrated-range check
    // further down is a WARN and is gated on having a calibration at all, so without this a
    // negative or out-of-wire-range threshold configured silently and pinned both switches to one
    // position for the life of the node.
    if (config.channel_switch_threshold < 0 || config.channel_switch_threshold > 2047) {
        RCLCPP_ERROR(
            get_logger(), "Parameter channel_switch_threshold = %d is outside 0-2047.",
            config.channel_switch_threshold);
        return std::nullopt;
    }

    // Fail configure on a bad channel number instead of the previous behaviour of reading it as
    // 0 - which the stick mapping clamps to full negative deflection.
    const std::array<std::pair<const char *, int>, 4> channel_roles{{
        {"linear_x_channel", config.linear_x_channel},
        {"angular_z_channel", config.angular_z_channel},
        {"e_stop_channel", config.e_stop_channel},
        {"e_stop_latch_reset_channel", config.e_stop_latch_reset_channel}}};
    for (const auto & [name, channel] : channel_roles) {
        if (!RcFrame::isValidChannel(channel)) {
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

    // The endpoints measured on this transmitter, over the output limits and inversion read
    // above. See applyCalibration() for what a calibration does and does not own.
    config = applyCalibration(config, calibration);

    const std::vector<std::string> problems = calibrationProblems(calibration, axisChannels(config));
    if (!problems.empty()) {
        RCLCPP_ERROR(get_logger(), "Unusable stick calibration: %s", problems.front().c_str());
        return std::nullopt;
    }

    // channel_switch_threshold stays an absolute raw value: a switch sits at the ends of its
    // travel, so comparing raw counts is right, and re-deriving a safety-critical threshold from
    // a measurement an operator just took is a worse failure mode than leaving it explicit. What
    // the calibration does buy is being able to notice when the threshold has fallen outside a
    // switch's actual range - which would leave that switch stuck reading one position forever.
    for (const auto & [name, channel] :
         {std::make_pair("e_stop_channel", config.e_stop_channel),
          std::make_pair("e_stop_latch_reset_channel", config.e_stop_latch_reset_channel)})
    {
        const std::size_t index = static_cast<std::size_t>(channel - 1);
        const int low = calibration.in_min[index];
        const int high = calibration.in_max[index];

        if (high > low && (config.channel_switch_threshold <= low ||
                           config.channel_switch_threshold >= high))
        {
            RCLCPP_WARN(
                get_logger(),
                "channel_switch_threshold %d is outside channel %d's calibrated range %d-%d (%s), "
                "so that switch will always read the same position. Re-measure the channel or "
                "move the threshold.",
                config.channel_switch_threshold, channel, low, high, name);
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
        "Stick calibration: linear_x on channel %d [%d, %d] mid %d deadband %d; angular_z on "
        "channel %d [%d, %d] mid %d deadband %d. Link lost after %ld ms without channels%s.",
        config.linear_x_channel, config.linear_x_mapping.in_min, config.linear_x_mapping.in_max,
        config.linear_x_mapping.in_mid, config.linear_x_mapping.deadband_counts,
        config.angular_z_channel, config.angular_z_mapping.in_min, config.angular_z_mapping.in_max,
        config.angular_z_mapping.in_mid, config.angular_z_mapping.deadband_counts,
        static_cast<long>(channel_timeout_ms),
        config.link.require_link_stats ? " or on stale / low-quality link stats" : "");

    return config;
}

RoverCrsfTeleopNode::CallbackReturn RoverCrsfTeleopNode::on_configure(const rclcpp_lifecycle::State &)
{
    auto config = readConfig();

    if (!config.has_value()) {
        return CallbackReturn::FAILURE;
    }

    base_config_ = *config;

    // Which calibration is in force is decided in the application layer and only reported here -
    // see resolveStartupCalibration(). An empty calibration_file means persistence is off.
    const std::string calibration_file = get_parameter("calibration_file").as_string();
    calibration_store_.reset();

    if (!calibration_file.empty()) {
        calibration_store_ = std::make_shared<YamlCalibrationStore>(calibration_file, get_logger());
    }

    const StartupCalibration startup =
        resolveStartupCalibration(*config, calibration_store_.get(), axisChannels(*config));

    *config = startup.config;
    calibration_source_ = startup.source;

    switch (startup.outcome) {
        case StartupCalibrationOutcome::kStoredApplied:
            RCLCPP_INFO(
                get_logger(), "Using the RC calibration saved in %s.", calibration_source_.c_str());
            break;

        case StartupCalibrationOutcome::kStoredRefused:
            RCLCPP_WARN(
                get_logger(),
                "Ignoring the RC calibration in '%s': %s Using the configured values instead.",
                calibration_file.c_str(), startup.detail.c_str());
            break;

        case StartupCalibrationOutcome::kNothingStored:
            RCLCPP_INFO(
                get_logger(),
                "No RC calibration saved at '%s' yet; using the configured values. Measure one "
                "with the rc/calibration services.",
                calibration_file.c_str());
            break;

        case StartupCalibrationOutcome::kNoStore:
            RCLCPP_INFO(
                get_logger(),
                "RC calibration persistence is off (calibration_file is empty): a calibration can "
                "still be measured and applied, but it will not survive a restart.");
            break;
    }

    velocity_publisher_ =
        std::make_shared<Ros2VelocityCommandPublisher>(*this, kCmdVelTopic, kCmdVelFrameId);
    safety_switch_ = std::make_shared<Ros2TriggerSafetySwitch>(*this);
    use_case_ = std::make_unique<TeleopUseCase>(*config, velocity_publisher_, safety_switch_);
    channels_rate_->clear();

    // Created here rather than in on_activate on purpose: a calibration runs while the node is
    // INACTIVE - that is the interlock - so its services and state topic have to work there.
    // The publisher's QoS exactly (system_ros_interface.cpp): reliable, volatile, depth 1. It
    // used to be transient_local, which handed a freshly configured node a latched sample without
    // waiting for the next cycle - convenient, but a latched sample from a publisher that has
    // since died looks perfectly fresh, and here a stale "engaged" is what grants permission to
    // sweep the sticks. The first sample now costs up to one 20 Hz cycle instead.
    safety_status_subscriber_ = create_subscription<rover_msgs::msg::SafetyStatus>(
        kSafetyStatusTopic, rclcpp::QoS(1).reliable().durability_volatile(),
        [this](const rover_msgs::msg::SafetyStatus & msg) {
            // An unhealthy link means the fields are last-known-good rather than current. Drop
            // the sample rather than age it: "cannot verify" is the honest answer, and it is what
            // an absent sample already produces.
            if (!msg.link_healthy) {
                last_safety_io_.reset();
                last_safety_io_at_.reset();
                updateCalibrationEStop();
                return;
            }

            last_safety_io_ = toSafetyIoFlags(msg);
            last_safety_io_at_ = std::chrono::steady_clock::now();
            updateCalibrationEStop();
        });

    e_stop_state_timeout_ = std::chrono::milliseconds(
        static_cast<int>(get_parameter("e_stop_state_timeout_s").as_double() * 1000.0));

    // Ages the last sample even when nothing is arriving and nothing is calibrating, so the page
    // shows "unverified" rather than a stale "engaged" after the hardware interface goes away.
    e_stop_watchdog_timer_ = create_wall_timer(
        e_stop_state_timeout_ / 2, [this]() { updateCalibrationEStop(); });

    calibration_ = std::make_unique<CalibrationUseCase>(
        config->calibration, axisChannels(*config), calibration_store_,
        // Explicit: TeleopControlPort is a private base, so the conversion has to happen here,
        // inside the class, rather than inside make_unique's forwarding.
        static_cast<TeleopControlPort &>(*this),
        std::chrono::seconds(get_parameter("calibration_timeout_s").as_int()),
        std::chrono::milliseconds(
            static_cast<int>(get_parameter("e_stop_grace_s").as_double() * 1000.0)));
    createCalibrationInterfaces();
    publishCalibrationState();

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

void RoverCrsfTeleopNode::createCalibrationInterfaces()
{
    using Trigger = std_srvs::srv::Trigger;
    using SetCalibration = rover_msgs::srv::SetRcCalibration;
    using StartCalibration = rover_msgs::srv::StartRcCalibration;

    // Reliable + transient-local, the opposite of the rc/* echoes: a page opening mid-session
    // must immediately learn that a calibration is running and that teleop is held off, and a
    // dropped phase change would leave its wizard out of step with the node.
    const auto state_qos = rclcpp::QoS(1).reliable().transient_local();
    calibration_state_publisher_ =
        create_publisher<rover_msgs::msg::RcCalibrationState>(kCalibrationStateTopic, state_qos);

    // Every handler returns immediately. Frames are decoded in the rc/raw subscription callback
    // on this same single-threaded executor, so a handler that waited for samples would stop the
    // frames it was waiting for and deadlock the node.
    calibration_start_service_ = create_service<StartCalibration>(
        kCalibrationStartService,
        [this](
            const std::shared_ptr<StartCalibration::Request> request,
            std::shared_ptr<StartCalibration::Response> response) {
            const SteadyTime now = std::chrono::steady_clock::now();
            const CalibrationOutcome outcome =
                calibration_->start(request->e_stop_confirmed, eStopState(now), now);
            response->success = outcome.ok;
            response->message = outcome.message;

            if (outcome.ok) {
                RCLCPP_WARN(
                    get_logger(),
                    "RC calibration started: teleop is held off until it finishes or times out.");
                startCalibrationHeartbeat();
            }

            publishCalibrationState();
        });

    calibration_sweep_service_ = create_service<Trigger>(
        kCalibrationSweepService,
        [this](
            const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
            const CalibrationOutcome outcome = calibration_->beginSweep();
            response->success = outcome.ok;
            response->message = outcome.message;
            publishCalibrationState();
        });

    calibration_finish_service_ = create_service<Trigger>(
        kCalibrationFinishService,
        [this](
            const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
            const CalibrationOutcome outcome = calibration_->finish();
            response->success = outcome.ok;
            response->message = outcome.message;
            publishCalibrationState();
        });

    calibration_cancel_service_ = create_service<Trigger>(
        kCalibrationCancelService,
        [this](
            const std::shared_ptr<Trigger::Request>, std::shared_ptr<Trigger::Response> response) {
            const CalibrationOutcome outcome = calibration_->cancel();
            response->success = outcome.ok;
            response->message = outcome.message;
            stopCalibrationHeartbeat();
            publishCalibrationState();
        });

    calibration_apply_service_ = create_service<SetCalibration>(
        kCalibrationApplyService,
        [this](
            const std::shared_ptr<SetCalibration::Request> request,
            std::shared_ptr<SetCalibration::Response> response) {
            // An all-zero calibration means "apply what you measured"; anything else is the
            // operator having adjusted a value before applying it.
            const ChannelCalibration requested = fromRcCalibrationMsg(request->calibration);
            const bool supplied = requested.in_max != std::array<int, RcFrame::kChannelCount>{};

            const CalibrationOutcome outcome =
                calibration_->apply(supplied ? &requested : nullptr, request->persist);
            response->success = outcome.ok;
            response->message = outcome.message;

            if (outcome.ok) {
                RCLCPP_INFO(get_logger(), "%s", outcome.message.c_str());
                stopCalibrationHeartbeat();
            }

            publishCalibrationState();
        });
}

void RoverCrsfTeleopNode::startCalibrationHeartbeat()
{
    const double rate_hz = get_parameter("calibration_state_rate_hz").as_double();
    const auto period = std::chrono::duration<double>(1.0 / std::max(0.1, rate_hz));

    // Also re-evaluates the E-Stop: the subscription callback covers a released button, but a
    // publisher that stops entirely produces no callback at all, and that has to time out into
    // kUnknown and cancel the session too.
    calibration_state_timer_ = create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(period),
        [this]() {
            updateCalibrationEStop();
            publishCalibrationState();
        });
}

void RoverCrsfTeleopNode::stopCalibrationHeartbeat()
{
    // Nothing to stream at idle: the transient-local publisher has already latched the final
    // state for whoever connects next.
    if (calibration_state_timer_) {
        calibration_state_timer_->cancel();
        calibration_state_timer_.reset();
    }
}

EStopState RoverCrsfTeleopNode::eStopState(const SteadyTime now) const
{
    // Arrival time, not SafetyStatus.io_sample_time: the two differ by at most one poll period
    // and arrival is what detects a publisher that has stopped. Anything older than the timeout
    // is "cannot verify" rather than "still whatever it was".
    if (!last_safety_io_.has_value() || !last_safety_io_at_.has_value() ||
        (now - *last_safety_io_at_) > e_stop_state_timeout_)
    {
        return EStopState::kUnknown;
    }

    return isSafeToCalibrate(*last_safety_io_) ? EStopState::kEngaged : EStopState::kReleased;
}

void RoverCrsfTeleopNode::updateCalibrationEStop()
{
    if (!calibration_) {
        return;
    }

    const SteadyTime now = std::chrono::steady_clock::now();
    const bool was_running = calibration_->sessionInProgress();
    const EStopState e_stop = eStopState(now);

    calibration_->onEStop(e_stop, now);

    if (was_running && !calibration_->sessionInProgress()) {
        RCLCPP_WARN(
            get_logger(),
            "RC calibration cancelled: the E-Stop is no longer engaged. Teleop is no longer held "
            "off.");
        stopCalibrationHeartbeat();
        publishCalibrationState();
        reported_e_stop_ = e_stop;
        return;
    }

    // On change only. safety_status arrives at 20 Hz and this message is not small; the page needs
    // to know when the button moves, not that it is still where it was.
    if (e_stop != reported_e_stop_) {
        reported_e_stop_ = e_stop;
        publishCalibrationState();
    }
}

void RoverCrsfTeleopNode::publishCalibrationState()
{
    if (!calibration_ || !calibration_state_publisher_) {
        return;
    }

    calibration_state_publisher_->publish(toRcCalibrationStateMsg(
        calibration_->snapshot(std::chrono::steady_clock::now()), this->now()));
}

void RoverCrsfTeleopNode::setTeleopInhibited(const bool inhibited)
{
    if (!use_case_) {
        return;
    }

    use_case_->setCommandInhibited(inhibited);

    if (!inhibited) {
        // No frames reached the switch debouncers while the inhibit was set, and a calibration
        // sweep walks the E-Stop switch through both ends. Without this, the first frame
        // afterwards looks like a real edge and fires an E-Stop service call.
        use_case_->rearmSwitches();
    }
}

bool RoverCrsfTeleopNode::rebuildTeleop(const ChannelCalibration & calibration, std::string & reason)
{
    if (teleopCouldCommand()) {
        // Defence in depth - on_activate already refuses while a session is in progress, so this
        // is not reachable through the services. Kept because rebuilding resets the link monitor:
        // while active it would publish a spurious zero and log a link loss until the next
        // LINK_STATISTICS frame, which is a bad thing to discover through a future refactor.
        reason = "the node is active; deactivate it first";
        return false;
    }

    if (!velocity_publisher_ || !safety_switch_) {
        reason = "the node is not configured";
        return false;
    }

    // Applied to base_config_, never to the config currently in force: the endpoints a
    // calibration carries replace the previous ones, they do not compound on them.
    const TeleopConfig config = applyCalibration(base_config_, calibration);

    // Rebuilt rather than mutated: TeleopConfig is fanned out at construction into the link
    // monitor and both switch debouncers, so assigning the config alone would leave those on the
    // old values. The ports are reused, so the cmd_vel publisher keeps its activation state.
    use_case_ = std::make_unique<TeleopUseCase>(config, velocity_publisher_, safety_switch_);

    // Mirror it into the parameters, so `ros2 param get` agrees with what the rover is using.
    set_parameters(
        {rclcpp::Parameter("channel_in_min", toParameterArray(calibration.in_min)),
         rclcpp::Parameter("channel_in_mid", toParameterArray(calibration.in_mid)),
         rclcpp::Parameter("channel_in_max", toParameterArray(calibration.in_max)),
         rclcpp::Parameter("channel_deadband", toParameterArray(calibration.deadband))});

    calibration_source_ = "a calibration applied at run time";
    return true;
}

bool RoverCrsfTeleopNode::teleopCouldCommand() const
{
    return get_current_state().id() == lifecycle_msgs::msg::State::PRIMARY_STATE_ACTIVE;
}

void RoverCrsfTeleopNode::onRcChannels(const RcFrame & frame)
{
    const SteadyTime now = std::chrono::steady_clock::now();

    // Before the teleop rules, and unconditionally: the calibration page's channel bars come
    // from this even when no session is running, and the peak hold during a sweep is the reason
    // the measurement happens here instead of in a browser.
    const bool was_running = calibration_->sessionInProgress();
    calibration_->onFrame(frame, now);

    if (was_running && !calibration_->sessionInProgress()) {
        RCLCPP_WARN(get_logger(), "RC calibration timed out; teleop is no longer held off.");
        stopCalibrationHeartbeat();
        publishCalibrationState();
    }

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
    // Second, independent interlock: the node refuses to become able to command at all, rather
    // than becoming active and relying on the inhibit to hold. The sweep is at full throw.
    if (calibration_ && calibration_->sessionInProgress()) {
        RCLCPP_ERROR(
            get_logger(),
            "Refusing to activate: an RC calibration is in progress. Finish or cancel it first "
            "(%s).", kCalibrationCancelService);
        return CallbackReturn::FAILURE;
    }

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
    // A session in progress holds the inhibit, and the use case it would release it on is about
    // to be destroyed. Ending it here keeps the next configure from starting out inhibited.
    if (calibration_ && calibration_->sessionInProgress()) {
        RCLCPP_WARN(get_logger(), "Cancelling the RC calibration in progress: the node is being "
                                  "cleaned up.");
        calibration_->cancel();
    }

    stopCalibrationHeartbeat();
    calibration_.reset();
    calibration_store_.reset();
    e_stop_watchdog_timer_.reset();
    safety_status_subscriber_.reset();
    last_safety_io_.reset();
    last_safety_io_at_.reset();
    calibration_state_publisher_.reset();
    calibration_start_service_.reset();
    calibration_sweep_service_.reset();
    calibration_finish_service_.reset();
    calibration_cancel_service_.reset();
    calibration_apply_service_.reset();

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
        case TickStatus::kInhibited:
            RCLCPP_WARN(
                get_logger(), "RC calibration in progress: sent a zero command, teleop is silent.");
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

void RoverCrsfTeleopNode::diagnoseCalibration(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    // An abandoned calibration holds teleop off, and from the outside that looks exactly like a
    // rover that has stopped responding to the transmitter. It has to be visible here.
    if (!calibration_) {
        status.summary(
            diagnostic_msgs::msg::DiagnosticStatus::OK, "Not configured; nothing calibrating.");
        return;
    }

    const CalibrationSnapshot snapshot = calibration_->snapshot(std::chrono::steady_clock::now());

    status.add("Calibration in effect", calibration_source_);

    // The gate, and how old the evidence behind it is. A calibration that will not start is
    // almost always one of these two lines.
    const SteadyTime e_stop_now = std::chrono::steady_clock::now();
    switch (eStopState(e_stop_now)) {
        case EStopState::kEngaged:
            status.add("E-Stop", "engaged (calibration allowed)");
            break;
        case EStopState::kReleased:
            status.add("E-Stop", "released (calibration refused)");
            break;
        case EStopState::kUnknown:
            status.add("E-Stop", "unverified (calibration refused)");
            break;
    }

    status.add(
        "Safety IO age (ms)",
        last_safety_io_at_.has_value()
            ? std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
                                 e_stop_now - *last_safety_io_at_).count())
            : std::string("never received"));
    status.add("Persisted to", calibration_store_ ? calibration_store_->location() : "(disabled)");
    // All three endpoints come from the calibration in force, never from base_config_.
    // base_config_ deliberately keeps the *uncalibrated* endpoints - applyCalibration() is always
    // written against it - so pairing its in_min/in_max with the active in_mid reported a triple
    // that no calibration ever held. The channel number still comes from base_config_: that is a
    // parameter, not something a calibration measures.
    const auto linear_x_index = static_cast<std::size_t>(base_config_.linear_x_channel - 1);
    status.add(
        "linear_x endpoints",
        std::to_string(snapshot.active.in_min[linear_x_index]) + " / " +
            std::to_string(snapshot.active.in_mid[linear_x_index]) + " / " +
            std::to_string(snapshot.active.in_max[linear_x_index]));

    if (!snapshot.teleop_inhibited) {
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Idle.");
        return;
    }

    status.add("Samples", static_cast<int>(snapshot.samples));
    status.add("Seconds before it times out", static_cast<int>(snapshot.remaining_s));

    // WARN, never ERROR - like every other task here, because RC teleop is optional.
    status.summary(
        diagnostic_msgs::msg::DiagnosticStatus::WARN,
        "Calibration in progress: teleop is held off. " + snapshot.message);
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
