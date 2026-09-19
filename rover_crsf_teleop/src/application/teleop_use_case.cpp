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

#include "rover_crsf_teleop/application/teleop_use_case.hpp"

#include <utility>

namespace rover_crsf_teleop
{

TeleopUseCase::TeleopUseCase(
    const TeleopConfig & config,
    std::shared_ptr<VelocityCommandPort> velocity_port,
    std::shared_ptr<SafetySwitchPort> safety_switch_port)
: config_(config),
  velocity_port_(std::move(velocity_port)),
  safety_switch_port_(std::move(safety_switch_port)),
  link_monitor_(config.link),
  e_stop_switch_(config.channel_switch_threshold, config.switch_settle_frames),
  latch_reset_switch_(config.channel_switch_threshold, config.switch_settle_frames)
{
}

void TeleopUseCase::onChannels(const RcFrame & frame, const SteadyTime now)
{
    last_frame_ = frame;
    link_monitor_.onChannels(now);
}

void TeleopUseCase::onLinkStats(const std::uint8_t link_quality, const SteadyTime now)
{
    link_monitor_.onLinkStats(now, link_quality);
}

TickStatus TeleopUseCase::tick(const SteadyTime now)
{
    if (!last_frame_.has_value()) {
        return TickStatus::kWaitingForFirstFrame;
    }

    if (!link_monitor_.isHealthy(now)) {
        publish(VelocityCommand{});
        return TickStatus::kLinkLost;
    }

    VelocityCommand command;
    command.linear_x = mapChannel(config_.linear_x_channel, config_.linear_x_mapping);
    command.angular_z = mapChannel(config_.angular_z_channel, config_.angular_z_mapping);
    publish(limitRimSpeed(command, config_.max_wheel_rim_speed, config_.half_track_width));

    evaluateSwitches();

    return TickStatus::kActive;
}

void TeleopUseCase::stop()
{
    publish(VelocityCommand{});
}

void TeleopUseCase::publish(const VelocityCommand & command)
{
    // A centred stick maps to exactly 0.0 (see domain/stick_mapping.hpp), so "zero" here really
    // means "released", not "nearly released".
    if (command.isZero() && zero_sent_) {
        return;
    }

    velocity_port_->publish(command);
    zero_sent_ = command.isZero();
    last_command_ = command;
}

TeleopDiagnostics TeleopUseCase::diagnostics(const SteadyTime now) const
{
    TeleopDiagnostics diagnostics;
    diagnostics.first_frame_received = last_frame_.has_value();
    diagnostics.link = link_monitor_.snapshot(now);
    diagnostics.health = evaluateTeleopHealth(diagnostics.first_frame_received, diagnostics.link);
    diagnostics.last_command = last_command_;
    diagnostics.e_stop_switch = e_stop_switch_.position();
    diagnostics.latch_reset_switch = latch_reset_switch_.position();
    return diagnostics;
}

double TeleopUseCase::mapChannel(const int channel_number, const AxisMapping & mapping) const
{
    // An invalid channel number is rejected when the node is configured; should one get here
    // anyway, command nothing on that axis rather than whatever a default value would map to.
    const auto raw = last_frame_->channel(channel_number);
    return raw.has_value() ? mapAxis(*raw, mapping) : 0.0;
}

void TeleopUseCase::evaluateSwitches()
{
    if (const auto raw = last_frame_->channel(config_.e_stop_channel)) {
        if (const auto position = e_stop_switch_.update(*raw)) {
            if (*position == SwitchPosition::kLow) {
                safety_switch_port_->requestUserEStopSet();
            } else {
                safety_switch_port_->requestUserEStopReset();
            }
        }
    }

    if (const auto raw = last_frame_->channel(config_.e_stop_latch_reset_channel)) {
        if (latch_reset_switch_.update(*raw) == SwitchPosition::kLow) {
            safety_switch_port_->requestLatchReset();
        }
    }
}

}  // namespace rover_crsf_teleop
