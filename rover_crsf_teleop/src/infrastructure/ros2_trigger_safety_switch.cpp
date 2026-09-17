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

#include "rover_crsf_teleop/infrastructure/ros2_trigger_safety_switch.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace rover_crsf_teleop
{

namespace
{

std::shared_ptr<SafetyRequestStatus> makeStatus(const char * description)
{
    auto status = std::make_shared<SafetyRequestStatus>();
    status->description = description;
    return status;
}

}  // namespace

Ros2TriggerSafetySwitch::Ros2TriggerSafetySwitch(rclcpp_lifecycle::LifecycleNode & node)
: logger_(node.get_logger()),
  clock_(node.get_clock()),
  e_stop_set_(node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_set")),
  e_stop_reset_(
      node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_user_e_stop_reset")),
  e_stop_latch_reset_(
      node.create_client<std_srvs::srv::Trigger>("hardware_interface/sw_e_stop_latch_reset")),
  e_stop_set_status_(makeStatus("SW User E-Stop set")),
  e_stop_reset_status_(makeStatus("SW User E-Stop reset")),
  e_stop_latch_reset_status_(makeStatus("SW E-Stop latch reset"))
{
}

void Ros2TriggerSafetySwitch::requestUserEStopSet()
{
    callTriggerService(e_stop_set_, e_stop_set_status_);
}

void Ros2TriggerSafetySwitch::requestUserEStopReset()
{
    callTriggerService(e_stop_reset_, e_stop_reset_status_);
}

void Ros2TriggerSafetySwitch::requestLatchReset()
{
    callTriggerService(e_stop_latch_reset_, e_stop_latch_reset_status_);
}

std::vector<SafetyRequestStatus> Ros2TriggerSafetySwitch::requestStatuses() const
{
    std::vector<SafetyRequestStatus> statuses;

    for (const auto & [client, status] : {
             std::make_pair(e_stop_set_, e_stop_set_status_),
             std::make_pair(e_stop_reset_, e_stop_reset_status_),
             std::make_pair(e_stop_latch_reset_, e_stop_latch_reset_status_)}) {
        SafetyRequestStatus copy = *status;
        copy.service_ready = client->service_is_ready();
        statuses.push_back(std::move(copy));
    }

    return statuses;
}

void Ros2TriggerSafetySwitch::callTriggerService(
    const TriggerClient::SharedPtr & client, const std::shared_ptr<SafetyRequestStatus> & status)
{
    const std::string & description = status->description;

    if (!client->service_is_ready()) {
        status->outcome = SafetyRequestOutcome::kUnavailable;
        status->message.clear();
        RCLCPP_WARN_THROTTLE(
            logger_, *clock_, 1000,
            "Service for '%s' is not available; request dropped.", description.c_str());
        return;
    }

    auto request = std::make_shared<std_srvs::srv::Trigger::Request>();
    status->outcome = SafetyRequestOutcome::kPending;
    status->message.clear();

    // The response used to be discarded, so a refused request (e.g. the hardware interface
    // rejecting an E-Stop reset because the rover is still being commanded to move) was
    // invisible to the operator - and, because the request only re-fires on a switch change,
    // there was no retry either. Report it, in the log and on diagnostics.
    client->async_send_request(
        request,
        [logger = logger_, status](TriggerClient::SharedFuture future) {
            const auto response = future.get();

            if (response->success) {
                status->outcome = SafetyRequestOutcome::kSucceeded;
                status->message.clear();
                RCLCPP_INFO(logger, "%s succeeded.", status->description.c_str());
            } else {
                status->outcome = SafetyRequestOutcome::kRefused;
                status->message = response->message;
                RCLCPP_WARN(
                    logger, "%s was refused: %s", status->description.c_str(),
                    response->message.c_str());
            }
        });
}

}  // namespace rover_crsf_teleop
