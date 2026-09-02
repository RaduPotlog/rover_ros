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

#include "rover_hardware_interface/system_error_filter/rover_error_filter.hpp"

#include <algorithm>

namespace rover_hardware_interface
{

void ErrorFilter::updateError(const bool current_error)
{
    if (current_error) {
        ++current_error_count_;

        if (current_error_count_ >= max_error_count_) {
            error_ = true;
        }
    } else {
        current_error_count_ = 0;
    }
}

void ErrorFilter::clearError()
{
    error_ = false;
    current_error_count_ = 0;
}

RoverErrorFilter::RoverErrorFilter(
    const unsigned max_write_cmds_errors_count,
    const unsigned max_read_motor_states_errors_count,
    const unsigned max_read_driver_state_errors_count,
    const unsigned max_fault_flag_errors_count)
{
    error_filters_.emplace(ErrorsFilterIds::WRITE_CMDS, ErrorFilter(max_write_cmds_errors_count));
    error_filters_.emplace(
        ErrorsFilterIds::READ_MOTOR_STATES, ErrorFilter(max_read_motor_states_errors_count));
    error_filters_.emplace(
        ErrorsFilterIds::READ_DRIVER_STATE, ErrorFilter(max_read_driver_state_errors_count));
    error_filters_.emplace(ErrorsFilterIds::FAULT_FLAG, ErrorFilter(max_fault_flag_errors_count));
}

bool RoverErrorFilter::isError() const
{
    return std::any_of(
        error_filters_.begin(), error_filters_.end(),
        [](const auto & entry) { return entry.second.isError(); });
}

void RoverErrorFilter::updateError(const ErrorsFilterIds id, const bool current_error)
{
    clearErrorsIfFlagSet();
    error_filters_.at(id).updateError(current_error);
}

std::map<std::string, bool> RoverErrorFilter::getErrorMap() const
{
    std::map<std::string, bool> error_map;

    for (const auto & [id, filter] : error_filters_) {
        error_map[kErrorFilterIdNames.at(id)] = filter.isError();
    }

    return error_map;
}

void RoverErrorFilter::clearErrorsIfFlagSet()
{
    if (clear_errors_) {
        std::for_each(
            error_filters_.begin(), error_filters_.end(),
            [](auto & entry) { entry.second.clearError(); });
        clear_errors_.store(false);
    }
}

}  // namespace rover_hardware_interface
