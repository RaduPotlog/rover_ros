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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_ERROR_FILTER_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_ERROR_FILTER_HPP_

#include <atomic>
#include <map>
#include <string>

namespace rover_hardware_interface
{

// Debounces a single boolean error source: `current_error == true` on `max_error_count_`
// consecutive calls latches `error_` true. A `false` call resets the consecutive-run counter but
// does NOT un-latch an already-set `error_` - only clearError() does that.
class ErrorFilter
{

public:

    explicit ErrorFilter(const unsigned max_error_count) : max_error_count_(max_error_count) {}

    bool isError() const { return error_; }

    void updateError(const bool current_error);

    void clearError();

private:

    const unsigned max_error_count_;
    unsigned current_error_count_ = 0;
    bool error_ = false;
};

enum class ErrorsFilterIds
{
    WRITE_CMDS = 0,
    READ_MOTOR_STATES,
    READ_DRIVER_STATE,
    FAULT_FLAG,
};

const std::map<ErrorsFilterIds, std::string> kErrorFilterIdNames = {
    {ErrorsFilterIds::WRITE_CMDS,        "write_cmds_error"},
    {ErrorsFilterIds::READ_MOTOR_STATES, "read_motor_states_error"},
    {ErrorsFilterIds::READ_DRIVER_STATE, "read_driver_state_error"},
    {ErrorsFilterIds::FAULT_FLAG,        "fault_flag_error"},
};

// Aggregates one ErrorFilter per ErrorsFilterIds category. updateError()/isError() must always be
// called from the same thread (the RT control loop); setClearErrorsFlag() may be called from any
// thread (e.g. a ROS service callback) - the actual clear is applied lazily on the next
// updateError() call via `clear_errors_`, an atomic_bool.
class RoverErrorFilter
{

public:

    RoverErrorFilter(
        const unsigned max_write_cmds_errors_count,
        const unsigned max_read_motor_states_errors_count,
        const unsigned max_read_driver_state_errors_count,
        const unsigned max_fault_flag_errors_count);

    bool isError() const;

    bool isError(const ErrorsFilterIds id) const { return error_filters_.at(id).isError(); }

    void updateError(const ErrorsFilterIds id, const bool current_error);

    std::map<std::string, bool> getErrorMap() const;

    void setClearErrorsFlag() { clear_errors_.store(true); }

private:

    void clearErrorsIfFlagSet();

    std::atomic_bool clear_errors_ = false;
    std::map<ErrorsFilterIds, ErrorFilter> error_filters_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_ROVER_ERROR_FILTER_HPP_
