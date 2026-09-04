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

#ifndef ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_
#define ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_

#include <map>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

#include "rover_hardware_interface/rover_modbus/modbus_types.hpp"
#include "rover_hardware_interface/rover_system/rover_system.hpp"

namespace rover_hardware_interface
{

class RoverA1System : public RoverSystem
{

public:

    RoverA1System();

    ~RoverA1System() = default;

    /**
     * @brief Adds key-value pairs to a diagnostic status if the value is true.
     *
     * This function iterates over a map of key-value pairs and adds each pair to the diagnostic status
     * if the value is true. The key prefix and suffix can be optionally provided to modify the keys
     * before adding them to the status.
     *
     * @tparam ValueT The type of the values in the map, must be evaluable to boolean context.
     * @param status The diagnostic status wrapper to add the key-value pairs to.
     * @param kv_map The map of key-value pairs.
     * @param key_prefix The optional prefix to add to each key.
     * @param key_suffix The optional suffix to add to each key.
     */
    template <typename ValueT>
    void addKeyValueIfTrue(
    diagnostic_updater::DiagnosticStatusWrapper & status,
    const std::map<std::string, ValueT> & kv_map, const std::string & key_prefix = "",
    const std::string & key_suffix = "")
    {
        for (const auto & [key, value] : kv_map) {
            if (static_cast<bool>(value)) {
                status.add(key_prefix + key + key_suffix, value);
            }
        }
    }

protected:

    void defineRoverDriver() override;

    // Reads the Modbus TCP endpoint (host/port/retry) that RoverA1System's safety controller
    // uses - see rover_system.hpp's declaration for why this is per-variant rather than generic.
    void readRoverControllerSettings() override;

    // Constructs the Modbus-backed RoverSafetyController and the two domain-port adapters
    // (RoverGpioPort, EmergencyStopIoPort) over it. A future rover variant using a different
    // safety-IO transport would provide its own implementation instead.
    void defineRoverController() override;

    void updateHwStates(const rclcpp::Time & time) override;

    void updateDriverStateMsg() override;

    void getSpeedCmd(std::vector<float> & speed_cmd) const override;

    void diagnoseErrors(diagnostic_updater::DiagnosticStatusWrapper & status) override;
    void diagnoseStatus(diagnostic_updater::DiagnosticStatusWrapper & status) override;

    static const inline std::vector<std::string> joints_ = {"fl", "fr", "rl", "rr"};

    // Modbus TCP endpoint for the safety controller (E-Stop / GPIO), read from URDF. Owned here
    // (not by RoverSystem) because it is specific to this variant's Modbus-backed safety
    // controller - see defineRoverController()/readRoverControllerSettings().
    ModbusSettings modbus_settings_;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_ROVER_SYSTEM_ROVER_A1_SYSTEM_HPP_
