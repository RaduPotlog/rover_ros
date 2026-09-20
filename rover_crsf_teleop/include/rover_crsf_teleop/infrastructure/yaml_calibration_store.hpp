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

#ifndef ROVER_CRSF_TELEOP_INFRASTRUCTURE_YAML_CALIBRATION_STORE_HPP_
#define ROVER_CRSF_TELEOP_INFRASTRUCTURE_YAML_CALIBRATION_STORE_HPP_

#include <filesystem>
#include <optional>
#include <string>

#include <rclcpp/logger.hpp>

#include "rover_crsf_teleop/domain/ports.hpp"

namespace rover_crsf_teleop
{

// The schema this adapter writes and is willing to read back. Bump it when the file layout
// changes incompatibly; an unrecognised version is treated as "no calibration" rather than
// guessed at, because guessing wrong mis-maps the sticks.
constexpr int kCalibrationSchemaVersion = 1;

// Keeps the measured calibration in a YAML file of its own, separate from the shipped parameter
// file.
//
// A separate file, not a second ros__parameters overlay, on purpose: a parameter file only takes
// effect on a process restart, and the whole point of the calibration flow is that applying it
// does not restart anything. It also keeps "what we shipped" and "what this rover measured"
// distinguishable when a rover misbehaves in the field.
//
// Writes are atomic (temp file + rename), so a power cut during a save cannot leave a
// half-written file that silently mis-maps the sticks.
class YamlCalibrationStore : public CalibrationStorePort
{

public:

    YamlCalibrationStore(std::filesystem::path path, rclcpp::Logger logger);

    std::optional<StoredCalibration> load() override;

    bool save(const ChannelCalibration & calibration, std::string & error) override;

    std::string location() const override { return path_.string(); }

private:

    std::filesystem::path path_;
    rclcpp::Logger logger_;
};

}  // namespace rover_crsf_teleop

#endif  // ROVER_CRSF_TELEOP_INFRASTRUCTURE_YAML_CALIBRATION_STORE_HPP_
