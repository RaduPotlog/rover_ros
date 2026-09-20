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

#include "rover_crsf_teleop/infrastructure/yaml_calibration_store.hpp"

#include <array>
#include <chrono>
#include <ctime>
#include <fstream>
#include <system_error>
#include <utility>

#include <rclcpp/logging.hpp>

#include "yaml-cpp/yaml.h"

namespace rover_crsf_teleop
{

namespace
{

constexpr const char * kMinKey = "channel_in_min";
constexpr const char * kMidKey = "channel_in_mid";
constexpr const char * kMaxKey = "channel_in_max";
constexpr const char * kDeadbandKey = "channel_deadband";

std::string utcNow()
{
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    std::tm tm{};
    gmtime_r(&now, &tm);

    std::array<char, 32> buffer{};
    std::strftime(buffer.data(), buffer.size(), "%Y-%m-%dT%H:%M:%SZ", &tm);
    return std::string(buffer.data());
}

// Reads one 16-element array. Any other length is a malformed file, not something to pad.
bool readChannelArray(
    const YAML::Node & root, const char * key, std::array<int, RcFrame::kChannelCount> & out,
    std::string & error)
{
    const YAML::Node node = root[key];

    if (!node || !node.IsSequence() || node.size() != RcFrame::kChannelCount) {
        error = std::string("'") + key + "' must be a sequence of " +
                std::to_string(RcFrame::kChannelCount) + " values";
        return false;
    }

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        out[i] = node[i].as<int>();
    }

    return true;
}

void writeChannelArray(
    YAML::Emitter & emitter, const char * key,
    const std::array<int, RcFrame::kChannelCount> & values)
{
    emitter << YAML::Key << key << YAML::Value << YAML::Flow << YAML::BeginSeq;
    for (const int value : values) {
        emitter << value;
    }
    emitter << YAML::EndSeq;
}

}  // namespace

YamlCalibrationStore::YamlCalibrationStore(std::filesystem::path path, rclcpp::Logger logger)
: path_(std::move(path)), logger_(std::move(logger))
{
}

std::optional<StoredCalibration> YamlCalibrationStore::load()
{
    std::error_code ec;
    if (!std::filesystem::exists(path_, ec)) {
        return std::nullopt;
    }

    StoredCalibration stored;
    std::string error;

    // yaml-cpp throws on malformed input, and a bad calibration file must not take the node down
    // on boot - it falls back to the shipped parameters instead.
    try {
        const YAML::Node root = YAML::LoadFile(path_.string())["rc_calibration"];

        if (!root || !root.IsMap()) {
            error = "no 'rc_calibration' mapping";
        } else if (!root["schema_version"] ||
                   root["schema_version"].as<int>() != kCalibrationSchemaVersion)
        {
            error = "unsupported schema_version (this build reads " +
                    std::to_string(kCalibrationSchemaVersion) + ")";
        } else if (readChannelArray(root, kMinKey, stored.calibration.in_min, error) &&
                   readChannelArray(root, kMidKey, stored.calibration.in_mid, error) &&
                   readChannelArray(root, kMaxKey, stored.calibration.in_max, error) &&
                   readChannelArray(root, kDeadbandKey, stored.calibration.deadband, error))
        {
            stored.schema_version = kCalibrationSchemaVersion;
            stored.created = root["created"] ? root["created"].as<std::string>() : "";
            return stored;
        }
    } catch (const std::exception & exception) {
        error = exception.what();
    }

    RCLCPP_WARN(
        logger_, "Ignoring the RC calibration at '%s': %s. Using the configured values instead.",
        path_.c_str(), error.c_str());
    return std::nullopt;
}

bool YamlCalibrationStore::save(const ChannelCalibration & calibration, std::string & error)
{
    std::error_code ec;
    if (!path_.parent_path().empty()) {
        std::filesystem::create_directories(path_.parent_path(), ec);
        if (ec) {
            error = "cannot create " + path_.parent_path().string() + ": " + ec.message();
            return false;
        }
    }

    YAML::Emitter emitter;
    emitter << YAML::BeginMap << YAML::Key << "rc_calibration" << YAML::Value << YAML::BeginMap;
    emitter << YAML::Key << "schema_version" << YAML::Value << kCalibrationSchemaVersion;
    emitter << YAML::Key << "created" << YAML::Value << utcNow();
    writeChannelArray(emitter, kMinKey, calibration.in_min);
    writeChannelArray(emitter, kMidKey, calibration.in_mid);
    writeChannelArray(emitter, kMaxKey, calibration.in_max);
    writeChannelArray(emitter, kDeadbandKey, calibration.deadband);
    emitter << YAML::EndMap << YAML::EndMap;

    // Temp file then rename: same directory, so the rename is atomic and a reader can never see
    // a partially written calibration.
    const std::filesystem::path temporary = path_.string() + ".tmp";

    {
        std::ofstream out(temporary, std::ios::trunc);
        if (!out) {
            error = "cannot write " + temporary.string();
            return false;
        }

        out << "# rover_crsf_teleop RC calibration - WRITTEN BY THE NODE.\n"
               "#\n"
               "# Measured by the calibration flow (rc/calibration/*) and applied on top of the\n"
               "# channel_in_* parameters in config/rover_crsf_teleop.yaml. Delete this file to\n"
               "# go back to the shipped values.\n"
               "#\n"
               "# Raw 11-bit CRSF counts, index N-1 = channel N. Hand-edit only if you have read\n"
               "# domain/stick_mapping.hpp: in_min < in_mid < in_max, and each half-throw must\n"
               "# clear the deadband, or that direction of the axis is silently dead.\n";
        out << emitter.c_str() << "\n";
        out.flush();

        if (!out) {
            error = "failed while writing " + temporary.string();
            return false;
        }
    }

    std::filesystem::rename(temporary, path_, ec);
    if (ec) {
        error = "cannot replace " + path_.string() + ": " + ec.message();
        std::filesystem::remove(temporary, ec);
        return false;
    }

    RCLCPP_INFO(logger_, "Saved the RC calibration to '%s'.", path_.c_str());
    return true;
}

}  // namespace rover_crsf_teleop
