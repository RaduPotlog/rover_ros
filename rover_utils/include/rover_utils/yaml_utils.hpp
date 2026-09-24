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

#ifndef ROVER_UTILS_YAML_UTILS_HPP_
#define ROVER_UTILS_YAML_UTILS_HPP_

#include <stdexcept>
#include <string>

#include "yaml-cpp/yaml.h"

namespace rover_utils
{

// Thrown by getYAMLKeyValue() when the key is absent, as opposed to present but not convertible.
// A std::runtime_error, so existing catches still see it.
class MissingYAMLKeyError : public std::runtime_error
{
public:
    explicit MissingYAMLKeyError(const std::string & key)
    : std::runtime_error("Missing '" + key + "' in description.")
    {
    }
};

template <typename T>
T getYAMLKeyValue(const YAML::Node & description, const std::string & key)
{
    if (!description[key]) {
        throw MissingYAMLKeyError(key);
    }
    
    try {
        return description[key].as<T>();
    } catch (const YAML::BadConversion & e) {
        throw std::runtime_error("Failed to convert '" + static_cast<std::string>(key) + "' key.");
    }
}

template <typename T>
T getYAMLKeyValue(const YAML::Node & description, const std::string & key, const T default_value)
{
    try {
        return getYAMLKeyValue<T>(description, key);
    } catch (const MissingYAMLKeyError &) {
        return default_value;
    }
}

}  // namespace rover_utils

#endif  // ROVER_UTILS_YAML_UTILS_HPP_
