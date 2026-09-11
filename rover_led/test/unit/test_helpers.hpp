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

#ifndef ROVER_LED_TEST_UNIT_TEST_HELPERS_HPP_
#define ROVER_LED_TEST_UNIT_TEST_HELPERS_HPP_

#include <array>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "yaml-cpp/yaml.h"

#include "rover_led/domain/animation/animation.hpp"

namespace rover_led::test
{

using Rgba = std::array<std::uint8_t, 4>;

// Paints every LED with one colour; the red channel carries the iteration
// index so tests can tell frames apart. Can be told to fail on demand.
class StubAnimation : public Animation
{

public:

    StubAnimation(const Rgba color = {0, 0, 0, 255}) : color_(color) {}

    void initialize(
        const YAML::Node & animation_description,
        const std::size_t num_led,
        const float controller_frequency) override
    {
        if (animation_description["fail_initialize"]) {
            throw std::runtime_error("stub initialize failure");
        }

        Animation::initialize(animation_description, num_led, controller_frequency);
    }

    void setParam(const std::string & param) override
    {
        param_ = param;
    }

    const std::string & param() const
    {
        return param_;
    }

    bool fail_update = false;

protected:

    std::vector<std::uint8_t> updateFrame() override
    {
        if (fail_update) {
            throw std::runtime_error("stub update failure");
        }

        std::vector<std::uint8_t> frame;

        for (std::size_t i = 0; i < getNumberOfLeds(); i++) {
            frame.push_back(static_cast<std::uint8_t>(color_[0] + getAnimationIteration()));
            frame.push_back(color_[1]);
            frame.push_back(color_[2]);
            frame.push_back(color_[3]);
        }

        return frame;
    }

private:

    Rgba color_;
    std::string param_;
};

// Duration in seconds; at 10 Hz one second is 10 frames.
inline YAML::Node stubDescription(const float duration = 1.0f, const std::size_t repeat = 1)
{
    YAML::Node description;
    description["duration"] = duration;
    description["repeat"] = repeat;

    return description;
}

inline std::shared_ptr<StubAnimation> makeStub(
    const std::size_t num_led,
    const Rgba color = {0, 0, 0, 255},
    const float duration = 1.0f,
    const std::size_t repeat = 1)
{
    auto animation = std::make_shared<StubAnimation>(color);
    animation->initialize(stubDescription(duration, repeat), num_led, 10.0f);

    return animation;
}

inline Rgba pixel(const std::vector<std::uint8_t> & frame, const std::size_t led)
{
    return {frame.at(led * 4), frame.at(led * 4 + 1), frame.at(led * 4 + 2), frame.at(led * 4 + 3)};
}

}  // namespace rover_led::test

#endif  // ROVER_LED_TEST_UNIT_TEST_HELPERS_HPP_
