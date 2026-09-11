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

#include "rover_led/domain/sk9822_frame_encoder.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <stdexcept>
#include <vector>

namespace rover_led
{

void SK9822FrameEncoder::setGlobalBrightness(const float brightness)
{
    if (brightness < 0.0f || brightness > 1.0f) {
        throw std::out_of_range("Brightness out of range [0.0, 1.0].");
    }

    setGlobalBrightness(std::uint8_t(ceil(brightness * 31.0f)));
}

void SK9822FrameEncoder::setGlobalBrightness(const std::uint8_t brightness)
{
    if (brightness > 31) {
        throw std::out_of_range("Brightness out of range [0, 31].");
    }

    global_brightness_ = std::uint16_t(brightness);
}

std::vector<std::uint8_t> SK9822FrameEncoder::encode(
    const std::vector<std::uint8_t> & rgba_frame) const
{
    if (rgba_frame.size() % 4 != 0) {
        throw std::runtime_error("Incorrect number of bytes to convert frame.");
    }

    const std::size_t buffer_size = 4 + rgba_frame.size() + 4;
    std::vector<std::uint8_t> buffer(buffer_size);

    std::fill(buffer.begin(), buffer.begin() + 4, 0x00);
    std::fill(buffer.end() - 4, buffer.end(), 0xFF);

    for (std::size_t i = 0; i < rgba_frame.size() / 4; i++) {
        const std::size_t padding = i * 4;
        const std::uint8_t brightness = (std::uint16_t(rgba_frame[padding + 3]) * global_brightness_) / 255;
        buffer[4 + padding] = 0xE0 | brightness;
        buffer[4 + padding + 1] = std::uint8_t(pow(rgba_frame[padding + 2] / 255.0, kCorrectionGamma) * kCorrBlue);
        buffer[4 + padding + 2] = std::uint8_t(pow(rgba_frame[padding + 1] / 255.0, kCorrectionGamma) * kCorrGreen);
        buffer[4 + padding + 3] = std::uint8_t(pow(rgba_frame[padding + 0] / 255.0, kCorrectionGamma) * kCorrRed);
    }

    return buffer;
}

std::vector<std::uint8_t> SK9822FrameEncoder::encodeForUdpBridge(
    const std::vector<std::uint8_t> & rgba_frame) const
{
    auto buffer = encode(rgba_frame);

    // Start/end frames are uniform, so rotating every word leaves them intact.
    for (std::size_t i = 0; i < buffer.size(); i += 4) {
        std::rotate(buffer.begin() + i, buffer.begin() + i + 1, buffer.begin() + i + 4);
    }

    return buffer;
}

}  // namespace rover_led
