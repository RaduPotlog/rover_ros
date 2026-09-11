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

#ifndef ROVER_LED_DOMAIN_SK9822_FRAME_ENCODER_HPP_
#define ROVER_LED_DOMAIN_SK9822_FRAME_ENCODER_HPP_

#include <cstdint>
#include <vector>

namespace rover_led
{

// Encodes an RGBA8 frame into the SK9822 LED protocol: a 4-byte start frame
// (0x00), one [0xE0 | brightness, B, G, R] word per LED with gamma and colour
// correction, and a 4-byte end frame (0xFF). Alpha scales the 5-bit global
// brightness.
class SK9822FrameEncoder
{

public:

    // brightness in [0.0, 1.0]; throws std::out_of_range otherwise.
    void setGlobalBrightness(const float brightness);

    // brightness in [0, 31]; throws std::out_of_range otherwise.
    void setGlobalBrightness(const std::uint8_t brightness);

    std::uint8_t getGlobalBrightness() const
    {
        return static_cast<std::uint8_t>(global_brightness_);
    }

    // SK9822 wire order, as clocked out over SPI.
    std::vector<std::uint8_t> encode(const std::vector<std::uint8_t> & rgba_frame) const;

    // The rover's UDP-to-SPI LED bridge expects each 4-byte word rotated to
    // [B, G, R, 0xE0 | brightness].
    std::vector<std::uint8_t> encodeForUdpBridge(const std::vector<std::uint8_t> & rgba_frame) const;

private:

    static constexpr std::uint16_t kCorrRed = 245;
    static constexpr std::uint16_t kCorrGreen = 255;
    static constexpr std::uint16_t kCorrBlue = 240;
    static constexpr float kCorrectionGamma = 2.2f;

    std::uint16_t global_brightness_ = 31;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_SK9822_FRAME_ENCODER_HPP_
