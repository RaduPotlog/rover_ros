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

#include "rover_led/sk9822.hpp"

#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace rover_led
{

SK9822::SK9822(
    SPIDeviceInterface::SharedPtr spi_device, 
    const std::string & device_name,
    const std::uint32_t speed, const bool cs_high)
: spi_device_(spi_device)
, device_name_(device_name)
, speed_(speed)
, file_descriptor_(spi_device->open(device_name))
{
    (void)cs_high;
}

SK9822::~SK9822() 
{ 
    spi_device_->close(file_descriptor_); 
}

void SK9822::setGlobalBrightness(const float brightness)
{
    if (brightness < 0.0f || brightness > 1.0f) {
        throw std::out_of_range("Brightness out of range [0.0, 1.0].");
    }

    setGlobalBrightness(std::uint8_t(ceil(brightness * 31.0f)));
}

void SK9822::setGlobalBrightness(const std::uint8_t brightness)
{
    if (brightness > 31) {
        throw std::out_of_range("Brightness out of range [0, 31].");
    }

    global_brightness_ = std::uint16_t(brightness);
}

std::vector<std::uint8_t> SK9822::setPanel(const std::vector<std::uint8_t> & frame) const
{
    const auto buffer = rgbFrameToBGRBuffer(frame);

    return buffer;
}

std::vector<std::uint8_t> SK9822::rgbFrameToBGRBuffer(
    const std::vector<std::uint8_t> & frame) const
{
    if (frame.size() % 4 != 0) {
        throw std::runtime_error("Incorrect number of bytes to convert frame.");
    }

    const std::size_t buffer_size = (4 * sizeof(std::uint8_t)) + frame.size() + (4 * sizeof(std::uint8_t));
    std::vector<std::uint8_t> buffer(buffer_size);

    std::fill(buffer.begin(), buffer.begin() + 4, 0x00);
    std::fill(buffer.end() - 4, buffer.end(), 0xFF);

    for (std::size_t i = 0; i < frame.size() / 4; i++) {
        const std::size_t padding = i * 4;
        const std::uint8_t brightness = (std::uint16_t(frame[padding + 3]) * global_brightness_) / 255;
        buffer[4 + padding] = 0xE0 | brightness;
        buffer[4 + padding + 1] = std::uint8_t(pow(frame[padding + 2] / 255.0, kCorrectionGamma) * kCorrBlue);
        buffer[4 + padding + 2] = std::uint8_t(pow(frame[padding + 1] / 255.0, kCorrectionGamma) * kCorrGreen);
        buffer[4 + padding + 3] = std::uint8_t(pow(frame[padding + 0] / 255.0, kCorrectionGamma) * kCorrRed);
    }

    return buffer;
}

void SK9822::spiSendBuffer(const std::vector<std::uint8_t> & buffer) const
{
    if (buffer.size() == 0) {
        throw std::runtime_error("Buffer size must be at least 1 byte.");
    }

    spi_device_->ioctrl(buffer);
}

}  // namespace rover_led
