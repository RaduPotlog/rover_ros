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

#ifndef ROVER_LED_APA102_HPP_
#define ROVER_LED_APA102_HPP_

#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>

namespace rover_led
{

class SPIDeviceInterface
{

public:
  
    virtual ~SPIDeviceInterface() = default;

    virtual int open(const std::string & device) = 0;

    virtual int ioctrl(const std::vector<std::uint8_t> & buffer) = 0;

    virtual int close(int fd) = 0;

    using SharedPtr = std::shared_ptr<SPIDeviceInterface>;
};

class SPIDevice : public SPIDeviceInterface
{

public:
  
    int open(const std::string & device) override 
    { 
        (void)device;
        return 0;    
    }

    int ioctrl(const std::vector<std::uint8_t>& buffer) override
    {
        (void)buffer;
        return 0;
    }

    int close(int fd) override 
    { 
        (void)fd;
        return 0; 
    }

protected:
 
};

class SK9822Interface
{

public:
  
    virtual ~SK9822Interface() = default;

    virtual void setGlobalBrightness(const std::uint8_t brightness) = 0;
    virtual void setGlobalBrightness(const float brightness) = 0;
    virtual std::vector<std::uint8_t> setPanel(const std::vector<std::uint8_t> & frame) const = 0;

    using SharedPtr = std::shared_ptr<SK9822Interface>;
    
};

class SK9822 : public SK9822Interface
{

public:

    SK9822(
        SPIDeviceInterface::SharedPtr spi_device, 
        const std::string & device_name,
        const std::uint32_t speed = 800000, 
        const bool cs_high = false);
    
    ~SK9822();

    void setGlobalBrightness(const std::uint8_t brightness) override;

    void setGlobalBrightness(const float brightness) override;

    std::vector<std::uint8_t> setPanel(const std::vector<std::uint8_t> & frame) const override;

protected:
  
    std::vector<std::uint8_t> rgbFrameToBGRBuffer(const std::vector<std::uint8_t> & frame) const;

    void spiSendBuffer(const std::vector<std::uint8_t> & buffer) const;

    std::uint16_t global_brightness_;

private:
    
    static constexpr std::uint8_t kBits = 8;

    static constexpr std::uint16_t kCorrRed = 245;
    static constexpr std::uint16_t kCorrGreen = 255;
    static constexpr std::uint16_t kCorrBlue = 240;
    static constexpr float kCorrectionGamma = 2.2;

    SPIDeviceInterface::SharedPtr spi_device_;

    const std::string device_name_;
    const std::uint32_t speed_;
    const int file_descriptor_;

    rclcpp::Logger logger_{rclcpp::get_logger("RoverSystem")};
};

}  // namespace rover_led

#endif  // ROVER_LED_APA102_HPP_