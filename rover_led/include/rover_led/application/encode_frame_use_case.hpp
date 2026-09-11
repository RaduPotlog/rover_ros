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

#ifndef ROVER_LED_APPLICATION_ENCODE_FRAME_USE_CASE_HPP_
#define ROVER_LED_APPLICATION_ENCODE_FRAME_USE_CASE_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "rover_led/domain/sk9822_frame_encoder.hpp"

namespace rover_led
{

// A rendered panel frame as received from the controller. Times are
// nanoseconds on the same clock as the `now_ns` passed to execute().
struct RgbaFrame
{
    std::int64_t stamp_ns;
    std::string encoding;
    std::uint32_t height;
    std::uint32_t width;
    std::vector<std::uint8_t> data;
};

struct EncodeFrameResult
{
    bool accepted = false;
    // Why the frame was rejected (empty when accepted).
    std::string error;
    // Bytes for the UDP LED bridge (empty when rejected).
    std::vector<std::uint8_t> payload;
};

// Validates an incoming panel frame and encodes it for one LED channel.
// Frames that are stale, older than the previous frame, or not a 1 x num_led
// RGBA8 image are rejected.
class EncodeFrameUseCase
{

public:

    static constexpr const char * kRgba8Encoding = "rgba8";

    EncodeFrameUseCase(
        std::shared_ptr<const SK9822FrameEncoder> encoder,
        const std::size_t num_led,
        const std::int64_t frame_timeout_ns,
        const std::int64_t initial_stamp_ns);

    EncodeFrameResult execute(const RgbaFrame & frame, const std::int64_t now_ns);

    // Payload that switches every LED of the channel off.
    std::vector<std::uint8_t> encodeBlank() const;

    std::size_t getNumberOfLeds() const
    {
        return num_led_;
    }

private:

    std::string validate(const RgbaFrame & frame, const std::int64_t now_ns) const;

    std::shared_ptr<const SK9822FrameEncoder> encoder_;
    const std::size_t num_led_;
    const std::int64_t frame_timeout_ns_;
    std::int64_t last_stamp_ns_;
};

}  // namespace rover_led

#endif  // ROVER_LED_APPLICATION_ENCODE_FRAME_USE_CASE_HPP_
