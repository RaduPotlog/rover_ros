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

#include "rover_led/application/encode_frame_use_case.hpp"

#include <string>
#include <utility>
#include <vector>

namespace rover_led
{

EncodeFrameUseCase::EncodeFrameUseCase(
    std::shared_ptr<const SK9822FrameEncoder> encoder,
    const std::size_t num_led,
    const std::int64_t frame_timeout_ns,
    const std::int64_t initial_stamp_ns)
: encoder_(std::move(encoder))
, num_led_(num_led)
, frame_timeout_ns_(frame_timeout_ns)
, last_stamp_ns_(initial_stamp_ns)
{

}

EncodeFrameResult EncodeFrameUseCase::execute(const RgbaFrame & frame, const std::int64_t now_ns)
{
    EncodeFrameResult result;
    result.error = validate(frame, now_ns);

    // Ordering is tracked against every received frame, accepted or not.
    last_stamp_ns_ = frame.stamp_ns;

    if (!result.error.empty()) {
        return result;
    }

    result.payload = encoder_->encodeForUdpBridge(frame.data);
    result.accepted = true;

    return result;
}

std::vector<std::uint8_t> EncodeFrameUseCase::encodeBlank() const
{
    return encoder_->encodeForUdpBridge(std::vector<std::uint8_t>(num_led_ * 4, 0));
}

std::string EncodeFrameUseCase::validate(const RgbaFrame & frame, const std::int64_t now_ns) const
{
    if (now_ns - frame.stamp_ns > frame_timeout_ns_) {
        return "Timeout exceeded, ignoring frame";
    }

    if (frame.stamp_ns < last_stamp_ns_) {
        return "Dropping message from past";
    }

    if (frame.encoding != kRgba8Encoding) {
        return "Incorrect image encoding ('" + frame.encoding + "')";
    }

    if (frame.height != 1) {
        return "Incorrect image height " + std::to_string(frame.height);
    }

    if (frame.width != num_led_) {
        return "Incorrect image width " + std::to_string(frame.width);
    }

    if (frame.data.size() != num_led_ * 4) {
        return "Incorrect image data size " + std::to_string(frame.data.size());
    }

    return "";
}

}  // namespace rover_led
