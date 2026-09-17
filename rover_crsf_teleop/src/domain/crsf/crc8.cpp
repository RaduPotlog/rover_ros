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
//
// Derived from crsf_receiver (MIT, Andrey Tulyakov) - https://github.com/AndreyTulyakov/ros2_crsf_receiver

#include "rover_crsf_teleop/domain/crsf/crc8.hpp"

namespace rover_crsf_teleop::crsf
{

Crc8::Crc8(const std::uint8_t polynomial)
{
    for (std::size_t index = 0; index < lut_.size(); ++index) {
        auto crc = static_cast<std::uint8_t>(index);

        for (int shift = 0; shift < 8; ++shift) {
            const bool high_bit_set = (crc & 0x80U) != 0U;
            crc = static_cast<std::uint8_t>(crc << 1U);

            if (high_bit_set) {
                crc = static_cast<std::uint8_t>(crc ^ polynomial);
            }
        }

        lut_[index] = crc;
    }
}

std::uint8_t Crc8::calc(const std::uint8_t * const data, const std::size_t length) const
{
    std::uint8_t crc = 0;

    for (std::size_t i = 0; i < length; ++i) {
        crc = lut_[static_cast<std::uint8_t>(crc ^ data[i])];
    }

    return crc;
}

}  // namespace rover_crsf_teleop::crsf
