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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRC8_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRC8_HPP_

#include <array>
#include <cstddef>
#include <cstdint>

namespace rover_crsf_teleop::crsf
{

// Table-driven CRC8 for an arbitrary polynomial, initial value 0, no reflection, no final xor.
// CRSF uses CRC8/DVB-S2 (polynomial 0xD5, see crsf_protocol.hpp).
//
// The lookup table is built at construction, so a Crc8 is cheap to use but not free to create -
// hold one for the lifetime of the parser rather than constructing it per frame.
class Crc8
{

public:

    explicit Crc8(std::uint8_t polynomial);

    std::uint8_t calc(const std::uint8_t * data, std::size_t length) const;

private:

    std::array<std::uint8_t, 256> lut_{};
};

}  // namespace rover_crsf_teleop::crsf

#endif  // ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRC8_HPP_
