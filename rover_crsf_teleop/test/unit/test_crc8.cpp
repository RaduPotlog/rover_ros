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

// Unit tests for the CRSF CRC8 (CRC8/DVB-S2, polynomial 0xD5).

#include <gtest/gtest.h>

#include <cstdint>
#include <random>
#include <vector>

#include "rover_crsf_teleop/domain/crsf/crc8.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_protocol.hpp"

namespace rover_crsf_teleop::crsf
{
namespace
{

// Bit-by-bit reference. The production code is table-driven; cross-checking the two is what
// catches a corrupted lookup table without hardcoding 256 magic numbers into this file.
std::uint8_t referenceCrc8(const std::vector<std::uint8_t> & data, const std::uint8_t polynomial)
{
    std::uint8_t crc = 0;

    for (const std::uint8_t byte : data) {
        crc = static_cast<std::uint8_t>(crc ^ byte);

        for (int bit = 0; bit < 8; ++bit) {
            const bool high_bit_set = (crc & 0x80U) != 0U;
            crc = static_cast<std::uint8_t>(crc << 1U);

            if (high_bit_set) {
                crc = static_cast<std::uint8_t>(crc ^ polynomial);
            }
        }
    }

    return crc;
}

TEST(Crc8Test, EmptyBufferIsZero)
{
    const Crc8 crc(kCrcPolynomial);
    EXPECT_EQ(crc.calc(nullptr, 0), 0);
}

TEST(Crc8Test, MatchesReferenceForEverySingleByte)
{
    const Crc8 crc(kCrcPolynomial);

    for (int value = 0; value <= 0xFF; ++value) {
        const auto byte = static_cast<std::uint8_t>(value);
        EXPECT_EQ(crc.calc(&byte, 1), referenceCrc8({byte}, kCrcPolynomial)) << "byte " << value;
    }
}

TEST(Crc8Test, MatchesReferenceForRandomBuffers)
{
    const Crc8 crc(kCrcPolynomial);

    std::mt19937 rng(20250917);
    std::uniform_int_distribution<int> byte_dist(0, 255);
    std::uniform_int_distribution<std::size_t> size_dist(1, 64);

    for (int trial = 0; trial < 1000; ++trial) {
        std::vector<std::uint8_t> data(size_dist(rng));

        for (std::uint8_t & byte : data) {
            byte = static_cast<std::uint8_t>(byte_dist(rng));
        }

        EXPECT_EQ(crc.calc(data.data(), data.size()), referenceCrc8(data, kCrcPolynomial))
            << "trial " << trial;
    }
}

// The on-wire CRC of a real RC_CHANNELS_PACKED frame: computed over type + payload, i.e. bytes
// [2 .. length_byte] of the frame. Captured from the receiver this package replaces.
TEST(Crc8Test, MatchesCapturedFrameChecksum)
{
    const Crc8 crc(kCrcPolynomial);

    const std::vector<std::uint8_t> frame{
        0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F,
        0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0, 0x81, 0x0F, 0x7C, 0xAD};

    // length_byte = 0x18 = 24 covers type + 22 payload + crc, so 23 bytes are checksummed.
    EXPECT_EQ(crc.calc(frame.data() + 2, 23), 0xAD);
}

}  // namespace
}  // namespace rover_crsf_teleop::crsf
