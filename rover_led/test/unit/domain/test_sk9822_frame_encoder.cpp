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

#include <cstdint>
#include <stdexcept>
#include <tuple>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/domain/sk9822_frame_encoder.hpp"

using rover_led::SK9822FrameEncoder;
using Bytes = std::vector<std::uint8_t>;

namespace
{

Bytes led(const SK9822FrameEncoder & encoder, const Bytes & rgba)
{
    const auto buffer = encoder.encode(rgba);

    return Bytes(buffer.begin() + 4, buffer.begin() + 8);
}

}  // namespace

TEST(SK9822FrameEncoder, WrapsLedWordsInStartAndEndFrames)
{
    SK9822FrameEncoder encoder;
    const auto buffer = encoder.encode(Bytes(3 * 4, 0));

    ASSERT_EQ(buffer.size(), 4u + 3 * 4 + 4);
    EXPECT_EQ(Bytes(buffer.begin(), buffer.begin() + 4), Bytes(4, 0x00));
    EXPECT_EQ(Bytes(buffer.end() - 4, buffer.end()), Bytes(4, 0xFF));
}

// Reference values computed independently:
// [0xE0 | (a * brightness / 255), (b/255)^2.2 * 240, (g/255)^2.2 * 255, (r/255)^2.2 * 245]
class SK9822KnownVectors
    : public ::testing::TestWithParam<std::tuple<Bytes, std::uint8_t, Bytes>>
{
};

TEST_P(SK9822KnownVectors, EncodesLed)
{
    const auto & [rgba, brightness, expected] = GetParam();

    SK9822FrameEncoder encoder;
    encoder.setGlobalBrightness(brightness);

    EXPECT_EQ(led(encoder, rgba), expected);
}

INSTANTIATE_TEST_SUITE_P(
    Vectors, SK9822KnownVectors,
    ::testing::Values(
        std::make_tuple(Bytes{255, 255, 255, 255}, std::uint8_t(31), Bytes{0xFF, 240, 255, 245}),
        std::make_tuple(Bytes{0, 0, 0, 0}, std::uint8_t(31), Bytes{0xE0, 0, 0, 0}),
        std::make_tuple(Bytes{128, 64, 32, 128}, std::uint8_t(31), Bytes{0xEF, 2, 12, 53}),
        std::make_tuple(Bytes{10, 200, 90, 255}, std::uint8_t(16), Bytes{0xF0, 24, 149, 0}),
        std::make_tuple(Bytes{255, 0, 0, 1}, std::uint8_t(31), Bytes{0xE0, 0, 0, 245})));

TEST(SK9822FrameEncoder, DefaultsToFullBrightness)
{
    EXPECT_EQ(SK9822FrameEncoder().getGlobalBrightness(), 31);
}

TEST(SK9822FrameEncoder, FloatBrightnessRoundsUpToFiveBits)
{
    SK9822FrameEncoder encoder;

    for (const auto & [brightness, expected] : std::vector<std::pair<float, int>>{
            {0.0f, 0}, {0.1f, 4}, {0.33f, 11}, {0.5f, 16}, {0.9f, 28}, {1.0f, 31}}) {
        encoder.setGlobalBrightness(brightness);
        EXPECT_EQ(encoder.getGlobalBrightness(), expected) << brightness;
    }
}

TEST(SK9822FrameEncoder, RejectsOutOfRangeBrightness)
{
    SK9822FrameEncoder encoder;

    EXPECT_THROW(encoder.setGlobalBrightness(-0.1f), std::out_of_range);
    EXPECT_THROW(encoder.setGlobalBrightness(1.1f), std::out_of_range);
    EXPECT_THROW(encoder.setGlobalBrightness(std::uint8_t(32)), std::out_of_range);
    EXPECT_EQ(encoder.getGlobalBrightness(), 31);
}

TEST(SK9822FrameEncoder, RejectsIncompletePixels)
{
    EXPECT_THROW(SK9822FrameEncoder().encode(Bytes(5, 0)), std::runtime_error);
}

TEST(SK9822FrameEncoder, UdpBridgeLayoutRotatesEachWord)
{
    SK9822FrameEncoder encoder;
    const Bytes rgba{255, 255, 255, 255, 0, 0, 0, 0};

    const auto spi = encoder.encode(rgba);
    const auto udp = encoder.encodeForUdpBridge(rgba);

    EXPECT_EQ(
        udp, (Bytes{0x00, 0x00, 0x00, 0x00, 240, 255, 245, 0xFF, 0, 0, 0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF}));
    ASSERT_EQ(udp.size(), spi.size());
}
