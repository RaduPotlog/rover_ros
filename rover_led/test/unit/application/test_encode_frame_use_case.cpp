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
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"

#include "rover_led/application/encode_frame_use_case.hpp"

using rover_led::EncodeFrameUseCase;
using rover_led::RgbaFrame;
using rover_led::SK9822FrameEncoder;
using Bytes = std::vector<std::uint8_t>;

namespace
{

constexpr std::int64_t kSecond = 1'000'000'000;
constexpr std::int64_t kStart = 100 * kSecond;
constexpr std::int64_t kTimeout = kSecond / 10;

class EncodeFrameUseCaseTest : public ::testing::Test
{

protected:

    EncodeFrameUseCaseTest()
    : encoder_(std::make_shared<SK9822FrameEncoder>())
    , use_case_(encoder_, 2, kTimeout, kStart)
    {

    }

    static RgbaFrame frame(const std::int64_t stamp_ns)
    {
        return RgbaFrame{stamp_ns, "rgba8", 1, 2, {255, 0, 0, 255, 0, 0, 255, 128}};
    }

    std::shared_ptr<SK9822FrameEncoder> encoder_;
    EncodeFrameUseCase use_case_;
};

}  // namespace

TEST_F(EncodeFrameUseCaseTest, EncodesAValidFrameForTheUdpBridge)
{
    const auto input = frame(kStart + 10);
    const auto result = use_case_.execute(input, kStart + 20);

    EXPECT_TRUE(result.accepted);
    EXPECT_TRUE(result.error.empty());
    EXPECT_EQ(result.payload, encoder_->encodeForUdpBridge(input.data));
}

TEST_F(EncodeFrameUseCaseTest, AcceptsAPanelFoldedIntoRows)
{
    auto folded = frame(kStart + 10);
    folded.height = 2;
    folded.width = 1;
    const auto result = use_case_.execute(folded, kStart + 20);

    EXPECT_TRUE(result.accepted);
    EXPECT_EQ(result.payload, encoder_->encodeForUdpBridge(folded.data));
}

TEST_F(EncodeFrameUseCaseTest, UsesTheCurrentBrightness)
{
    encoder_->setGlobalBrightness(0.5f);
    const auto input = frame(kStart);

    EXPECT_EQ(use_case_.execute(input, kStart).payload[7], 0xE0 | 16);
}

TEST_F(EncodeFrameUseCaseTest, RejectsStaleFrames)
{
    const auto result = use_case_.execute(frame(kStart), kStart + kTimeout + 1);

    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error, "Timeout exceeded, ignoring frame");
    EXPECT_TRUE(result.payload.empty());

    EXPECT_TRUE(use_case_.execute(frame(kStart + 1), kStart + kTimeout + 1).accepted);
}

TEST_F(EncodeFrameUseCaseTest, RejectsFramesOlderThanThePreviousOne)
{
    EXPECT_FALSE(use_case_.execute(frame(kStart - 1), kStart).accepted);

    ASSERT_TRUE(use_case_.execute(frame(kStart + 50), kStart + 50).accepted);

    const auto result = use_case_.execute(frame(kStart + 40), kStart + 60);
    EXPECT_FALSE(result.accepted);
    EXPECT_EQ(result.error, "Dropping message from past");
}

TEST_F(EncodeFrameUseCaseTest, RejectedFramesStillAdvanceTheOrderingReference)
{
    // Wrong encoding, but its stamp becomes the new reference.
    auto bad = frame(kStart + 50);
    bad.encoding = "bgr8";
    ASSERT_FALSE(use_case_.execute(bad, kStart + 50).accepted);

    EXPECT_FALSE(use_case_.execute(frame(kStart + 40), kStart + 60).accepted);
}

TEST_F(EncodeFrameUseCaseTest, RejectsMalformedImages)
{
    auto encoding = frame(kStart);
    encoding.encoding = "rgb8";
    EXPECT_EQ(use_case_.execute(encoding, kStart).error, "Incorrect image encoding ('rgb8')");

    auto height = frame(kStart);
    height.height = 2;
    EXPECT_EQ(use_case_.execute(height, kStart).error, "Incorrect image size 2x2");

    auto width = frame(kStart);
    width.width = 3;
    EXPECT_EQ(use_case_.execute(width, kStart).error, "Incorrect image size 1x3");

    auto data = frame(kStart);
    data.data.pop_back();
    EXPECT_EQ(use_case_.execute(data, kStart).error, "Incorrect image data size 7");
}

TEST_F(EncodeFrameUseCaseTest, BlankPayloadSwitchesEveryLedOff)
{
    EXPECT_EQ(
        use_case_.encodeBlank(),
        (Bytes{0, 0, 0, 0, 0, 0, 0, 0xE0, 0, 0, 0, 0xE0, 0xFF, 0xFF, 0xFF, 0xFF}));
}
