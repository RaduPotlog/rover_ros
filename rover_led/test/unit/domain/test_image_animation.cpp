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

#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"

#include "../test_helpers.hpp"

namespace gil = boost::gil;

using rover_led::test::pixel;
using rover_led::test::Rgba;

namespace
{

class ImageAnimationTest : public ::testing::Test
{

protected:

    void SetUp() override
    {
        dir_ = std::filesystem::temp_directory_path() /
            ("rover_led_test_" + std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
             ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::create_directories(dir_);
    }

    void TearDown() override
    {
        std::filesystem::remove_all(dir_);
    }

    // Writes a width x height PNG whose pixels come from `color_at(x, y)`.
    std::string writePng(
        const std::string & name, const std::size_t width, const std::size_t height,
        const std::function<Rgba(std::size_t, std::size_t)> & color_at)
    {
        gil::rgba8_image_t image(width, height);
        auto view = gil::view(image);

        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto c = color_at(x, y);
                view(x, y) = gil::rgba8_pixel_t(c[0], c[1], c[2], c[3]);
            }
        }

        const auto path = (dir_ / name).string();
        gil::write_view(path, gil::const_view(image), gil::png_tag());

        return path;
    }

    std::filesystem::path dir_;
};

YAML::Node description(const std::string & image, const float duration)
{
    YAML::Node node;
    node["image"] = image;
    node["duration"] = duration;

    return node;
}

// LEDs whose alpha is non-zero.
std::vector<std::size_t> litLeds(const std::vector<std::uint8_t> & frame)
{
    std::vector<std::size_t> lit;

    for (std::size_t i = 0; i < frame.size() / 4; i++) {
        if (frame[i * 4 + 3] != 0) {
            lit.push_back(i);
        }
    }

    return lit;
}

}  // namespace

TEST_F(ImageAnimationTest, PlaysImageRowsTopToBottom)
{
    // 4 LEDs x 5 frames (0.5 s at 10 Hz): no resampling needed.
    const auto image = writePng("rows.png", 4, 5, [](std::size_t x, std::size_t y) {
        return Rgba{std::uint8_t(10 * y), std::uint8_t(x), 7, 255};
    });

    rover_led::ImageAnimation animation;
    animation.initialize(description(image, 0.5f), 4, 10.0f);

    for (std::size_t row = 0; row < 5; row++) {
        animation.update();
        const auto frame = animation.getFrame();

        for (std::size_t led = 0; led < 4; led++) {
            EXPECT_EQ(pixel(frame, led), (Rgba{std::uint8_t(10 * row), std::uint8_t(led), 7, 255}))
                << "row " << row << " led " << led;
        }
    }

    EXPECT_TRUE(animation.isFinished());
}

TEST_F(ImageAnimationTest, ColorOptionRecoloursByNormalizedBrightness)
{
    const auto image = writePng("grey.png", 2, 1, [](std::size_t x, std::size_t) {
        return x == 0 ? Rgba{100, 100, 100, 255} : Rgba{50, 50, 50, 128};
    });

    auto desc = description(image, 0.1f);
    desc["color"] = 0x00FF80;

    rover_led::ImageAnimation animation;
    animation.initialize(desc, 2, 10.0f);
    animation.update();
    const auto frame = animation.getFrame();

    // Brightest pixel maps to the full colour; the other to half of it.
    EXPECT_EQ(pixel(frame, 0), (Rgba{0, 255, 128, 255}));
    EXPECT_EQ(pixel(frame, 1), (Rgba{0, 127, 63, 128}));
}

TEST_F(ImageAnimationTest, RequiresAnExistingAbsolutePath)
{
    rover_led::ImageAnimation animation;

    EXPECT_THROW(
        animation.initialize(description("$(find rover_led)/animations/x.png", 1.0f), 4, 10.0f),
        std::runtime_error);
    EXPECT_THROW(animation.initialize(description("relative.png", 1.0f), 4, 10.0f), std::runtime_error);
    EXPECT_THROW(
        animation.initialize(description((dir_ / "missing.png").string(), 1.0f), 4, 10.0f),
        std::runtime_error);
}

class MovingImageAnimationTest : public ImageAnimationTest
{

protected:

    // A 3 px wide, 2 frame tall opaque object; column x has red = 100 + x.
    YAML::Node objectDescription()
    {
        const auto image = writePng("object.png", 3, 2, [](std::size_t x, std::size_t) {
            return Rgba{std::uint8_t(100 + x), 0, 0, 255};
        });

        auto desc = description(image, 1.0f);
        desc["object_width"] = 3;

        return desc;
    }
};

TEST_F(MovingImageAnimationTest, ParamPlacesTheObjectAlongTheSegment)
{
    rover_led::MovingImageAnimation animation;
    animation.initialize(objectDescription(), 10, 10.0f);

    animation.setParam("0.0");
    animation.update();
    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{0, 1, 2}));
    EXPECT_EQ(pixel(animation.getFrame(), 0)[0], 100);

    animation.reset();
    animation.setParam("1.0");
    animation.update();
    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{7, 8, 9}));
    EXPECT_EQ(pixel(animation.getFrame(), 9)[0], 102);
}

TEST_F(MovingImageAnimationTest, ObjectIsOnlyShownForTheSplashDuration)
{
    rover_led::MovingImageAnimation animation;
    animation.initialize(objectDescription(), 10, 10.0f);
    animation.setParam("0.5");

    // Without splash_duration the image height (2 frames) is used.
    animation.update();
    EXPECT_FALSE(litLeds(animation.getFrame()).empty());
    animation.update();
    EXPECT_FALSE(litLeds(animation.getFrame()).empty());
    animation.update();
    EXPECT_TRUE(litLeds(animation.getFrame()).empty());
}

TEST_F(MovingImageAnimationTest, StartOffsetDelaysTheObject)
{
    auto desc = objectDescription();
    desc["start_offset"] = 0.2;

    rover_led::MovingImageAnimation animation;
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("0.0");

    animation.update();
    EXPECT_TRUE(litLeds(animation.getFrame()).empty());
    animation.update();
    EXPECT_TRUE(litLeds(animation.getFrame()).empty());
    animation.update();
    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{0, 1, 2}));
}

TEST_F(MovingImageAnimationTest, MirroringFlipsPositionAndImage)
{
    auto desc = objectDescription();
    desc["position_mirrored"] = true;
    desc["image_mirrored"] = true;

    rover_led::MovingImageAnimation animation;
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("0.0");
    animation.update();

    // Position 0.0 mirrored is the far end; the image is drawn right-to-left.
    const auto frame = animation.getFrame();
    EXPECT_EQ(litLeds(frame), (std::vector<std::size_t>{7, 8, 9}));
    EXPECT_EQ(pixel(frame, 7)[0], 102);
    EXPECT_EQ(pixel(frame, 9)[0], 100);
}

TEST_F(MovingImageAnimationTest, EmptyParamUsesTheDefaultPosition)
{
    auto desc = objectDescription();
    desc["default_image_position"] = 1.0;

    rover_led::MovingImageAnimation animation;
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("");
    animation.update();

    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{7, 8, 9}));
}

TEST_F(MovingImageAnimationTest, RejectsMissingOrInvalidParam)
{
    rover_led::MovingImageAnimation animation;
    animation.initialize(objectDescription(), 10, 10.0f);

    EXPECT_THROW(animation.setParam(""), std::runtime_error);
    EXPECT_THROW(animation.setParam("left"), std::runtime_error);
}

TEST_F(MovingImageAnimationTest, OutOfRangeParamIsClamped)
{
    rover_led::MovingImageAnimation animation;
    animation.initialize(objectDescription(), 10, 10.0f);
    animation.setParam("7.5");
    animation.update();

    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{7, 8, 9}));
}

TEST_F(MovingImageAnimationTest, RequiresAnExistingAbsolutePath)
{
    rover_led::MovingImageAnimation animation;

    EXPECT_THROW(
        animation.initialize(description("$(find rover_led)/animations/x.png", 1.0f), 10, 10.0f),
        std::runtime_error);
    EXPECT_THROW(animation.initialize(description("relative.png", 1.0f), 10, 10.0f), std::runtime_error);
    EXPECT_THROW(
        animation.initialize(description((dir_ / "missing.png").string(), 1.0f), 10, 10.0f),
        std::runtime_error);
}

TEST_F(MovingImageAnimationTest, ColorOptionRecoloursTheObject)
{
    const auto image = writePng("grey_object.png", 3, 2, [](std::size_t, std::size_t) {
        return Rgba{100, 100, 100, 255};
    });

    auto desc = description(image, 1.0f);
    desc["object_width"] = 3;
    desc["color"] = 0x00FF80;

    rover_led::MovingImageAnimation animation;
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("0.0");
    animation.update();

    // A uniform image is its own brightest pixel, so every LED gets the full colour.
    const auto frame = animation.getFrame();
    EXPECT_EQ(litLeds(frame), (std::vector<std::size_t>{0, 1, 2}));

    for (std::size_t led = 0; led < 3; led++) {
        EXPECT_EQ(pixel(frame, led), (Rgba{0, 255, 128, 255})) << "led " << led;
    }
}

TEST_F(MovingImageAnimationTest, SplashDurationResamplesTheImageHeight)
{
    // 0.3 s at 10 Hz stretches the 2 row image to 3 frames.
    auto desc = objectDescription();
    desc["splash_duration"] = 0.3;

    rover_led::MovingImageAnimation animation;
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("0.5");

    for (int update = 1; update <= 3; update++) {
        animation.update();
        EXPECT_FALSE(litLeds(animation.getFrame()).empty()) << "update " << update;
    }

    animation.update();
    EXPECT_TRUE(litLeds(animation.getFrame()).empty());
}
