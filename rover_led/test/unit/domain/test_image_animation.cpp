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
#include <functional>
#include <map>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"
#include "rover_led/domain/ports/image_source.hpp"

#include "../test_helpers.hpp"

using rover_led::test::pixel;
using rover_led::test::Rgba;

namespace
{

// Serves images from memory and records the names it was asked for.
class FakeImageSource : public rover_led::IImageSource
{

public:

    rover_led::RgbaImage read(const std::string & image) const override
    {
        requested.push_back(image);

        const auto it = images.find(image);

        if (it == images.end()) {
            throw std::runtime_error("No image '" + image + "'");
        }

        return it->second;
    }

    std::map<std::string, rover_led::RgbaImage> images;
    mutable std::vector<std::string> requested;
};

class ImageAnimationTest : public ::testing::Test
{

protected:

    // Adds a width x height image whose pixels come from `color_at(x, y)`; returns its name.
    std::string addImage(
        const std::string & name, const std::size_t width, const std::size_t height,
        const std::function<Rgba(std::size_t, std::size_t)> & color_at)
    {
        rover_led::RgbaImage image;
        image.width = width;
        image.height = height;

        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto c = color_at(x, y);
                image.pixels.insert(image.pixels.end(), c.begin(), c.end());
            }
        }

        images_->images[name] = image;

        return name;
    }

    std::shared_ptr<FakeImageSource> images_ = std::make_shared<FakeImageSource>();
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
    const auto image = addImage("rows.png", 4, 5, [](std::size_t x, std::size_t y) {
        return Rgba{std::uint8_t(10 * y), std::uint8_t(x), 7, 255};
    });

    rover_led::ImageAnimation animation(images_);
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
    const auto image = addImage("grey.png", 2, 1, [](std::size_t x, std::size_t) {
        return x == 0 ? Rgba{100, 100, 100, 255} : Rgba{50, 50, 50, 128};
    });

    auto desc = description(image, 0.1f);
    desc["color"] = 0x00FF80;

    rover_led::ImageAnimation animation(images_);
    animation.initialize(desc, 2, 10.0f);
    animation.update();
    const auto frame = animation.getFrame();

    // Brightest pixel maps to the full colour; the other to half of it.
    EXPECT_EQ(pixel(frame, 0), (Rgba{0, 255, 128, 255}));
    EXPECT_EQ(pixel(frame, 1), (Rgba{0, 127, 63, 128}));
}

TEST_F(ImageAnimationTest, ReadsTheImageTheDescriptionNames)
{
    const auto image = addImage("rows.png", 4, 5, [](std::size_t, std::size_t) {
        return Rgba{1, 2, 3, 255};
    });

    rover_led::ImageAnimation animation(images_);
    animation.initialize(description(image, 0.5f), 4, 10.0f);

    EXPECT_EQ(images_->requested, (std::vector<std::string>{"rows.png"}));
}

TEST_F(ImageAnimationTest, FailsWhenTheImageSourceFails)
{
    rover_led::ImageAnimation animation(images_);

    EXPECT_THROW(animation.initialize(description("unknown.png", 1.0f), 4, 10.0f), std::runtime_error);
}

TEST_F(ImageAnimationTest, RequiresAnImageKey)
{
    YAML::Node desc;
    desc["duration"] = 1.0f;

    rover_led::ImageAnimation animation(images_);

    EXPECT_THROW(animation.initialize(desc, 4, 10.0f), std::runtime_error);
    EXPECT_TRUE(images_->requested.empty());
}

TEST_F(ImageAnimationTest, RejectsAnImageWhosePixelsDoNotMatchItsSize)
{
    images_->images["short.png"] = rover_led::RgbaImage{2, 2, std::vector<std::uint8_t>(12, 255)};
    images_->images["empty.png"] = rover_led::RgbaImage{0, 0, {}};

    rover_led::ImageAnimation animation(images_);

    EXPECT_THROW(animation.initialize(description("short.png", 1.0f), 4, 10.0f), std::runtime_error);
    EXPECT_THROW(animation.initialize(description("empty.png", 1.0f), 4, 10.0f), std::runtime_error);
}

class MovingImageAnimationTest : public ImageAnimationTest
{

protected:

    // A 3 px wide, 2 frame tall opaque object; column x has red = 100 + x.
    YAML::Node objectDescription()
    {
        const auto image = addImage("object.png", 3, 2, [](std::size_t x, std::size_t) {
            return Rgba{std::uint8_t(100 + x), 0, 0, 255};
        });

        auto desc = description(image, 1.0f);
        desc["object_width"] = 3;

        return desc;
    }
};

TEST_F(MovingImageAnimationTest, ParamPlacesTheObjectAlongTheSegment)
{
    rover_led::MovingImageAnimation animation(images_);
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
    rover_led::MovingImageAnimation animation(images_);
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

    rover_led::MovingImageAnimation animation(images_);
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

    rover_led::MovingImageAnimation animation(images_);
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

    rover_led::MovingImageAnimation animation(images_);
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("");
    animation.update();

    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{7, 8, 9}));
}

TEST_F(MovingImageAnimationTest, RejectsMissingOrInvalidParam)
{
    rover_led::MovingImageAnimation animation(images_);
    animation.initialize(objectDescription(), 10, 10.0f);

    EXPECT_THROW(animation.setParam(""), std::runtime_error);
    EXPECT_THROW(animation.setParam("left"), std::runtime_error);
}

TEST_F(MovingImageAnimationTest, OutOfRangeParamIsClamped)
{
    rover_led::MovingImageAnimation animation(images_);
    animation.initialize(objectDescription(), 10, 10.0f);
    animation.setParam("7.5");
    animation.update();

    EXPECT_EQ(litLeds(animation.getFrame()), (std::vector<std::size_t>{7, 8, 9}));
}

TEST_F(MovingImageAnimationTest, FailsWhenTheImageSourceFails)
{
    rover_led::MovingImageAnimation animation(images_);

    EXPECT_THROW(animation.initialize(description("unknown.png", 1.0f), 10, 10.0f), std::runtime_error);
}

TEST_F(MovingImageAnimationTest, ColorOptionRecoloursTheObject)
{
    const auto image = addImage("grey_object.png", 3, 2, [](std::size_t, std::size_t) {
        return Rgba{100, 100, 100, 255};
    });

    auto desc = description(image, 1.0f);
    desc["object_width"] = 3;
    desc["color"] = 0x00FF80;

    rover_led::MovingImageAnimation animation(images_);
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

    rover_led::MovingImageAnimation animation(images_);
    animation.initialize(desc, 10, 10.0f);
    animation.setParam("0.5");

    for (int update = 1; update <= 3; update++) {
        animation.update();
        EXPECT_FALSE(litLeds(animation.getFrame()).empty()) << "update " << update;
    }

    animation.update();
    EXPECT_TRUE(litLeds(animation.getFrame()).empty());
}
