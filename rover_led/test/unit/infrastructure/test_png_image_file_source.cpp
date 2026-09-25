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
#include <fstream>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"
#include "gmock/gmock.h"
#include "gtest/gtest.h"
#include "yaml-cpp/yaml.h"

#include "rover_led/domain/animation/image_animation.hpp"
#include "rover_led/domain/animation/moving_image_animation.hpp"
#include "rover_led/infrastructure/png_image_file_source.hpp"

#include "../test_helpers.hpp"

namespace gil = boost::gil;

using rover_led::test::pixel;
using rover_led::test::Rgba;
using ::testing::HasSubstr;

namespace
{

class PngImageFileSourceTest : public ::testing::Test
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

    std::string writeFile(const std::string & name, const std::string & text)
    {
        const auto path = (dir_ / name).string();
        std::ofstream(path) << text;

        return path;
    }

    std::filesystem::path dir_;
    rover_led::PngImageFileSource source_;
};

YAML::Node description(const std::string & image, const float duration)
{
    YAML::Node node;
    node["image"] = image;
    node["duration"] = duration;

    return node;
}

std::string readError(const rover_led::PngImageFileSource & source, const std::string & image)
{
    try {
        source.read(image);
    } catch (const std::runtime_error & e) {
        return e.what();
    }

    ADD_FAILURE() << "expected a std::runtime_error for '" << image << "'";
    return "";
}

}  // namespace

TEST_F(PngImageFileSourceTest, DecodesEveryPixelRowByRowFromTheTopLeft)
{
    const auto path = writePng("pixels.png", 3, 2, [](std::size_t x, std::size_t y) {
        return Rgba{std::uint8_t(10 * x), std::uint8_t(20 * y), 7, std::uint8_t(255 - 60 * x)};
    });

    const auto image = source_.read(path);

    EXPECT_EQ(image.width, 3u);
    EXPECT_EQ(image.height, 2u);
    EXPECT_EQ(image.pixels, (std::vector<std::uint8_t>{
        0, 0, 7, 255,   10, 0, 7, 195,   20, 0, 7, 135,
        0, 20, 7, 255,  10, 20, 7, 195,  20, 20, 7, 135}));
}

TEST_F(PngImageFileSourceTest, ConvertsAnRgbPngToOpaqueRgba)
{
    gil::rgb8_image_t rgb(2, 1);
    gil::view(rgb)(0, 0) = gil::rgb8_pixel_t(1, 2, 3);
    gil::view(rgb)(1, 0) = gil::rgb8_pixel_t(4, 5, 6);

    const auto path = (dir_ / "rgb.png").string();
    gil::write_view(path, gil::const_view(rgb), gil::png_tag());

    const auto image = source_.read(path);

    EXPECT_EQ(image.width, 2u);
    EXPECT_EQ(image.height, 1u);
    EXPECT_EQ(image.pixels, (std::vector<std::uint8_t>{1, 2, 3, 255, 4, 5, 6, 255}));
}

TEST_F(PngImageFileSourceTest, DecodesAShippedImage)
{
    const auto image = source_.read(std::string(ROVER_LED_SOURCE_DIR) + "/animations/rover_a1/flood_light.png");

    EXPECT_EQ(image.width, 46u);
    EXPECT_EQ(image.height, 350u);
    EXPECT_EQ(image.pixels, std::vector<std::uint8_t>(46 * 350 * 4, 255));
}

TEST_F(PngImageFileSourceTest, RejectsARelativePath)
{
    EXPECT_THAT(readError(source_, "relative.png"), HasSubstr("expected an absolute path"));
}

TEST_F(PngImageFileSourceTest, RejectsAnUnresolvedFindSubstitution)
{
    EXPECT_THAT(
        readError(source_, "$(find rover_led)/animations/x.png"),
        HasSubstr("unresolved $(find <pkg>) substitution?"));
}

TEST_F(PngImageFileSourceTest, RejectsAMissingFile)
{
    const auto path = (dir_ / "missing.png").string();

    EXPECT_EQ(readError(source_, path), "File doesn't exists: " + path);
}

TEST_F(PngImageFileSourceTest, RejectsAFileThatIsNotAPng)
{
    const auto path = writeFile("not_a.png", "not a png");

    EXPECT_THROW(source_.read(path), std::runtime_error);
}

TEST_F(PngImageFileSourceTest, ImageAnimationRequiresAnExistingAbsolutePath)
{
    rover_led::ImageAnimation animation(std::make_shared<rover_led::PngImageFileSource>());

    EXPECT_THROW(
        animation.initialize(description("$(find rover_led)/animations/x.png", 1.0f), 4, 10.0f),
        std::runtime_error);
    EXPECT_THROW(animation.initialize(description("relative.png", 1.0f), 4, 10.0f), std::runtime_error);
    EXPECT_THROW(
        animation.initialize(description((dir_ / "missing.png").string(), 1.0f), 4, 10.0f),
        std::runtime_error);
}

TEST_F(PngImageFileSourceTest, MovingImageAnimationRequiresAnExistingAbsolutePath)
{
    rover_led::MovingImageAnimation animation(std::make_shared<rover_led::PngImageFileSource>());

    EXPECT_THROW(
        animation.initialize(description("$(find rover_led)/animations/x.png", 1.0f), 10, 10.0f),
        std::runtime_error);
    EXPECT_THROW(animation.initialize(description("relative.png", 1.0f), 10, 10.0f), std::runtime_error);
    EXPECT_THROW(
        animation.initialize(description((dir_ / "missing.png").string(), 1.0f), 10, 10.0f),
        std::runtime_error);
}

TEST_F(PngImageFileSourceTest, ImageAnimationPlaysAPngFileRowByRow)
{
    // 4 LEDs x 5 frames (0.5 s at 10 Hz): no resampling needed.
    const auto image = writePng("rows.png", 4, 5, [](std::size_t x, std::size_t y) {
        return Rgba{std::uint8_t(10 * y), std::uint8_t(x), 7, 255};
    });

    rover_led::ImageAnimation animation(std::make_shared<rover_led::PngImageFileSource>());
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
