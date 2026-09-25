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

#include "rover_led/infrastructure/png_image_file_source.hpp"

#include <cstddef>
#include <filesystem>
#include <stdexcept>
#include <string>

#include "boost/gil.hpp"
#include "boost/gil/extension/io/png.hpp"

namespace gil = boost::gil;

namespace rover_led
{

RgbaImage PngImageFileSource::read(const std::string & image) const
{
    const std::filesystem::path path(image);

    if (!path.is_absolute()) {
        throw std::runtime_error(
            "Invalid image path '" + image + "': expected an absolute path "
            "(unresolved $(find <pkg>) substitution?)");
    }

    if (!std::filesystem::exists(path)) {
        throw std::runtime_error("File doesn't exists: " + std::string(path));
    }

    gil::rgba8_image_t decoded;
    gil::read_and_convert_image(std::string(path), decoded, gil::png_tag());

    constexpr std::size_t kBytesPerPixel = 4;

    RgbaImage result;
    result.width = decoded.width();
    result.height = decoded.height();
    result.pixels.resize(result.width * result.height * kBytesPerPixel);

    const auto decoded_view = gil::const_view(decoded);

    for (std::size_t y = 0; y < result.height; y++) {
        for (std::size_t x = 0; x < result.width; x++) {
            const auto & pixel = decoded_view(x, y);
            auto * p = &result.pixels[(y * result.width + x) * kBytesPerPixel];
            p[0] = pixel[0];
            p[1] = pixel[1];
            p[2] = pixel[2];
            p[3] = pixel[3];
        }
    }

    return result;
}

}  // namespace rover_led
