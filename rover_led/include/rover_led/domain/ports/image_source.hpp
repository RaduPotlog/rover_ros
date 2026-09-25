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

#ifndef ROVER_LED_DOMAIN_PORTS_IMAGE_SOURCE_HPP_
#define ROVER_LED_DOMAIN_PORTS_IMAGE_SOURCE_HPP_

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rover_led
{

// A decoded image: width x height pixels, row by row from the top-left, 4 bytes each
// (red, green, blue, straight alpha).
struct RgbaImage
{
    std::size_t width = 0;
    std::size_t height = 0;
    std::vector<std::uint8_t> pixels;
};

// Provides the pixels of the image an animation description names (its "image" value), so
// animations neither read files nor decode image formats. Throws std::runtime_error if it can't.
class IImageSource
{

public:

    virtual ~IImageSource() = default;

    virtual RgbaImage read(const std::string & image) const = 0;
};

}  // namespace rover_led

#endif  // ROVER_LED_DOMAIN_PORTS_IMAGE_SOURCE_HPP_
