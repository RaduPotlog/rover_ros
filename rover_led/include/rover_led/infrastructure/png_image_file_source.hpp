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

#ifndef ROVER_LED_INFRASTRUCTURE_PNG_IMAGE_FILE_SOURCE_HPP_
#define ROVER_LED_INFRASTRUCTURE_PNG_IMAGE_FILE_SOURCE_HPP_

#include <string>

#include "rover_led/domain/ports/image_source.hpp"

namespace rover_led
{

// Reads animation images from PNG files (any PNG colour type, converted to RGBA8). The name is an
// absolute path: "$(find <pkg>)" is resolved when the catalog is loaded (resolvePackageSubstitution).
class PngImageFileSource : public IImageSource
{

public:

    // Throws std::runtime_error for a relative or missing path, or a file that isn't a PNG
    // (std::ios_base::failure from Boost.GIL; since C++11 this derives from std::system_error,
    // which derives from std::runtime_error, so catching runtime_error still catches it).
    RgbaImage read(const std::string & image) const override;
};

}  // namespace rover_led

#endif  // ROVER_LED_INFRASTRUCTURE_PNG_IMAGE_FILE_SOURCE_HPP_
