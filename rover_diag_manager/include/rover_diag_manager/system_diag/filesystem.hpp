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

#ifndef ROVER_DIAG_MANAGER_SYSTEM_DIAG_FILESYSTEM_HPP_
#define ROVER_DIAG_MANAGER_SYSTEM_DIAG_FILESYSTEM_HPP_

#include <filesystem>
#include <fstream>

namespace rover_diag_manager
{

class FilesystemInterface
{

public:

    virtual ~FilesystemInterface() = default;

    virtual uintmax_t getSpaceCapacity(const std::string & filesystem_path) const = 0;

    virtual uintmax_t getSpaceAvailable(const std::string & filesystem_path) const = 0;

    virtual std::string readFile(const std::string & file_path) const = 0;

    using SharedPtr = std::shared_ptr<FilesystemInterface>;
};

class Filesystem : public FilesystemInterface
{
public:

    inline uintmax_t getSpaceCapacity(const std::string & filesystem_path) const override
    {
        const auto path = std::filesystem::path(filesystem_path);
        const auto space_info = std::filesystem::space(path);

        return space_info.capacity;
    }

    inline uintmax_t getSpaceAvailable(const std::string & filesystem_path) const override
    {
        const auto path = std::filesystem::path(filesystem_path);
        const auto space_info = std::filesystem::space(path);

        return space_info.available;
    }

    std::string readFile(const std::string & file_path) const override
    {
        const auto path = std::filesystem::path(file_path);

        if (!std::filesystem::exists(path)) {
            throw std::invalid_argument("File doesn't exist, given path " + path.string());
        }

        std::ifstream file(path);
        
        if (!file.is_open()) {
            throw std::runtime_error("Failed to open, given path " + path.string());
        }

        std::stringstream buffer;
        buffer << file.rdbuf();

        file.close();
        
        return buffer.str();
    }
};

}  // namespace rover_diag_manager

#endif  // ROVER_DIAG_MANAGER_SYSTEM_DIAG_FILESYSTEM_HPP_
