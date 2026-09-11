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

#include <gtest/gtest.h>

#include <map>
#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"

#include "rover_diag_manager/infrastructure/linux_system_metrics_source.hpp"

using rover_diag_manager::infrastructure::FilesystemInterface;
using rover_diag_manager::infrastructure::LinuxSystemMetricsSource;

namespace
{

/** @brief In-memory FilesystemInterface double; per-path files, and fixed disk capacity. */
class FakeFilesystem : public FilesystemInterface
{
public:
    std::uintmax_t getSpaceCapacity(const std::string &) const override {return capacity;}

    std::uintmax_t getSpaceAvailable(const std::string &) const override {return available;}

    std::string readFile(const std::string & file_path) const override
    {
        const auto it = files.find(file_path);
        if (it == files.end()) {
            throw std::invalid_argument("File doesn't exist, given path " + file_path);
        }
        return it->second;
    }

    std::map<std::string, std::string> files;
    std::uintmax_t capacity{1000};
    std::uintmax_t available{600};
};

class LinuxSystemMetricsSourceTest : public ::testing::Test
{
protected:
    static void SetUpTestSuite() {rclcpp::init(0, nullptr);}
    static void TearDownTestSuite() {rclcpp::shutdown();}

    std::shared_ptr<FakeFilesystem> filesystem_ = std::make_shared<FakeFilesystem>();
};

}  // namespace

TEST_F(LinuxSystemMetricsSourceTest, RejectsNullFilesystem)
{
    EXPECT_THROW(
        LinuxSystemMetricsSource(nullptr, rclcpp::get_logger("test")), std::invalid_argument);
}

TEST_F(LinuxSystemMetricsSourceTest, ParsesTemperatureFromMillidegrees)
{
    filesystem_->files[LinuxSystemMetricsSource::kTemperatureInfoFilename] = "45123";

    LinuxSystemMetricsSource source(filesystem_, rclcpp::get_logger("test"));
    const auto sample = source.sample();

    ASSERT_TRUE(sample.cpu_temperature.has_value());
    EXPECT_FLOAT_EQ(*sample.cpu_temperature, 45.12f);
}

TEST_F(LinuxSystemMetricsSourceTest, MissingTemperatureFileYieldsNullopt)
{
    LinuxSystemMetricsSource source(filesystem_, rclcpp::get_logger("test"));
    const auto sample = source.sample();

    EXPECT_FALSE(sample.cpu_temperature.has_value());
}

TEST_F(LinuxSystemMetricsSourceTest, ComputesDiskUsageFromCapacityAndAvailable)
{
    filesystem_->capacity = 1000;
    filesystem_->available = 750;   // 25% used

    LinuxSystemMetricsSource source(filesystem_, rclcpp::get_logger("test"));
    const auto sample = source.sample();

    ASSERT_TRUE(sample.disk_usage.has_value());
    EXPECT_FLOAT_EQ(*sample.disk_usage, 25.0f);
}

TEST_F(LinuxSystemMetricsSourceTest, ZeroCapacityYieldsNulloptDiskUsage)
{
    filesystem_->capacity = 0;
    filesystem_->available = 0;

    LinuxSystemMetricsSource source(filesystem_, rclcpp::get_logger("test"));
    const auto sample = source.sample();

    EXPECT_FALSE(sample.disk_usage.has_value());
}
