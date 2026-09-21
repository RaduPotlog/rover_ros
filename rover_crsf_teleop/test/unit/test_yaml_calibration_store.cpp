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

#include <filesystem>
#include <fstream>

#include <rclcpp/logging.hpp>

#include "rover_crsf_teleop/infrastructure/yaml_calibration_store.hpp"

namespace rover_crsf_teleop
{
namespace
{

class YamlCalibrationStoreTest : public ::testing::Test
{

protected:

    void SetUp() override
    {
        directory_ = std::filesystem::temp_directory_path() /
                     ("rover_crsf_teleop_store_" +
                      std::to_string(::testing::UnitTest::GetInstance()->random_seed()) + "_" +
                      ::testing::UnitTest::GetInstance()->current_test_info()->name());
        std::filesystem::remove_all(directory_);
        path_ = directory_ / "rc_calibration.yaml";
    }

    void TearDown() override { std::filesystem::remove_all(directory_); }

    YamlCalibrationStore store()
    {
        return YamlCalibrationStore(path_, rclcpp::get_logger("test_yaml_calibration_store"));
    }

    void write(const std::string & contents) const
    {
        std::filesystem::create_directories(directory_);
        std::ofstream out(path_, std::ios::trunc);
        out << contents;
    }

    std::filesystem::path directory_;
    std::filesystem::path path_;
};

ChannelCalibration sample()
{
    ChannelCalibration calibration = defaultCalibration();
    calibration.in_min[2] = 180;
    calibration.in_mid[2] = 1004;
    calibration.in_max[2] = 1800;
    calibration.deadband[2] = 11;
    return calibration;
}

TEST_F(YamlCalibrationStoreTest, AMissingFileIsNoCalibration)
{
    auto subject = store();
    EXPECT_FALSE(subject.load().has_value());
}

TEST_F(YamlCalibrationStoreTest, SaveCreatesTheDirectoryAndRoundTrips)
{
    auto subject = store();
    std::string error;

    // The persistent volume is empty on a fresh device, so the directory has to be created.
    ASSERT_TRUE(subject.save(sample(), error)) << error;
    ASSERT_TRUE(std::filesystem::exists(path_));

    const auto loaded = subject.load();
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->schema_version, kCalibrationSchemaVersion);
    EXPECT_FALSE(loaded->created.empty());
    EXPECT_EQ(loaded->calibration.in_min[2], 180);
    EXPECT_EQ(loaded->calibration.in_mid[2], 1004);
    EXPECT_EQ(loaded->calibration.in_max[2], 1800);
    EXPECT_EQ(loaded->calibration.deadband[2], 11);
    EXPECT_EQ(loaded->calibration.in_mid[0], kDefaultCrsfChannelMid);
}

TEST_F(YamlCalibrationStoreTest, SaveLeavesNoTemporaryFileBehind)
{
    auto subject = store();
    std::string error;
    ASSERT_TRUE(subject.save(sample(), error)) << error;

    // The rename is what makes the write atomic; a leftover .tmp means it did not happen.
    EXPECT_FALSE(std::filesystem::exists(path_.string() + ".tmp"));
}

TEST_F(YamlCalibrationStoreTest, SaveReplacesAnEarlierCalibration)
{
    auto subject = store();
    std::string error;
    ASSERT_TRUE(subject.save(defaultCalibration(), error)) << error;
    ASSERT_TRUE(subject.save(sample(), error)) << error;

    const auto loaded = subject.load();
    ASSERT_TRUE(loaded.has_value());
    EXPECT_EQ(loaded->calibration.in_mid[2], 1004);
}

TEST_F(YamlCalibrationStoreTest, GarbageIsIgnoredRatherThanThrown)
{
    // A corrupt calibration must not take the node down on boot - it falls back to the shipped
    // parameters instead.
    write("rc_calibration: [this is not a mapping\n");

    auto subject = store();
    EXPECT_NO_THROW({ EXPECT_FALSE(subject.load().has_value()); });
}

TEST_F(YamlCalibrationStoreTest, AFileWithoutTheRootKeyIsIgnored)
{
    write("something_else:\n  schema_version: 1\n");

    auto subject = store();
    EXPECT_FALSE(subject.load().has_value());
}

TEST_F(YamlCalibrationStoreTest, AnUnknownSchemaVersionIsIgnoredRatherThanGuessedAt)
{
    write("rc_calibration:\n  schema_version: 99\n  channel_in_min: [1]\n");

    auto subject = store();
    EXPECT_FALSE(subject.load().has_value());
}

TEST_F(YamlCalibrationStoreTest, AShortChannelArrayIsIgnoredRatherThanPadded)
{
    std::string contents = "rc_calibration:\n  schema_version: 1\n";
    contents += "  channel_in_min: [172, 172, 172]\n";
    contents += "  channel_in_mid: [992]\n";
    contents += "  channel_in_max: [1811]\n";
    contents += "  channel_deadband: [30]\n";
    write(contents);

    // Padding would silently map the missing channels to whatever the default happened to be.
    auto subject = store();
    EXPECT_FALSE(subject.load().has_value());
}

TEST_F(YamlCalibrationStoreTest, LocationNamesTheFile)
{
    auto subject = store();
    EXPECT_EQ(subject.location(), path_.string());
}

}  // namespace
}  // namespace rover_crsf_teleop
