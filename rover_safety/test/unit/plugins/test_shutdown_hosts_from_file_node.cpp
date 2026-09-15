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

#include <cstdio>
#include <filesystem>
#include <fstream>
#include <string>

#include "behaviortree_cpp/bt_factory.h"

#include "rover_safety/plugins/action/shutdown_hosts_from_file_node.hpp"

#include "plugin_test_utils.hpp"

using rover_safety::test::OneShotHttpServer;
using rover_safety::test::singleNodeTree;
using rover_safety::test::tickUntilDone;

namespace
{

class ShutdownHostsFromFileTest : public ::testing::Test
{
protected:
    void SetUp() override
    {
        factory_.registerNodeType<rover_safety::ShutdownHostsFromFile>("ShutdownHostsFromFile");
        const auto * test_info = ::testing::UnitTest::GetInstance()->current_test_info();
        file_path_ = std::filesystem::temp_directory_path() /
                     (std::string("rover_safety_") + test_info->name() + "_hosts.yaml");
    }

    void TearDown() override { std::filesystem::remove(file_path_); }

    BT::NodeStatus runWithFile(const std::string & yaml)
    {
        std::ofstream(file_path_) << yaml;
        return runWithPath(file_path_.string());
    }

    BT::NodeStatus runWithPath(const std::string & path)
    {
        auto tree = factory_.createTreeFromText(
            singleNodeTree("ShutdownHostsFromFile", {{"shutdown_hosts_file", path}}));
        return tickUntilDone(tree);
    }

    BT::BehaviorTreeFactory factory_;
    std::filesystem::path file_path_;
};

}  // namespace

TEST_F(ShutdownHostsFromFileTest, WrongNodeNameThrows)
{
    EXPECT_THROW(
        { auto tree = factory_.createTreeFromText(singleNodeTree("WrongShutdownHostsFromFile", {})); },
        BT::RuntimeError);
}

TEST_F(ShutdownHostsFromFileTest, MissingFileFails)
{
    EXPECT_EQ(runWithPath(file_path_.string()), BT::NodeStatus::FAILURE);
}

TEST_F(ShutdownHostsFromFileTest, EmptyPathFails)
{
    EXPECT_EQ(runWithPath(""), BT::NodeStatus::FAILURE);
}

TEST_F(ShutdownHostsFromFileTest, EmptyHostListSucceeds)
{
    EXPECT_EQ(runWithFile("hosts: []\n"), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsFromFileTest, FileWithoutHostsSucceeds)
{
    EXPECT_EQ(runWithFile("# only comments\n"), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsFromFileTest, HostsNotAListFails)
{
    EXPECT_EQ(runWithFile("hosts:\n  ip: 127.0.0.1\n"), BT::NodeStatus::FAILURE);
}

TEST_F(ShutdownHostsFromFileTest, HostWithoutIpIsIgnored)
{
    EXPECT_EQ(runWithFile("hosts:\n  - port: 3003\n"), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsFromFileTest, UnreachableHostIsSkipped)
{
    EXPECT_EQ(
        runWithFile(std::string("hosts:\n  - ip: ") + rover_safety::test::kUnreachableIp + "\n    timeout: 1.0\n"),
        BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsFromFileTest, HostRefusingShutdownFails)
{
    OneShotHttpServer server("403 Forbidden");

    EXPECT_EQ(
        runWithFile(
            "hosts:\n  - ip: 127.0.0.1\n    port: " + server.port() + "\n    secret: s\n    timeout: 1.0\n"),
        BT::NodeStatus::FAILURE);
    EXPECT_NE(server.request().find("GET /shutdown?ts="), std::string::npos);
}
