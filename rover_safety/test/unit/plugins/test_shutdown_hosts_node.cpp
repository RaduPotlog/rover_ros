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


#include <gmock/gmock.h>
#include <gtest/gtest.h>

#include <memory>
#include <string>
#include <vector>

#include "rover_safety/plugins/shutdown_hosts_node.hpp"

using rover_safety::ShutdownHostInterface;
using rover_safety::ShutdownHostState;
using HostList = std::vector<std::shared_ptr<ShutdownHostInterface>>;

namespace
{

class MockShutdownHost : public ShutdownHostInterface
{
public:
    explicit MockShutdownHost(std::size_t hash) : ShutdownHostInterface(hash) {}

    MOCK_METHOD(void, call, (), (override));
    MOCK_METHOD(void, halt, (), (override));
    MOCK_METHOD(std::string, getIp, (), (const, override));
    MOCK_METHOD(std::string, getError, (), (const, override));
    MOCK_METHOD(std::string, getOutput, (), (const, override));
    MOCK_METHOD(ShutdownHostState, getState, (), (const, override));

    using NiceMock = testing::NiceMock<MockShutdownHost>;
};

class ShutdownHostsWrapper : public rover_safety::ShutdownHosts
{
public:
    ShutdownHostsWrapper(const std::string & name, const BT::NodeConfig & conf)
    : rover_safety::ShutdownHosts(name, conf)
    {
    }

    using rover_safety::ShutdownHosts::onRunning;
    using rover_safety::ShutdownHosts::onStart;
    using rover_safety::ShutdownHosts::removeDuplicatedHosts;

    static BT::PortsList providedPorts() { return {}; }

    bool updateHosts(HostList & hosts) override
    {
        hosts = hosts_to_set_;
        return update_hosts_success_;
    }

    void setHosts(HostList hosts, bool success)
    {
        hosts_to_set_ = std::move(hosts);
        update_hosts_success_ = success;
    }

private:
    HostList hosts_to_set_;
    bool update_hosts_success_{true};
};

class ShutdownHostsNodeTest : public testing::Test
{
protected:
    void createWrapper(HostList hosts, bool success)
    {
        wrapper_ = std::make_unique<ShutdownHostsWrapper>("ShutdownHosts", BT::NodeConfig());
        wrapper_->setHosts(std::move(hosts), success);
    }

    BT::NodeStatus runToCompletion()
    {
        auto status = wrapper_->onStart();
        for (int i = 0; i < 100 && status == BT::NodeStatus::RUNNING; ++i) {
            status = wrapper_->onRunning();
        }
        return status;
    }

    std::unique_ptr<ShutdownHostsWrapper> wrapper_;
};

}  // namespace

TEST_F(ShutdownHostsNodeTest, RemovesDuplicatedHosts)
{
    using rover_safety::ShutdownHost;
    HostList hosts = {
        std::make_shared<ShutdownHost>("127.0.0.1", "3003", "password", 1.0),
        std::make_shared<ShutdownHost>("localhost", "3003", "password", 1.0),
        std::make_shared<ShutdownHost>("localhost", "3003", "password", 1.0),
        std::make_shared<ShutdownHost>("127.0.0.1", "3003", "password", 1.0),
        std::make_shared<ShutdownHost>("127.0.0.1", "8080", "password", 1.0),
    };
    createWrapper({}, true);

    wrapper_->removeDuplicatedHosts(hosts);
    EXPECT_EQ(hosts.size(), 3u);
}

TEST_F(ShutdownHostsNodeTest, SucceedsWhenHostsSucceedOrAreSkipped)
{
    auto host_1 = std::make_shared<MockShutdownHost::NiceMock>(0);
    auto host_2 = std::make_shared<MockShutdownHost::NiceMock>(1);
    EXPECT_CALL(*host_1, call()).Times(1);
    EXPECT_CALL(*host_2, call()).Times(1);
    ON_CALL(*host_1, getState()).WillByDefault(testing::Return(ShutdownHostState::SUCCESS));
    ON_CALL(*host_2, getState()).WillByDefault(testing::Return(ShutdownHostState::SKIPPED));
    createWrapper({host_1, host_2}, true);

    EXPECT_EQ(runToCompletion(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsNodeTest, ResponseReceivedKeepsPolling)
{
    auto host = std::make_shared<MockShutdownHost::NiceMock>(0);
    createWrapper({host}, true);

    EXPECT_EQ(wrapper_->onStart(), BT::NodeStatus::RUNNING);

    EXPECT_CALL(*host, getState())
        .WillOnce(testing::Return(ShutdownHostState::RESPONSE_RECEIVED))
        .WillRepeatedly(testing::Return(ShutdownHostState::SUCCESS));

    EXPECT_EQ(wrapper_->onRunning(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(wrapper_->onRunning(), BT::NodeStatus::RUNNING);
    EXPECT_EQ(wrapper_->onRunning(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsNodeTest, FailsWhenAHostFails)
{
    auto host = std::make_shared<MockShutdownHost::NiceMock>(0);
    ON_CALL(*host, getState()).WillByDefault(testing::Return(ShutdownHostState::FAILURE));
    createWrapper({host}, true);

    EXPECT_EQ(runToCompletion(), BT::NodeStatus::FAILURE);
    EXPECT_EQ(wrapper_->getFailedHosts().size(), 1u);
}

TEST_F(ShutdownHostsNodeTest, FailsWhenUpdateHostsFails)
{
    createWrapper({std::make_shared<MockShutdownHost::NiceMock>(0)}, false);
    EXPECT_EQ(wrapper_->onStart(), BT::NodeStatus::FAILURE);
}

TEST_F(ShutdownHostsNodeTest, NoHostsSucceeds)
{
    createWrapper({}, true);
    EXPECT_EQ(wrapper_->onStart(), BT::NodeStatus::SUCCESS);
}

TEST_F(ShutdownHostsNodeTest, RestartForgetsPreviousRun)
{
    auto failing = std::make_shared<MockShutdownHost::NiceMock>(0);
    ON_CALL(*failing, getState()).WillByDefault(testing::Return(ShutdownHostState::FAILURE));
    createWrapper({failing}, true);
    ASSERT_EQ(runToCompletion(), BT::NodeStatus::FAILURE);

    auto succeeding = std::make_shared<MockShutdownHost::NiceMock>(1);
    ON_CALL(*succeeding, getState()).WillByDefault(testing::Return(ShutdownHostState::SUCCESS));
    wrapper_->setHosts({succeeding}, true);

    EXPECT_EQ(runToCompletion(), BT::NodeStatus::SUCCESS);
    EXPECT_TRUE(wrapper_->getFailedHosts().empty());
}
