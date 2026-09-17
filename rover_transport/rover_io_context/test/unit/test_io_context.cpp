// Copyright 2021 LeoDrive.
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

// Developed by LeoDrive, 2021
//
// Modified 2026 by Mechatronics Academy: relayouted from ros-drivers/transport_drivers
// v1.2.0; restyled; namespace updated.

#include <gtest/gtest.h>

#include <cstddef>
#include <cstdint>
#include <functional>
#include <memory>
#include <thread>
#include <vector>

#include "rover_io_context/infrastructure/io_context.hpp"

using rover::transport::IoContext;

namespace
{

constexpr std::size_t kLength = 10;

void counter(std::shared_ptr<std::vector<std::int32_t>> container, std::int32_t count)
{
    container->push_back(count);
}

void checkContainerSize(std::shared_ptr<std::vector<std::int32_t>> container)
{
    EXPECT_EQ(container->size(), kLength);
}

}  // namespace

TEST(IoContextTest, DefaultLifeCycleTest)
{
    IoContext ctx;
    EXPECT_EQ(ctx.isServiceStopped(), false);
    EXPECT_EQ(ctx.serviceThreadCount(), std::thread::hardware_concurrency());
}

TEST(IoContextTest, ConcurrentLifeCycleTest)
{
    IoContext ctx(kLength);
    EXPECT_EQ(ctx.isServiceStopped(), false);
    EXPECT_EQ(ctx.serviceThreadCount(), kLength);
}

TEST(IoContextTest, SingleThreadPostTaskTest)
{
    IoContext ctx(1);
    EXPECT_EQ(ctx.isServiceStopped(), false);
    EXPECT_EQ(ctx.serviceThreadCount(), std::uint32_t(1));

    std::shared_ptr<std::vector<std::int32_t>> container(new std::vector<std::int32_t>());
    for (std::size_t i = 0; i < kLength; ++i) {
        ctx.post(std::bind(counter, container, static_cast<std::int32_t>(i)));
    }
    ctx.post(std::bind(checkContainerSize, container));
}
