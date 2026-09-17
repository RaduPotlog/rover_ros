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
// v1.2.0; restyled; the rclcpp "Thread(s) Created" log removed to keep this package free
// of ROS dependencies.

#include "rover_io_context/infrastructure/io_context.hpp"

#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>

namespace rover::transport
{

IoContext::IoContext()
: IoContext(std::thread::hardware_concurrency())
{
}

IoContext::IoContext(std::size_t threads_count)
: ios_(new asio::io_service()),
  work_(new asio::io_service::work(ios())),
  ios_thread_workers_(new ThreadGroup())
{
    for (std::size_t i = 0; i < threads_count; ++i) {
        ios_thread_workers_->createThread(
            [this]()
            {
                ios().run();
            });
    }
}

IoContext::~IoContext()
{
    waitForExit();
}

asio::io_service & IoContext::ios() const
{
    return *ios_;
}

bool IoContext::isServiceStopped()
{
    return ios().stopped();
}

std::uint32_t IoContext::serviceThreadCount()
{
    return static_cast<std::uint32_t>(ios_thread_workers_->size());
}

void IoContext::waitForExit()
{
    if (!ios().stopped()) {
        ios().post([&]() {work_.reset();});
    }

    ios().stop();
    ios_thread_workers_->joinAll();
}

}  // namespace rover::transport
