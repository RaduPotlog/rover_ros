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
// v1.2.0 into rover_io_context/infrastructure; io_context/common.hpp folded in; restyled
// to the workspace convention; the rclcpp log line dropped so this package carries no ROS
// dependency.

#ifndef ROVER_IO_CONTEXT_INFRASTRUCTURE_IO_CONTEXT_HPP_
#define ROVER_IO_CONTEXT_INFRASTRUCTURE_IO_CONTEXT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <thread>
#include <utility>
#include <vector>

#include <asio.hpp>

namespace rover::transport
{

// A workaround for boost::thread_group.
// Copied from https://gist.github.com/coin-au-carre/ceb8a790cec3b3535b015be3ec2a1ce2
struct ThreadGroup
{
    std::vector<std::thread> threads;

    ThreadGroup() = default;
    ThreadGroup(const ThreadGroup &) = delete;
    ThreadGroup & operator=(const ThreadGroup &) = delete;
    ThreadGroup(ThreadGroup &&) = delete;
    ThreadGroup & operator=(ThreadGroup &&) = delete;

    template<class ... Args>
    void createThread(Args && ... args)
    {
        threads.emplace_back(std::forward<Args>(args)...);
    }

    void addThread(std::thread && thread)
    {
        threads.emplace_back(std::move(thread));
    }

    std::size_t size() const
    {
        return threads.size();
    }

    void joinAll()
    {
        for (auto & thread : threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }
};

// Owns an ASIO io_service and the pool of threads running it. Every ByteStreamPort adapter
// in rover_serial_driver / rover_udp_driver borrows one of these by const reference; the
// node owns it, so the context always outlives the ports it serves.
class IoContext
{

public:

    IoContext();

    explicit IoContext(std::size_t threads_count);

    ~IoContext();

    IoContext(const IoContext &) = delete;
    IoContext & operator=(const IoContext &) = delete;

    asio::io_service & ios() const;

    bool isServiceStopped();

    std::uint32_t serviceThreadCount();

    void waitForExit();

    template<class F>
    void post(F f)
    {
        ios().post(f);
    }

private:

    std::shared_ptr<asio::io_service> ios_;
    std::shared_ptr<asio::io_service::work> work_;
    std::shared_ptr<ThreadGroup> ios_thread_workers_;
};

}  // namespace rover::transport

#endif  // ROVER_IO_CONTEXT_INFRASTRUCTURE_IO_CONTEXT_HPP_
