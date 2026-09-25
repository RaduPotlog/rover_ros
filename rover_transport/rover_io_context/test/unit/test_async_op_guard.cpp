// Copyright 2026 Mechatronics Academy
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

#include <atomic>
#include <chrono>
#include <future>

#include <asio.hpp>

#include "rover_io_context/infrastructure/async_op_guard.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"

using namespace std::chrono_literals;
using rover::transport::AsyncOpGuard;
using rover::transport::IoContext;

namespace
{

constexpr auto kTimeout = 10s;

// Several io threads, so nothing is serialized unless the guard does it.
constexpr std::size_t kThreads = 4;

}  // namespace

TEST(AsyncOpGuardTest, CloseWaitsForARunningHandler)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());

    std::promise<void> started;
    std::promise<void> release;
    auto release_future = release.get_future().share();
    std::atomic<bool> handler_finished{false};
    guard.post(
        [&]()
        {
            started.set_value();
            release_future.wait();
            handler_finished = true;
        });
    ASSERT_EQ(started.get_future().wait_for(kTimeout), std::future_status::ready);

    std::atomic<bool> finished_when_closing{false};
    auto closing = std::async(
        std::launch::async, [&]()
        {
            guard.closeAndDrain([&]() {finished_when_closing = handler_finished.load();});
        });
    release.set_value();

    ASSERT_EQ(closing.wait_for(kTimeout), std::future_status::ready);
    EXPECT_TRUE(finished_when_closing);
}

TEST(AsyncOpGuardTest, CloseReturnsOnlyAfterTheCancelledOperationsHandlerRan)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());

    asio::steady_timer timer(ctx.ios(), 1h);
    std::atomic<bool> handler_ran{false};
    std::atomic<bool> aborted{false};
    guard.post(
        [&]()
        {
            timer.async_wait(
                guard.wrap(
                    [&](const asio::error_code & error)
                    {
                        aborted = error == asio::error::operation_aborted;
                        handler_ran = true;
                    }));
        });

    // Like closing a socket: cancels the pending operation, whose handler is still to run.
    guard.closeAndDrain([&]() {timer.cancel();});

    EXPECT_TRUE(handler_ran);
    EXPECT_TRUE(aborted);
}

TEST(AsyncOpGuardTest, CloseFromOneOfTheHandlersDoesNotWaitForItself)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());

    std::promise<void> closed;
    guard.post([&]() {guard.closeAndDrain([&]() {closed.set_value();});});

    EXPECT_EQ(closed.get_future().wait_for(kTimeout), std::future_status::ready);
}

TEST(AsyncOpGuardTest, CloseAfterTheContextStoppedRunsDirectly)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());
    ctx.waitForExit();

    bool closed = false;
    guard.closeAndDrain([&]() {closed = true;});

    EXPECT_TRUE(closed);
}

TEST(AsyncOpGuardTest, CloseAfterStopStillWaitsForAHandlerRunningOnAnotherThread)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());

    std::promise<void> started;
    std::promise<void> release;
    auto release_future = release.get_future().share();
    std::atomic<bool> handler_finished{false};
    guard.post(
        [&]()
        {
            started.set_value();
            release_future.wait();
            handler_finished = true;
        });
    ASSERT_EQ(started.get_future().wait_for(kTimeout), std::future_status::ready);

    // stop() without joining: the handler above keeps running on its io thread.
    ctx.ios().stop();
    ASSERT_TRUE(ctx.ios().stopped());

    std::atomic<bool> finished_when_closing{false};
    auto closing = std::async(
        std::launch::async, [&]()
        {
            guard.closeAndDrain([&]() {finished_when_closing = handler_finished.load();});
        });
    EXPECT_EQ(closing.wait_for(100ms), std::future_status::timeout)
        << "closed while a handler was still running";
    release.set_value();

    ASSERT_EQ(closing.wait_for(kTimeout), std::future_status::ready);
    EXPECT_TRUE(finished_when_closing);
}

TEST(AsyncOpGuardTest, PostedWorkIsSerialized)
{
    IoContext ctx(kThreads);
    AsyncOpGuard guard(ctx.ios());

    constexpr int kJobs = 2000;
    int counter = 0;  // Unsynchronized on purpose: the strand is the only protection.
    std::promise<void> all_done;
    for (int i = 0; i < kJobs; ++i) {
        guard.post(
            [&]()
            {
                if (++counter == kJobs) {
                    all_done.set_value();
                }
            });
    }

    ASSERT_EQ(all_done.get_future().wait_for(kTimeout), std::future_status::ready);
    guard.closeAndDrain([]() {});
    EXPECT_EQ(counter, kJobs);
}
