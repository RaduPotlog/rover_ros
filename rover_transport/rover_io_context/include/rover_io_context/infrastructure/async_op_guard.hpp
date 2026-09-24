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

#ifndef ROVER_IO_CONTEXT_INFRASTRUCTURE_ASYNC_OP_GUARD_HPP_
#define ROVER_IO_CONTEXT_INFRASTRUCTURE_ASYNC_OP_GUARD_HPP_

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstddef>
#include <functional>
#include <future>
#include <memory>
#include <mutex>
#include <utility>

#include <asio.hpp>

namespace rover::transport
{

// Makes close() of an ASIO stream safe against its own completion handlers.
//
// Closing an ASIO socket or port does not wait for a handler that is already running on
// another io thread, nor for one that completed before the close and is still queued. Either
// can then call into the receive callback, or into the stream object itself, after the owner
// has destroyed them. The guard runs every handler it wraps on one strand, counts the ones
// still outstanding, and closeAndDrain() runs the close on that strand and returns only once
// all of them have finished.
class AsyncOpGuard
{

public:

    explicit AsyncOpGuard(asio::io_context & ios)
    : ios_(ios),
      strand_(asio::make_strand(ios.get_executor()))
    {
    }

    AsyncOpGuard(const AsyncOpGuard &) = delete;
    AsyncOpGuard & operator=(const AsyncOpGuard &) = delete;

    // Wraps a completion handler to run on the strand and count as outstanding until it
    // returns. Call it when the operation is started, not later.
    template<class Handler>
    auto wrap(Handler handler)
    {
        begin();
        return asio::bind_executor(
            strand_,
            [this, handler = std::move(handler)](auto && ... args) mutable
            {
                handler(std::forward<decltype(args)>(args)...);
                end();
            });
    }

    // Runs `work` on the strand, counted like a handler. For starting operations from outside
    // the io threads, so every use of the stream happens on the strand.
    void post(std::function<void()> work)
    {
        asio::post(wrap([work = std::move(work)]() {work();}));
    }

    // Runs `close` on the strand, then waits until every wrapped handler has returned (a
    // closed stream completes its pending operations with operation_aborted). Closes directly,
    // without waiting, when called from one of the handlers or once the io_context has stopped
    // - no handler can run then, and waiting would never end.
    void closeAndDrain(const std::function<void()> & close)
    {
        if (strand_.running_in_this_thread() || ios_.stopped()) {
            close();
            return;
        }

        struct State
        {
            std::atomic<bool> claimed{false};
            std::promise<void> done;
        };
        auto state = std::make_shared<State>();
        auto done = state->done.get_future();
        asio::post(
            strand_, [state, close]()
            {
                if (!state->claimed.exchange(true)) {
                    close();
                }
                state->done.set_value();
            });

        while (done.wait_for(kPollPeriod) != std::future_status::ready) {
            if (ios_.stopped()) {
                // The posted close may never run now. Close here, unless it is running.
                if (!state->claimed.exchange(true)) {
                    close();
                } else {
                    done.wait();
                }
                break;
            }
        }

        std::unique_lock<std::mutex> lock(mutex_);
        while (!drained_.wait_for(lock, kPollPeriod, [this]() {return outstanding_ == 0;})) {
            if (ios_.stopped()) {
                break;  // The queued handlers will never run.
            }
        }
    }

private:

    static constexpr std::chrono::milliseconds kPollPeriod{20};

    void begin()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        ++outstanding_;
    }

    void end()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (--outstanding_ == 0) {
            drained_.notify_all();
        }
    }

    asio::io_context & ios_;
    asio::strand<asio::io_context::executor_type> strand_;
    std::mutex mutex_;
    std::condition_variable drained_;
    std::size_t outstanding_{0};
};

}  // namespace rover::transport

#endif  // ROVER_IO_CONTEXT_INFRASTRUCTURE_ASYNC_OP_GUARD_HPP_
