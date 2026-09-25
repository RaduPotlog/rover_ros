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

#ifndef ROVER_SAFETY_INFRASTRUCTURE_COMMAND_HANDLER_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_COMMAND_HANDLER_HPP_

#include <sys/types.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

namespace rover_safety::infrastructure
{

enum class CommandState {
    IDLE = 0,
    RUNNING,
    SUCCESS,
    FAILURE,
};

/**
 * Runs a bash command in a child process and watches it from a background thread, so a behavior
 * tree can poll getState() without blocking. The command runs in its own process group, so a
 * timeout or halt() kills everything it spawned. execute() may be called again to re-run.
 *
 * Each run keeps its state in an Execution shared with its watcher thread. halt() waits a bounded
 * time for the watcher; should the child survive SIGKILL (a process stuck in uninterruptible
 * sleep), the watcher is detached with its Execution instead of hanging the tree.
 */
class CommandHandler
{

public:

    /** How long halt() waits for the watcher to reap a killed command. */
    static constexpr std::chrono::milliseconds kHaltTimeout{2000};

    CommandHandler() = default;

    CommandHandler(const CommandHandler &) = delete;
    CommandHandler & operator=(const CommandHandler &) = delete;

    ~CommandHandler();

    void execute(const std::string & command, const std::chrono::milliseconds & timeout);

    /** Kills a running command and waits, at most kHaltTimeout, for the watcher thread. */
    void halt();

    CommandState getState()
    {
        return execution_ ? execution_->state.load() : CommandState::IDLE;
    }

    std::string getOutput();

    std::string getError();

private:

    /** One run of a command, owned jointly by the handler and the run's watcher thread. */
    struct Execution
    {
        int read_fd{-1};
        pid_t child_pid{-1};
        std::chrono::milliseconds timeout{0};
        std::chrono::time_point<std::chrono::steady_clock> start_time;

        std::atomic<CommandState> state{CommandState::RUNNING};
        std::atomic<bool> kill_requested{false};

        // Guards output, error and done.
        std::mutex mtx;
        std::condition_variable done_cv;
        std::string output;
        std::string error;
        bool done{false};

        void setError(const std::string & message)
        {
            std::lock_guard<std::mutex> lock(mtx);
            error = message;
        }
    };

    static void watch(const std::shared_ptr<Execution> & execution);

    static bool startChild(Execution & execution, const std::string & command);

    /** Appends whatever the pipe holds to the output; false when nothing was read. */
    static bool readOutput(Execution & execution);

    std::shared_ptr<Execution> execution_;
    std::thread watcher_;
};

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_COMMAND_HANDLER_HPP_
