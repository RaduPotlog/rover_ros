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

#ifndef ROVER_SAFETY_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
#define ROVER_SAFETY_PLUGINS_ACTION_COMMAND_HANDLER_HPP_

#include <fcntl.h>
#include <signal.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdlib>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include "rover_safety/behavior_tree_utils.hpp"

namespace rover_safety
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

    ~CommandHandler()
    {
        halt();
    }

    void execute(const std::string & command, const std::chrono::milliseconds & timeout);

    /** Kills a running command and waits, at most kHaltTimeout, for the watcher thread. */
    void halt();

    CommandState getState()
    {
        return execution_ ? execution_->state.load() : CommandState::IDLE;
    }

    std::string getOutput()
    {
        if (!execution_) {
            return "";
        }

        std::lock_guard<std::mutex> lock(execution_->mtx);

        return execution_->output;
    }

    std::string getError()
    {
        if (!execution_) {
            return "";
        }

        std::lock_guard<std::mutex> lock(execution_->mtx);

        return execution_->error;
    }

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

inline void CommandHandler::execute(
    const std::string & command,
    const std::chrono::milliseconds & timeout_ms)
{
    halt();

    execution_ = std::make_shared<Execution>();
    execution_->timeout = timeout_ms;

    if (!startChild(*execution_, command)) {
        execution_->state = CommandState::FAILURE;
        return;
    }

    watcher_ = std::thread(&CommandHandler::watch, execution_);
}

inline void CommandHandler::halt()
{
    if (!watcher_.joinable()) {
        return;
    }

    // The watcher thread owns the child: it kills, reaps it and closes the pipe.
    execution_->kill_requested = true;

    bool finished = false;
    {
        std::unique_lock<std::mutex> lock(execution_->mtx);
        finished = execution_->done_cv.wait_for(
            lock, kHaltTimeout, [this]() { return execution_->done; });
    }

    if (finished) {
        watcher_.join();
        return;
    }

    // The child did not die from SIGKILL. The watcher keeps its own reference to the Execution,
    // so letting it go is safe. Report from a fresh Execution: should the child be reaped later,
    // the detached watcher must not rewrite the outcome callers already saw.
    watcher_.detach();
    execution_ = std::make_shared<Execution>();
    execution_->setError("Command did not exit after SIGKILL");
    execution_->state = CommandState::FAILURE;
}

inline void CommandHandler::watch(const std::shared_ptr<Execution> & execution)
{
    bool killed = false;
    std::string kill_reason;

    while (true) {
        while (readOutput(*execution)) {
        }

        int status = 0;

        if (waitpid(execution->child_pid, &status, WNOHANG) == execution->child_pid) {
            while (readOutput(*execution)) {
            }
            close(execution->read_fd);
            execution->read_fd = -1;

            if (killed) {
                execution->setError(kill_reason);
                execution->state = CommandState::FAILURE;
            } else if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
                execution->state = CommandState::SUCCESS;
            } else if (WIFEXITED(status)) {
                execution->setError("Command return code: " + std::to_string(WEXITSTATUS(status)));
                execution->state = CommandState::FAILURE;
            } else {
                execution->setError("Command terminated by signal " + std::to_string(WTERMSIG(status)));
                execution->state = CommandState::FAILURE;
            }

            {
                std::lock_guard<std::mutex> lock(execution->mtx);
                execution->done = true;
            }
            execution->done_cv.notify_all();

            return;
        }

        if (!killed &&
            (execution->kill_requested.load() || timeoutExceeded(execution->start_time, execution->timeout)))
        {
            kill_reason = execution->kill_requested.load() ? "Command halted" : "Timeout exceeded";
            // Negative pid: the whole process group started by the command. Falls back to the child
            // alone should the group not exist yet.
            if (kill(-execution->child_pid, SIGKILL) != 0) {
                kill(execution->child_pid, SIGKILL);
            }
            killed = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

inline bool CommandHandler::startChild(Execution & execution, const std::string & command)
{
    int pipefd[2]{-1, -1};

    if (pipe(pipefd) == -1) {
        execution.setError("Failed to create pipe");

        return false;
    }

    // Non-blocking read end: the watcher thread polls it.
    int flags = fcntl(pipefd[0], F_GETFL, 0);
    fcntl(pipefd[0], F_SETFL, flags | O_NONBLOCK);

    execution.child_pid = fork();
    execution.start_time = std::chrono::steady_clock::now();

    if (execution.child_pid == -1) {
        close(pipefd[0]);
        close(pipefd[1]);
        execution.setError("Failed to fork");

        return false;
    }

    if (execution.child_pid == 0) {
        setpgid(0, 0);                   // Own process group, killed as a whole
        close(pipefd[0]);                // Close unused read end
        dup2(pipefd[1], STDOUT_FILENO);  // Redirect stdout to pipe
        dup2(pipefd[1], STDERR_FILENO);  // Redirect stderr to pipe
        close(pipefd[1]);                // Close write end after redirecting

        execl("/bin/bash", "bash", "-c", command.c_str(), nullptr);
        _exit(127);
    }

    // Also set from the parent so a kill right after fork() cannot miss the group.
    setpgid(execution.child_pid, execution.child_pid);
    close(pipefd[1]);  // Close unused write end
    execution.read_fd = pipefd[0];

    return true;
}

inline bool CommandHandler::readOutput(Execution & execution)
{
    char buffer[128];

    const ssize_t bytes_read = read(execution.read_fd, buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        std::lock_guard<std::mutex> lock(execution.mtx);
        execution.output += buffer;

        return true;
    }

    return false;
}

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
