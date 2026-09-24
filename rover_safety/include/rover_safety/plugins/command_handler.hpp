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
 */
class CommandHandler
{

public:

    CommandHandler() = default;

    CommandHandler(const CommandHandler &) = delete;
    CommandHandler & operator=(const CommandHandler &) = delete;

    ~CommandHandler()
    {
        halt();
    }

    void execute(const std::string & command, const std::chrono::milliseconds & timeout);

    /** Kills a running command and waits for the watcher thread. No-op when nothing runs. */
    void halt();

    CommandState getState()
    {
        return state_.load();
    }

    std::string getOutput()
    {
        std::lock_guard<std::mutex> lock(output_mtx_);

        return output_;
    }

    std::string getError()
    {
        std::lock_guard<std::mutex> lock(error_mtx_);

        return error_;
    }

private:

    void checkExecution();

    bool executeCommandInChildProcess(const std::string & command);

    /** Appends whatever the pipe holds to the output; false when nothing was read. */
    bool readCommandOutput();

    void setError(const std::string & error)
    {
        std::lock_guard<std::mutex> lock(error_mtx_);
        error_ = error;
    }

    int pipefd_[2]{-1, -1};
    pid_t child_pid_{-1};
    std::chrono::milliseconds timeout_ms_{0};
    std::chrono::time_point<std::chrono::steady_clock> command_time_;

    std::atomic<CommandState> state_{CommandState::IDLE};
    std::atomic<bool> kill_requested_{false};
    std::string output_;
    std::mutex output_mtx_;
    std::string error_;
    std::mutex error_mtx_;
    std::thread command_checker_thread_;
};

inline void CommandHandler::execute(
    const std::string & command,
    const std::chrono::milliseconds & timeout_ms)
{
    halt();

    {
        std::lock_guard<std::mutex> lock(output_mtx_);
        output_.clear();
    }
    setError("");

    timeout_ms_ = timeout_ms;
    kill_requested_ = false;
    state_ = CommandState::RUNNING;

    if (!executeCommandInChildProcess(command)) {
        state_ = CommandState::FAILURE;
    }
}

inline void CommandHandler::halt()
{
    if (command_checker_thread_.joinable()) {
        // The watcher thread owns the child: it kills, reaps it and closes the pipe.
        kill_requested_ = true;
        command_checker_thread_.join();
    }
}

inline void CommandHandler::checkExecution()
{
    bool killed = false;
    std::string kill_reason;

    while (true) {
        while (readCommandOutput()) {
        }

        int status = 0;

        if (waitpid(child_pid_, &status, WNOHANG) == child_pid_) {
            while (readCommandOutput()) {
            }
            close(pipefd_[0]);
            pipefd_[0] = -1;

            if (killed) {
                setError(kill_reason);
                state_ = CommandState::FAILURE;
            } else if (WIFEXITED(status) && WEXITSTATUS(status) == 0) {
                state_ = CommandState::SUCCESS;
            } else if (WIFEXITED(status)) {
                setError("Command return code: " + std::to_string(WEXITSTATUS(status)));
                state_ = CommandState::FAILURE;
            } else {
                setError("Command terminated by signal " + std::to_string(WTERMSIG(status)));
                state_ = CommandState::FAILURE;
            }

            return;
        }

        if (!killed && (kill_requested_.load() || timeoutExceeded(command_time_, timeout_ms_))) {
            kill_reason = kill_requested_.load() ? "Command halted" : "Timeout exceeded";
            // Negative pid: the whole process group started by the command. Falls back to the child
            // alone should the group not exist yet.
            if (kill(-child_pid_, SIGKILL) != 0) {
                kill(child_pid_, SIGKILL);
            }
            killed = true;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
}

inline bool CommandHandler::executeCommandInChildProcess(const std::string & command)
{
    if (pipe(pipefd_) == -1) {
        setError("Failed to create pipe");

        return false;
    }

    // Non-blocking read end: the watcher thread polls it.
    int flags = fcntl(pipefd_[0], F_GETFL, 0);
    fcntl(pipefd_[0], F_SETFL, flags | O_NONBLOCK);

    child_pid_ = fork();
    command_time_ = std::chrono::steady_clock::now();

    if (child_pid_ == -1) {
        close(pipefd_[0]);
        close(pipefd_[1]);
        setError("Failed to fork");

        return false;
    }

    if (child_pid_ == 0) {
        setpgid(0, 0);                    // Own process group, killed as a whole
        close(pipefd_[0]);                // Close unused read end
        dup2(pipefd_[1], STDOUT_FILENO);  // Redirect stdout to pipe
        dup2(pipefd_[1], STDERR_FILENO);  // Redirect stderr to pipe
        close(pipefd_[1]);                // Close write end after redirecting

        execl("/bin/bash", "bash", "-c", command.c_str(), nullptr);
        _exit(127);
    }

    // Also set from the parent so a kill right after fork() cannot miss the group.
    setpgid(child_pid_, child_pid_);
    close(pipefd_[1]);  // Close unused write end

    command_checker_thread_ = std::thread(&CommandHandler::checkExecution, this);

    return true;
}

inline bool CommandHandler::readCommandOutput()
{
    char buffer[128];

    const ssize_t bytes_read = read(pipefd_[0], buffer, sizeof(buffer) - 1);

    if (bytes_read > 0) {
        buffer[bytes_read] = '\0';
        std::lock_guard<std::mutex> lock(output_mtx_);
        output_ += buffer;

        return true;
    }

    return false;
}

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_COMMAND_HANDLER_HPP_
