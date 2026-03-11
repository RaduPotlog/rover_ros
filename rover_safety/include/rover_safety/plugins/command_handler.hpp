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
#include <sys/wait.h>

#include <atomic>
#include <chrono>
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

class CommandHandler
{

public:
    
    explicit CommandHandler() 
    {

    }

    ~CommandHandler()
    {
        killChildProcess();
        
        if (command_checker_thread_.joinable()) {
            command_checker_thread_.join();
        }
    };

    void execute(const std::string & command, const std::chrono::milliseconds & timeout);

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

    bool readCommandOutput();

  void killChildProcess();

    int pipefd_[2];
    pid_t m_child_pid_;
    std::chrono::milliseconds timeout_ms_;
    std::chrono::time_point<std::chrono::steady_clock> command_time_;

    std::atomic<CommandState> state_{CommandState::IDLE};
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
    timeout_ms_ = timeout_ms;
    state_ = CommandState::RUNNING;
    
    if (!executeCommandInChildProcess(command)) {
        state_ = CommandState::FAILURE;
    }
}

inline void CommandHandler::halt()
{
    killChildProcess();
    state_ = CommandState::FAILURE;
}

inline void CommandHandler::checkExecution()
{
    while (state_.load() == CommandState::RUNNING) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
        if (readCommandOutput()) {
            continue;
        }

        int status;
    
        if (waitpid(m_child_pid_, &status, WNOHANG) == m_child_pid_) {
            close(pipefd_[0]);  // Close read end after reading

            if (WEXITSTATUS(status) != 0) {
                std::lock_guard<std::mutex> lock(error_mtx_);
                error_ = "Command return code: " + std::to_string(WEXITSTATUS(status));
                state_ = CommandState::FAILURE;
            } else {
                state_ = CommandState::SUCCESS;
            }

            break;
        }

        if (timeoutExceeded(command_time_, timeout_ms_)) {
            killChildProcess();
            std::lock_guard<std::mutex> lock(error_mtx_);
            error_ = "Timeout exceeded";
            state_ = CommandState::FAILURE;
            break;
        }
    }
}

inline bool CommandHandler::executeCommandInChildProcess(const std::string & command)
{
    // Create a pipe
    if (pipe(pipefd_) == -1) {
        std::lock_guard<std::mutex> lock(error_mtx_);
        error_ = "Failed to create pipe";
       
        return false;
    }

    // Set the pipe to non-blocking mode
    int flags = fcntl(pipefd_[0], F_GETFL, 0);
    fcntl(pipefd_[0], F_SETFL, flags | O_NONBLOCK);

    // Create a child process that will execute the command
    m_child_pid_ = fork();
    command_time_ = std::chrono::steady_clock::now();

    if (m_child_pid_ == -1) {
        std::lock_guard<std::mutex> lock(error_mtx_);
        error_ = "Failed to fork";
    
        return false;
    }

    if (m_child_pid_ == 0) {
        close(pipefd_[0]);                // Close unused read end
        dup2(pipefd_[1], STDOUT_FILENO);  // Redirect stdout to pipe
        dup2(pipefd_[1], STDERR_FILENO);  // Redirect stderr to pipe
        close(pipefd_[1]);                // Close write end after redirecting

        execl("/bin/bash", "bash", "-c", command.c_str(), nullptr);
        exit(EXIT_FAILURE);
    }

    close(pipefd_[1]);  // Close unused write end

    command_checker_thread_ = std::thread(&CommandHandler::checkExecution, this);

    return true;
}

inline bool CommandHandler::readCommandOutput()
{
    char buffer[128];
    ssize_t bytes_read;

    bytes_read = read(pipefd_[0], buffer, sizeof(buffer) - 1);

    if ((bytes_read) > 0) {
        buffer[bytes_read] = '\0';
        std::lock_guard<std::mutex> lock(output_mtx_);
        output_ += buffer;

        return true;
    }

    return false;
}

inline void CommandHandler::killChildProcess()
{
    if (state_.load() != CommandState::RUNNING) {
        return;
    }
    
    close(pipefd_[0]);  // Close read end of the pipe
    kill(m_child_pid_, SIGKILL);
    
    int status;
    
    waitpid(m_child_pid_, &status, 0);
}

}  // namespace rover_safety

#endif  // ROVER_SAFETY_PLUGINS_ACTION_COMMAND_HANDLER_HPP_