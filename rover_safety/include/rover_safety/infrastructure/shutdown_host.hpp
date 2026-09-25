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

#ifndef ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_HOST_HPP_
#define ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_HOST_HPP_

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>

#include "rover_safety/infrastructure/command_handler.hpp"

namespace rover_safety::infrastructure
{

enum class ShutdownHostState {
    IDLE = 0,
    COMMAND_EXECUTED,
    RESPONSE_RECEIVED,
    PINGING,
    SKIPPED,
    SUCCESS,
    FAILURE,
};

class ShutdownHostInterface
{

public:

    ShutdownHostInterface(const std::size_t hash) : hash_(hash) 
    {

    }
  
    virtual ~ShutdownHostInterface() = default;

    virtual void call() = 0;
    virtual void halt() = 0;
    virtual std::string getIp() const = 0;
    virtual std::string getError() const = 0;
    virtual std::string getOutput() const = 0;
    virtual ShutdownHostState getState() const = 0;

    bool operator==(const ShutdownHostInterface & other) const 
    {  
        return hash_ == other.hash_; 
    }

    bool operator!=(const ShutdownHostInterface & other) const 
    { 
        return hash_ != other.hash_; 
    }
  
    bool operator<(const ShutdownHostInterface & other) const 
    { 
        return hash_ < other.hash_; 
    }

private:
  
    const std::size_t hash_;
};

class ShutdownHost : public ShutdownHostInterface
{

public:

    ShutdownHost();

    ShutdownHost(
        const std::string ip,
        const std::string & port = "3003",
        const std::string secret = "root",
        const float timeout = 5.0);

    ~ShutdownHost() = default;

    void call() override;
    void halt() override;
    std::string getIp() const override;
    std::string getError() const override;
    std::string getOutput() const override;
    ShutdownHostState getState() const override;

protected:

    /**
     * Whether the host answers ping, without blocking the tick: the first call starts a ping in
     * the background and returns std::nullopt, as do calls while it runs. The call after it
     * finishes returns the result, and the one after that starts a new ping.
     */
    std::optional<bool> pollAvailability();

    std::int64_t getTimeSinceEpoch();

private:

    void requestShutdown();

    bool commandRunning();

    bool checkServerResponse();

    const std::string ip_;
    const std::string port_;
    const std::string secret_;
    const std::chrono::milliseconds timeout_ms_;
    ShutdownHostState state_;

    std::chrono::time_point<std::chrono::steady_clock> request_time_;

    std::string failure_reason_;

    std::shared_ptr<CommandHandler> command_handler_;

    // `ping -w 1` exits within a second; the margin only covers a slow fork.
    static constexpr std::chrono::milliseconds kPingTimeout{2000};
    std::shared_ptr<CommandHandler> ping_handler_;
    bool ping_started_ = false;
};

}  // namespace rover_safety::infrastructure

#endif  // ROVER_SAFETY_INFRASTRUCTURE_SHUTDOWN_HOST_HPP_
