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

#include "rover_safety/infrastructure/shutdown_host.hpp"

#include <chrono>
#include <cstdint>
#include <functional>
#include <iomanip>
#include <ios>
#include <memory>
#include <optional>
#include <sstream>
#include <stdexcept>
#include <string>

#include <openssl/hmac.h>

#include "rover_safety/behavior_tree_utils.hpp"

namespace rover_safety::infrastructure
{

ShutdownHost::ShutdownHost()
: ShutdownHostInterface(std::hash<std::string>{}(""))
, ip_("")
, port_("3003")
, secret_("")
, timeout_ms_(5000)
, state_(ShutdownHostState::IDLE)
{
    command_handler_ = std::make_shared<CommandHandler>();
    ping_handler_ = std::make_shared<CommandHandler>();
}

ShutdownHost::ShutdownHost(
    const std::string ip,
    const std::string & port,
    const std::string secret,
    const float timeout)
: ShutdownHostInterface(std::hash<std::string>{}(ip + port))
, ip_(ip)
, port_(port)
, secret_(secret)
, timeout_ms_(static_cast<long long>(timeout * 1000))
, state_(ShutdownHostState::IDLE)
{
    command_handler_ = std::make_shared<CommandHandler>();
    ping_handler_ = std::make_shared<CommandHandler>();
}

void ShutdownHost::call()
{
    switch (state_) {
        case ShutdownHostState::IDLE: {
            const auto available = pollAvailability();

            if (!available) {
                break;
            }

            if (!*available) {
                state_ = ShutdownHostState::SKIPPED;
                break;
            }

            try {
                requestShutdown();
            } catch (const std::runtime_error & err) {
                state_ = ShutdownHostState::FAILURE;
                failure_reason_ = err.what();
                break;
            }
            
            state_ = ShutdownHostState::COMMAND_EXECUTED;
            break;
        }

        case ShutdownHostState::COMMAND_EXECUTED:
            if (commandRunning()) {
                break;
            }

            if (command_handler_->getState() == CommandState::FAILURE) {
                state_ = ShutdownHostState::FAILURE;
                failure_reason_ = command_handler_->getError();
                break;
            }

            state_ = ShutdownHostState::RESPONSE_RECEIVED;
            break;

        case ShutdownHostState::RESPONSE_RECEIVED:
            if (checkServerResponse()) {
                state_ = ShutdownHostState::PINGING;
                break;
            }
    
            state_ = ShutdownHostState::FAILURE;
            break;

        case ShutdownHostState::PINGING: {
            const auto available = pollAvailability();

            if (available && !*available) {
                state_ = ShutdownHostState::SUCCESS;
                break;
            }

            if (timeoutExceeded(request_time_, timeout_ms_)) {
                state_ = ShutdownHostState::FAILURE;
                failure_reason_ = "Timeout waiting for host to shutdown";
            }

            break;
        }

        default:
            break;
    }
}

void ShutdownHost::halt()
{ 
    command_handler_->halt(); 
    ping_handler_->halt();
    ping_started_ = false;
}

std::string ShutdownHost::getIp() const
{ 
    return ip_; 
}

std::string ShutdownHost::getError() const
{ 
    return failure_reason_; 
}

std::string ShutdownHost::getOutput() const
{ 
    return command_handler_->getOutput(); 
}

ShutdownHostState ShutdownHost::getState() const
{ 
    return state_; 
}

std::optional<bool> ShutdownHost::pollAvailability()
{
    if (!ping_started_) {
        ping_handler_->execute("ping -c 1 -w 1 " + ip_ + " > /dev/null", kPingTimeout);
        ping_started_ = true;
    }

    const auto state = ping_handler_->getState();

    if (state == CommandState::RUNNING) {
        return std::nullopt;
    }

    ping_started_ = false;

    return state == CommandState::SUCCESS;
}

std::int64_t ShutdownHost::getTimeSinceEpoch()
{
    return std::chrono::duration_cast<std::chrono::seconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}

void ShutdownHost::requestShutdown()
{
    request_time_ = std::chrono::steady_clock::now();
    const auto time_now_str = std::to_string(getTimeSinceEpoch());
    std::string string_to_sign = "/shutdown|" + time_now_str;

    unsigned char * hmac_result;
    unsigned int len = 32;
    hmac_result = HMAC(
    EVP_sha256(), secret_.c_str(), secret_.length(),
    reinterpret_cast<const unsigned char *>(string_to_sign.c_str()), string_to_sign.length(), NULL, NULL);

    std::stringstream ss;
    
    for (unsigned int i = 0; i < len; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(hmac_result[i]);
    }
    
    std::string sig = ss.str();

    const std::string command = "curl -s -w '%{errormsg}\\n%{http_code}' 'http://" + ip_ + ":" + 
                                port_ + "/shutdown?ts=" + time_now_str + "&sig=" + sig + "'";

    command_handler_->execute(command, timeout_ms_);
    
    if (command_handler_->getState() == CommandState::FAILURE) {
        throw std::runtime_error("Failed to execute command");
    }
}

bool ShutdownHost::commandRunning()
{
    return command_handler_->getState() == CommandState::RUNNING ? true : false;
}

bool ShutdownHost::checkServerResponse()
{
    const auto output = this->command_handler_->getOutput();
    // Output may have multiple lines. We are interested in the last one.
    const auto http_return_code = output.substr(output.find_last_of('\n') + 1);

    if (http_return_code != "200") {
        failure_reason_ = "Failed to shutdown remote host. Server return code: " + http_return_code;
        return false;
    }

    return true;
}

}  // namespace rover_safety::infrastructure
