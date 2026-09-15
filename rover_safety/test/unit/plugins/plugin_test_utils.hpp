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


#ifndef ROVER_SAFETY_TEST_UNIT_PLUGINS_PLUGIN_TEST_UTILS_HPP_
#define ROVER_SAFETY_TEST_UNIT_PLUGINS_PLUGIN_TEST_UTILS_HPP_

#include <arpa/inet.h>
#include <netinet/in.h>
#include <poll.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>

#include "behaviortree_cpp/bt_factory.h"

namespace rover_safety::test
{

/** XML of a tree holding one `node` with the given ports. */
inline std::string singleNodeTree(
    const std::string & node, const std::map<std::string, std::string> & ports)
{
    std::stringstream xml;
    xml << R"(<root BTCPP_format="4"><BehaviorTree ID="Test"><)" << node;
    for (const auto & [key, value] : ports) {
        xml << " " << key << "=\"" << value << "\"";
    }
    xml << "/></BehaviorTree></root>";
    return xml.str();
}

/** Ticks until the tree leaves RUNNING or `deadline` passes. */
inline BT::NodeStatus tickUntilDone(
    BT::Tree & tree, std::chrono::milliseconds deadline = std::chrono::seconds(20))
{
    const auto start = std::chrono::steady_clock::now();
    auto status = tree.tickOnce();
    while (status == BT::NodeStatus::RUNNING && std::chrono::steady_clock::now() - start < deadline) {
        tree.sleep(std::chrono::milliseconds(10));
        status = tree.tickOnce();
    }
    return status;
}

/**
 * Minimal HTTP server on 127.0.0.1 answering one request with `status_line` (e.g. "200 OK").
 * Stands in for the shutdown server of a remote host.
 */
class OneShotHttpServer
{
public:
    explicit OneShotHttpServer(std::string status_line)
    : status_line_(std::move(status_line))
    {
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0) {
            throw std::runtime_error("socket() failed");
        }
        const int enable = 1;
        setsockopt(fd_, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(enable));

        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        address.sin_port = 0;
        if (bind(fd_, reinterpret_cast<sockaddr *>(&address), sizeof(address)) != 0 || listen(fd_, 1) != 0) {
            close(fd_);
            throw std::runtime_error("bind()/listen() failed");
        }
        socklen_t length = sizeof(address);
        getsockname(fd_, reinterpret_cast<sockaddr *>(&address), &length);
        port_ = ntohs(address.sin_port);

        thread_ = std::thread([this]() { serve(); });
    }

    ~OneShotHttpServer()
    {
        stop_ = true;
        thread_.join();
        close(fd_);
    }

    std::string port() const { return std::to_string(port_); }

    std::string request()
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return request_;
    }

    /** A port on 127.0.0.1 nothing listens on. */
    static std::string closedPort()
    {
        const int fd = socket(AF_INET, SOCK_STREAM, 0);
        sockaddr_in address{};
        address.sin_family = AF_INET;
        address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
        bind(fd, reinterpret_cast<sockaddr *>(&address), sizeof(address));
        socklen_t length = sizeof(address);
        getsockname(fd, reinterpret_cast<sockaddr *>(&address), &length);
        close(fd);
        return std::to_string(ntohs(address.sin_port));
    }

private:
    void serve()
    {
        pollfd listener{fd_, POLLIN, 0};
        while (!stop_) {
            if (poll(&listener, 1, 50) <= 0) {
                continue;
            }
            const int client = accept(fd_, nullptr, nullptr);
            if (client < 0) {
                continue;
            }

            std::string received;
            char buffer[512];
            pollfd client_poll{client, POLLIN, 0};
            while (received.find("\r\n\r\n") == std::string::npos && poll(&client_poll, 1, 1000) > 0) {
                const auto n = read(client, buffer, sizeof(buffer));
                if (n <= 0) {
                    break;
                }
                received.append(buffer, static_cast<std::size_t>(n));
            }
            {
                std::lock_guard<std::mutex> lock(mutex_);
                request_ = received;
            }

            const std::string response =
                "HTTP/1.1 " + status_line_ + "\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
            [[maybe_unused]] const auto written = write(client, response.data(), response.size());
            close(client);
            return;
        }
    }

    std::string status_line_;
    int fd_{-1};
    unsigned port_{0};
    std::atomic<bool> stop_{false};
    std::mutex mutex_;
    std::string request_;
    std::thread thread_;
};

/** An address from TEST-NET-1 (RFC 5737): never routed, so ping fails. */
inline constexpr char kUnreachableIp[] = "192.0.2.1";

}  // namespace rover_safety::test

#endif  // ROVER_SAFETY_TEST_UNIT_PLUGINS_PLUGIN_TEST_UTILS_HPP_
