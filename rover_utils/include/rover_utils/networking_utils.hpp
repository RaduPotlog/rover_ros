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

#ifndef ROVER_UTILS_ROVER_UTILS_NETWORKING_UTILS_HPP_
#define ROVER_UTILS_ROVER_UTILS_NETWORKING_UTILS_HPP_

#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>

namespace rover_utils
{

inline bool isPortAvailable(int port)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    
    if (sock == -1) {
        return false;
    }

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(port);

    bool available = bind(sock, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) == 0;

    close(sock);

    return available;
}

}  // namespace rover_utils

#endif  // ROVER_UTILS_ROVER_UTILS_NETWORKING_UTILS_HPP_
