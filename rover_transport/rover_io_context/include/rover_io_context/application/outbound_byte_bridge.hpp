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

#ifndef ROVER_IO_CONTEXT_APPLICATION_OUTBOUND_BYTE_BRIDGE_HPP_
#define ROVER_IO_CONTEXT_APPLICATION_OUTBOUND_BYTE_BRIDGE_HPP_

#include <cstdint>
#include <vector>

#include "rover_io_context/domain/ports.hpp"

namespace rover::transport
{

// A publisher-side gate: bytes are written to the stream only while the owning node is
// ACTIVE. Upstream checked `get_current_state().id() == PRIMARY_STATE_ACTIVE` inline in
// each subscriber callback, which meant the single most safety-relevant rule in these
// drivers - "do not drive the hardware from a deactivated node" - was untestable without
// a full ROS graph. Here it is a plain flag the node sets from its lifecycle callbacks.
class OutboundByteBridge
{

public:

    explicit OutboundByteBridge(ByteStreamPort & stream);

    // Driven by on_activate / on_deactivate.
    void setActive(bool active);

    bool isActive() const;

    // Writes to the stream when active; silently drops otherwise. Dropping is correct:
    // a deactivated bridge has no business putting bytes on a UART or a socket, and the
    // sender has no channel to report back on.
    void send(const std::vector<uint8_t> & buffer);

    std::uint64_t bytesSent() const;

    std::uint64_t bytesDropped() const;

private:

    ByteStreamPort & stream_;
    bool active_{false};
    std::uint64_t bytes_sent_{0};
    std::uint64_t bytes_dropped_{0};
};

}  // namespace rover::transport

#endif  // ROVER_IO_CONTEXT_APPLICATION_OUTBOUND_BYTE_BRIDGE_HPP_
