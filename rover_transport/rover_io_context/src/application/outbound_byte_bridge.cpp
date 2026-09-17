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

#include "rover_io_context/application/outbound_byte_bridge.hpp"

namespace rover::transport
{

OutboundByteBridge::OutboundByteBridge(ByteStreamPort & stream)
: stream_(stream)
{
}

void OutboundByteBridge::setActive(bool active)
{
    active_ = active;
}

bool OutboundByteBridge::isActive() const
{
    return active_;
}

void OutboundByteBridge::send(const std::vector<uint8_t> & buffer)
{
    if (!active_) {
        bytes_dropped_ += buffer.size();
        return;
    }

    bytes_sent_ += buffer.size();
    stream_.asyncSend(buffer);
}

std::uint64_t OutboundByteBridge::bytesSent() const
{
    return bytes_sent_;
}

std::uint64_t OutboundByteBridge::bytesDropped() const
{
    return bytes_dropped_;
}

}  // namespace rover::transport
