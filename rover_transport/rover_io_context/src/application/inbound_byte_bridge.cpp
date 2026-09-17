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

#include "rover_io_context/application/inbound_byte_bridge.hpp"

namespace rover::transport
{

InboundByteBridge::InboundByteBridge(ByteStreamPort & stream, BytePublisherPort & publisher)
: stream_(stream),
  publisher_(publisher)
{
}

void InboundByteBridge::start()
{
    stream_.asyncReceive(
        [this](const std::vector<uint8_t> & buffer, std::size_t length)
        {
            onBytes(buffer, length);
        });
}

void InboundByteBridge::onBytes(const std::vector<uint8_t> & buffer, std::size_t length)
{
    if (length == 0) {
        return;
    }

    bytes_forwarded_ += length;
    publisher_.publish(buffer, length);
}

std::uint64_t InboundByteBridge::bytesForwarded() const
{
    return bytes_forwarded_;
}

}  // namespace rover::transport
