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

#ifndef ROVER_IO_CONTEXT_APPLICATION_INBOUND_BYTE_BRIDGE_HPP_
#define ROVER_IO_CONTEXT_APPLICATION_INBOUND_BYTE_BRIDGE_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rover_io_context/domain/ports.hpp"

namespace rover::transport
{

// Bytes off the wire -> the publisher. This is the whole of the "receive" direction that
// the serial bridge and the UDP receiver had duplicated between them.
//
// Holds references, not ownership: the node owns both the stream and the publisher and
// outlives this object.
class InboundByteBridge
{

public:

    InboundByteBridge(ByteStreamPort & stream, BytePublisherPort & publisher);

    // Arms the stream's async receive. Call once the stream is open.
    void start();

    // Called from the stream's receive callback - on the ASIO thread, not the executor
    // thread. Kept public so tests can drive it without a real stream.
    void onBytes(const std::vector<uint8_t> & buffer, std::size_t length);

    std::uint64_t bytesForwarded() const;

private:

    ByteStreamPort & stream_;
    BytePublisherPort & publisher_;
    std::uint64_t bytes_forwarded_{0};
};

}  // namespace rover::transport

#endif  // ROVER_IO_CONTEXT_APPLICATION_INBOUND_BYTE_BRIDGE_HPP_
