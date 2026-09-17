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
//
// Derived from crsf_receiver (MIT, Andrey Tulyakov) - https://github.com/AndreyTulyakov/ros2_crsf_receiver

#include "rover_crsf_teleop/domain/crsf/crsf_parser.hpp"

#include <algorithm>

#include "rover_crsf_teleop/domain/crsf/crsf_protocol.hpp"

namespace rover_crsf_teleop::crsf
{

namespace
{

// Reads up to three bytes starting at `offset`, little-endian, treating anything past the end of
// the payload as zero. Three bytes is enough to contain any 11-bit field at any bit offset.
std::uint32_t readWord(const std::uint8_t * const payload, const std::size_t length, const std::size_t offset)
{
    std::uint32_t word = 0;

    for (std::size_t i = 0; i < 3U; ++i) {
        if (offset + i < length) {
            word |= static_cast<std::uint32_t>(payload[offset + i]) << (8U * i);
        }
    }

    return word;
}

}  // namespace

RcFrame unpackRcChannels(const std::uint8_t * const payload, const std::size_t length)
{
    RcFrame frame;

    if (payload == nullptr) {
        return frame;
    }

    // The 16 channels are a single little-endian bit stream, 11 bits each, LSB first - not byte
    // aligned. Reading it with shifts rather than a packed bitfield struct matters: bitfield
    // allocation order is implementation-defined, and reinterpret_cast'ing a byte pointer onto
    // such a struct is a strict-aliasing violation. This is verified bit-identical to the
    // original bitfield decode in test_crsf_parser.cpp.
    std::size_t bit = 0;

    for (std::size_t channel = 0; channel < kRcChannelCount; ++channel) {
        const std::size_t byte_offset = bit / 8U;
        const unsigned int bit_offset = static_cast<unsigned int>(bit % 8U);

        const std::uint32_t word = readWord(payload, length, byte_offset);
        frame.channels[channel] = static_cast<int>((word >> bit_offset) & kRcChannelMask);

        bit += kRcChannelBits;
    }

    return frame;
}

RcLinkStats unpackLinkStatistics(const std::uint8_t * const payload, const std::size_t length)
{
    RcLinkStats stats;

    if (payload == nullptr) {
        return stats;
    }

    const auto byte_at = [payload, length](const std::size_t index) -> std::uint8_t {
        return index < length ? payload[index] : static_cast<std::uint8_t>(0);
    };

    stats.uplink_rssi_ant1 = byte_at(0);
    stats.uplink_rssi_ant2 = byte_at(1);
    stats.uplink_link_quality = byte_at(2);
    stats.uplink_snr = static_cast<std::int8_t>(byte_at(3));
    stats.active_antenna = byte_at(4);
    stats.rf_mode = byte_at(5);
    stats.uplink_tx_power = byte_at(6);
    stats.downlink_rssi = byte_at(7);
    stats.downlink_link_quality = byte_at(8);
    stats.downlink_snr = static_cast<std::int8_t>(byte_at(9));

    return stats;
}

CrsfParser::CrsfParser() : crc_(kCrcPolynomial)
{
    buffer_.reserve(kMaxFrameSize);
}

void CrsfParser::reset()
{
    buffer_.clear();
}

void CrsfParser::parse(const std::uint8_t * const data, const std::size_t length, CrsfSink & sink)
{
    if (data != nullptr && length > 0U) {
        // Bound the buffer before appending, not after: a peer that never completes a frame must
        // not be able to make us allocate without limit. Dropping the oldest bytes is right -
        // they are the stale partial frame, and the newest bytes are the ones worth keeping.
        if (buffer_.size() + length > kMaxBufferedBytes) {
            const std::size_t keep = kMaxBufferedBytes > length ? kMaxBufferedBytes - length : 0U;

            if (keep < buffer_.size()) {
                buffer_.erase(buffer_.begin(), buffer_.end() - static_cast<std::ptrdiff_t>(keep));
            }
        }

        buffer_.insert(buffer_.end(), data, data + length);
    }

    drain(sink);
}

void CrsfParser::drain(CrsfSink & sink)
{
    // Each pass either consumes a frame, resynchronises, or decides it needs more bytes - so the
    // loop always terminates.
    while (buffer_.size() > 1U) {
        if (buffer_[0] != kSyncByte) {
            resync();
            continue;
        }

        const std::uint8_t length_byte = buffer_[1];

        if (length_byte < kMinLengthByte || length_byte > kMaxLengthByte) {
            resync();
            continue;
        }

        // `length_byte` counts the type and the CRC but not the two header bytes.
        const std::size_t frame_size = static_cast<std::size_t>(length_byte) + 2U;

        if (buffer_.size() < frame_size) {
            return;  // Incomplete: wait for the rest of the frame.
        }

        const std::uint8_t received_crc = buffer_[frame_size - 1U];
        const std::uint8_t computed_crc = crc_.calc(buffer_.data() + 2, static_cast<std::size_t>(length_byte) - 1U);

        if (computed_crc == received_crc) {
            dispatchFrame(length_byte, sink);

            // Consume the whole frame. A corrupt frame is consumed too (below), rather than
            // resynchronised into, so a payload byte that happens to be 0xC8 cannot be mistaken
            // for the start of the next frame.
            buffer_.erase(buffer_.begin(), buffer_.begin() + static_cast<std::ptrdiff_t>(frame_size));
        } else {
            // Bad CRC means the length byte cannot be trusted either, so the frame boundary is
            // unknown: resynchronise rather than skipping a length we may have misread.
            resync();
        }
    }
}

void CrsfParser::dispatchFrame(const std::uint8_t length_byte, CrsfSink & sink)
{
    // Frames addressed to anything other than the flight controller are valid CRSF traffic that
    // simply is not for us - consume them silently.
    if (buffer_[0] != kAddressFlightController) {
        return;
    }

    const std::uint8_t type = buffer_[2];

    // Payload spans buffer_[3 .. frame_size - 2]: length_byte covers type + payload + crc.
    const std::size_t payload_size = static_cast<std::size_t>(length_byte) - 2U;
    const std::uint8_t * const payload = buffer_.data() + 3;

    switch (type) {
        case kFrameTypeRcChannelsPacked:
            // A short RC frame would decode to zeros, which the stick mapping reads as full
            // negative deflection - so require the full 22 bytes rather than trusting it.
            if (payload_size >= kRcChannelsPayloadSize) {
                sink.onRcChannels(unpackRcChannels(payload, payload_size));
            }
            break;

        case kFrameTypeLinkStatistics:
            if (payload_size >= kLinkStatisticsPayloadSize) {
                sink.onLinkStatistics(unpackLinkStatistics(payload, payload_size));
            }
            break;

        default:
            break;
    }
}

void CrsfParser::resync()
{
    // Skip the current (bad) leading byte, then look for the next sync byte to start from.
    const auto next = std::find(buffer_.begin() + 1, buffer_.end(), kSyncByte);

    buffer_.erase(buffer_.begin(), next);
}

}  // namespace rover_crsf_teleop::crsf
