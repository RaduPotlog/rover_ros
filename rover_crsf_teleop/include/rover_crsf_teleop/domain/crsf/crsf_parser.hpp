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

#ifndef ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PARSER_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PARSER_HPP_

#include <cstddef>
#include <cstdint>
#include <vector>

#include "rover_crsf_teleop/domain/crsf/crc8.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_protocol.hpp"
#include "rover_crsf_teleop/domain/rc_frame.hpp"

namespace rover_crsf_teleop::crsf
{

// What a CrsfParser hands back as it decodes. Implemented by the node, which forwards to the
// teleop use case and to the echo publishers.
//
// Called synchronously from within parse(), once per decoded frame, on the calling thread.
class CrsfSink
{

public:

    virtual ~CrsfSink() = default;

    virtual void onRcChannels(const RcFrame & frame) = 0;

    virtual void onLinkStatistics(const RcLinkStats & stats) = 0;
};

// Byte-stream CRSF decoder.
//
// The input is a raw serial stream arriving in arbitrary chunks - the transport decides where
// the boundaries fall, and a frame is routinely split across two of them - so the parser keeps
// its own buffer across calls and resynchronises on the sync byte whenever the stream is not
// where it expects to be.
//
// It deliberately owns NO clock. Whether the link is fresh enough to drive the rover is
// LinkMonitor's decision, made on a steady clock with an injected time so it can be tested; a
// second, independent notion of freshness in here would only be able to disagree with it.
//
// Not thread-safe: feed it from one thread. The node parses in its subscription callback, on the
// executor thread, which is the same thread the control timer runs on.
class CrsfParser
{

public:

    CrsfParser();

    // Appends `length` bytes and decodes as many complete frames as they complete, calling
    // `sink` once per frame. A null `data` or zero `length` still drains whatever is buffered.
    void parse(const std::uint8_t * data, std::size_t length, CrsfSink & sink);

    // Drops any partial frame. Call when the byte stream may have been interrupted, e.g. after
    // the serial link has been down, so a stale half-frame cannot join onto fresh bytes.
    void reset();

    // Bytes currently held pending completion. Exposed for tests and diagnostics.
    std::size_t bufferedBytes() const { return buffer_.size(); }

private:

    // Decodes whatever complete frames the buffer holds.
    void drain(CrsfSink & sink);

    // Handles one CRC-verified frame starting at buffer_[0].
    void dispatchFrame(std::uint8_t length_byte, CrsfSink & sink);

    // Drops the leading byte and everything up to the next sync byte. Clears the buffer when
    // there is no further sync byte to resynchronise onto.
    void resync();

    // A stream of sync bytes with length bytes that never complete would otherwise grow the
    // buffer without bound. Nothing legitimate needs more than two frames of slack.
    static constexpr std::size_t kMaxBufferedBytes = 2U * kMaxFrameSize;

    Crc8 crc_;
    std::vector<std::uint8_t> buffer_;
};

// Unpacks the 16 x 11-bit little-endian channel field of an RC_CHANNELS_PACKED payload.
//
// Exposed (rather than kept private) because it is the single most error-prone piece of this
// package and deserves to be tested directly against known-good byte vectors.
//
// Reads no further than `length`; channels whose bits fall outside the payload read as 0.
RcFrame unpackRcChannels(const std::uint8_t * payload, std::size_t length);

// Decodes a LINK_STATISTICS payload. Fields beyond `length` are left at their defaults.
RcLinkStats unpackLinkStatistics(const std::uint8_t * payload, std::size_t length);

}  // namespace rover_crsf_teleop::crsf

#endif  // ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PARSER_HPP_
