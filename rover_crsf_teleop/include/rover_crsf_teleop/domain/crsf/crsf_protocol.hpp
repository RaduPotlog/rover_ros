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
// The CRSF (TBS Crossfire) wire constants below are published by Team Black Sheep under the
// 2-Clause BSD License. They are restated here rather than copied from a GPL-licensed
// flight-controller header, so that this Apache-2.0 package carries no GPL code.

#ifndef ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PROTOCOL_HPP_
#define ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PROTOCOL_HPP_

#include <cstddef>
#include <cstdint>

namespace rover_crsf_teleop::crsf
{

// A CRSF frame on the wire is:
//
//   [0] sync / destination address
//   [1] length  = 1 (type) + payload + 1 (crc)
//   [2] type
//   [3 .. ] payload
//   [2 + length - 1] CRC8 over bytes [2 .. 2 + length - 2]
//
// Note that `length` counts the type and the CRC but NOT the two header bytes, so a complete
// frame occupies `length + 2` bytes.

// Frames from the receiver to us are addressed to the flight controller; this doubles as the
// frame sync byte. Frames addressed elsewhere (e.g. 0xEA, the handset) are valid CRSF and must
// be consumed and ignored rather than treated as corruption.
constexpr std::uint8_t kSyncByte = 0xC8;
constexpr std::uint8_t kAddressFlightController = 0xC8;

constexpr std::uint8_t kFrameTypeLinkStatistics = 0x14;
constexpr std::uint8_t kFrameTypeRcChannelsPacked = 0x16;

// CRC8/DVB-S2: polynomial 0xD5, initial value 0, no reflection, no final xor.
constexpr std::uint8_t kCrcPolynomial = 0xD5;

// Length-byte bounds. The minimum of 3 is type + one payload byte + CRC; below that a frame
// cannot carry anything. The maximum comes from the CRSF maximum payload of 60 bytes.
constexpr std::uint8_t kMaxPayloadLength = 60;
constexpr std::uint8_t kMinLengthByte = 3;
constexpr std::uint8_t kMaxLengthByte = kMaxPayloadLength + 2;

// Largest complete frame, header included: used to size and bound the receive buffer.
constexpr std::size_t kMaxFrameSize = static_cast<std::size_t>(kMaxLengthByte) + 2U;

// RC_CHANNELS_PACKED carries 16 channels of 11 bits each, LSB-first: 176 bits = 22 bytes.
constexpr std::size_t kRcChannelCount = 16;
constexpr std::size_t kRcChannelBits = 11;
constexpr std::size_t kRcChannelsPayloadSize = 22;
constexpr std::uint16_t kRcChannelMask = 0x07FF;

// LINK_STATISTICS carries ten single-byte fields.
constexpr std::size_t kLinkStatisticsPayloadSize = 10;

// Nominal stick endpoints, restated from the CRSF specification. These are defaults for the
// channel_in_* parameters, not hard limits - a transmitter's endpoints and trim move them.
constexpr int kChannelValueMin = 172;
constexpr int kChannelValueMid = 992;
constexpr int kChannelValueMax = 1811;

}  // namespace rover_crsf_teleop::crsf

#endif  // ROVER_CRSF_TELEOP_DOMAIN_CRSF_CRSF_PROTOCOL_HPP_
