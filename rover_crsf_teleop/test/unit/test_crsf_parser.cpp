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

// Unit tests for the CRSF byte-stream decoder.
//
// The GOLDEN_* frames below were generated from the packed-bitfield decoder this parser replaces
// (crsf_receiver's `struct CrsfChannels { unsigned chN : 11; }` + reinterpret_cast), so they pin
// the shift-based rewrite to exactly the behaviour that was running on the rover before it.

#include <gtest/gtest.h>

#include <array>
#include <cstdint>
#include <random>
#include <vector>

#include "rover_crsf_teleop/domain/crsf/crc8.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_parser.hpp"
#include "rover_crsf_teleop/domain/crsf/crsf_protocol.hpp"

namespace rover_crsf_teleop::crsf
{
namespace
{

using Bytes = std::vector<std::uint8_t>;

// Collects everything the parser emits, so a test can assert on counts as well as values.
class RecordingSink : public CrsfSink
{

public:

    void onRcChannels(const RcFrame & frame) override { frames.push_back(frame); }

    void onLinkStatistics(const RcLinkStats & stats) override { link_stats.push_back(stats); }

    std::vector<RcFrame> frames;
    std::vector<RcLinkStats> link_stats;
};

// --- Golden frames, generated from the original bitfield decoder ----------------------------

const Bytes kAllCentre{0xC8, 0x18, 0x16, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0,
                       0x81, 0x0F, 0x7C, 0xE0, 0x03, 0x1F, 0xF8, 0xC0, 0x07, 0x3E, 0xF0,
                       0x81, 0x0F, 0x7C, 0xAD};

const Bytes kAllMin{0xC8, 0x18, 0x16, 0xAC, 0x60, 0x05, 0x2B, 0x58, 0xC1, 0x0A, 0x56,
                    0xB0, 0x82, 0x15, 0xAC, 0x60, 0x05, 0x2B, 0x58, 0xC1, 0x0A, 0x56,
                    0xB0, 0x82, 0x15, 0x5B};

const Bytes kAllMax{0xC8, 0x18, 0x16, 0x13, 0x9F, 0xF8, 0xC4, 0x27, 0x3E, 0xF1, 0x89,
                    0x4F, 0x7C, 0xE2, 0x13, 0x9F, 0xF8, 0xC4, 0x27, 0x3E, 0xF1, 0x89,
                    0x4F, 0x7C, 0xE2, 0xB9};

// Every channel a different value: this is the vector that catches an off-by-one anywhere in the
// 11-bit shift chain, which uniform vectors cannot.
const Bytes kDistinct{0xC8, 0x18, 0x16, 0x64, 0xC8, 0x86, 0x53, 0x86, 0x83, 0xA3, 0x56,
                      0x89, 0xEC, 0x72, 0x0C, 0x0C, 0xA4, 0x3D, 0xD7, 0x0A, 0xDE, 0x2A,
                      0x2B, 0xFB, 0xE7, 0x74};

const std::array<int, 16> kDistinctValues{100,  217,  334,  451,  568,  685,  802,  919,
                                          1036, 1153, 1270, 1387, 1504, 1621, 1738, 1855};

// Exercises both extremes of the 11-bit range in one frame.
const Bytes kAlternating{0xC8, 0x18, 0x16, 0x00, 0xF8, 0x3F, 0x00, 0xFE, 0x0F, 0x80, 0xFF,
                         0x03, 0xE0, 0xFF, 0x00, 0xF8, 0x3F, 0x00, 0xFE, 0x0F, 0x80, 0xFF,
                         0x03, 0xE0, 0xFF, 0x97};

// uplink_snr = -7, downlink_snr = -12: the signed fields are the regression case for the old
// message definition, which typed uplink_snr as unsigned.
const Bytes kLinkStats{0xC8, 0x0C, 0x14, 0x2D, 0x32, 0x64, 0xF9, 0x01, 0x02, 0x03, 0x3C, 0x63, 0xF4, 0xCD};

Bytes withGarbagePrefix(const Bytes & frame, const Bytes & prefix)
{
    Bytes out = prefix;
    out.insert(out.end(), frame.begin(), frame.end());
    return out;
}

// --- Channel unpacking ----------------------------------------------------------------------

TEST(CrsfParserTest, DecodesAllChannelsAtCentre)
{
    RecordingSink sink;
    CrsfParser parser;
    parser.parse(kAllCentre.data(), kAllCentre.size(), sink);

    ASSERT_EQ(sink.frames.size(), 1U);

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        EXPECT_EQ(sink.frames[0].channels[i], kChannelValueMid) << "channel " << i;
    }
}

TEST(CrsfParserTest, DecodesEndpoints)
{
    for (const auto & [frame_bytes, expected] :
         {std::pair{kAllMin, kChannelValueMin}, std::pair{kAllMax, kChannelValueMax}}) {
        RecordingSink sink;
        CrsfParser parser;
        parser.parse(frame_bytes.data(), frame_bytes.size(), sink);

        ASSERT_EQ(sink.frames.size(), 1U);

        for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
            EXPECT_EQ(sink.frames[0].channels[i], expected) << "channel " << i;
        }
    }
}

TEST(CrsfParserTest, DecodesDistinctValuePerChannel)
{
    RecordingSink sink;
    CrsfParser parser;
    parser.parse(kDistinct.data(), kDistinct.size(), sink);

    ASSERT_EQ(sink.frames.size(), 1U);

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        EXPECT_EQ(sink.frames[0].channels[i], kDistinctValues[i]) << "channel " << i;
    }
}

TEST(CrsfParserTest, DecodesFullElevenBitRange)
{
    RecordingSink sink;
    CrsfParser parser;
    parser.parse(kAlternating.data(), kAlternating.size(), sink);

    ASSERT_EQ(sink.frames.size(), 1U);

    for (std::size_t i = 0; i < RcFrame::kChannelCount; ++i) {
        EXPECT_EQ(sink.frames[0].channels[i], (i % 2U == 0U) ? 0 : 2047) << "channel " << i;
    }
}

TEST(CrsfParserTest, UnpackNeverReadsPastTheDeclaredPayloadLength)
{
    // 22 bytes is the exact payload size; a shorter buffer must still be safe (the trailing
    // channels simply read as 0) rather than reading past the end.
    const std::uint8_t * const payload = kDistinct.data() + 3;

    const RcFrame full = unpackRcChannels(payload, kRcChannelsPayloadSize);
    EXPECT_EQ(full.channels[15], kDistinctValues[15]);

    const RcFrame truncated = unpackRcChannels(payload, 4U);
    EXPECT_EQ(truncated.channels[0], kDistinctValues[0]);
    EXPECT_EQ(truncated.channels[15], 0);
}

// --- Link statistics ------------------------------------------------------------------------

TEST(CrsfParserTest, DecodesLinkStatisticsIncludingNegativeSnr)
{
    RecordingSink sink;
    CrsfParser parser;
    parser.parse(kLinkStats.data(), kLinkStats.size(), sink);

    ASSERT_EQ(sink.link_stats.size(), 1U);
    const RcLinkStats & stats = sink.link_stats[0];

    EXPECT_EQ(stats.uplink_rssi_ant1, 45);
    EXPECT_EQ(stats.uplink_rssi_ant2, 50);
    EXPECT_EQ(stats.uplink_link_quality, 100);
    EXPECT_EQ(stats.uplink_snr, -7);
    EXPECT_EQ(stats.active_antenna, 1);
    EXPECT_EQ(stats.rf_mode, 2);
    EXPECT_EQ(stats.uplink_tx_power, 3);
    EXPECT_EQ(stats.downlink_rssi, 60);
    EXPECT_EQ(stats.downlink_link_quality, 99);
    EXPECT_EQ(stats.downlink_snr, -12);
}

// --- Framing, resynchronisation and robustness ----------------------------------------------

TEST(CrsfParserTest, DeliveredOneByteAtATimeYieldsExactlyOneFrame)
{
    // Chunk boundaries are decided by the transport, not by frame boundaries, so a frame split
    // across reads is the normal case rather than an edge case.
    RecordingSink sink;
    CrsfParser parser;

    for (const std::uint8_t byte : kDistinct) {
        parser.parse(&byte, 1U, sink);
    }

    ASSERT_EQ(sink.frames.size(), 1U);
    EXPECT_EQ(sink.frames[0].channels[3], kDistinctValues[3]);
}

TEST(CrsfParserTest, SplitAcrossTwoChunksYieldsExactlyOneFrame)
{
    RecordingSink sink;
    CrsfParser parser;

    parser.parse(kAllCentre.data(), 7U, sink);
    EXPECT_TRUE(sink.frames.empty());

    parser.parse(kAllCentre.data() + 7, kAllCentre.size() - 7U, sink);
    EXPECT_EQ(sink.frames.size(), 1U);
}

TEST(CrsfParserTest, ResynchronisesPastGarbage)
{
    const Bytes stream = withGarbagePrefix(kAllCentre, {0x01, 0x02, 0x03, 0xFF});

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    EXPECT_EQ(sink.frames.size(), 1U);
}

TEST(CrsfParserTest, ResynchronisesPastAFalseSyncByteInGarbage)
{
    // A stray 0xC8 followed by an implausible length must not swallow the real frame behind it.
    const Bytes stream = withGarbagePrefix(kAllCentre, {0xC8, 0x01, 0x77});

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    EXPECT_EQ(sink.frames.size(), 1U);
}

TEST(CrsfParserTest, TwoFramesInOneChunk)
{
    Bytes stream = kAllCentre;
    stream.insert(stream.end(), kLinkStats.begin(), kLinkStats.end());

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    EXPECT_EQ(sink.frames.size(), 1U);
    EXPECT_EQ(sink.link_stats.size(), 1U);
}

TEST(CrsfParserTest, PayloadByteEqualToSyncByteDoesNotSplitTheFrame)
{
    // kAlternating's payload contains 0x00/0xFF runs; build a frame whose payload deliberately
    // contains 0xC8 and confirm the parser consumes the frame whole.
    Bytes frame{0xC8, 0x18, 0x16};
    Bytes payload(kRcChannelsPayloadSize, 0xC8);
    frame.insert(frame.end(), payload.begin(), payload.end());

    const Crc8 crc(kCrcPolynomial);
    frame.push_back(crc.calc(frame.data() + 2, 23U));

    Bytes stream = frame;
    stream.insert(stream.end(), kAllCentre.begin(), kAllCentre.end());

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    // Both frames decode: the 0xC8-filled one, then the centre frame after it.
    ASSERT_EQ(sink.frames.size(), 2U);
    EXPECT_EQ(sink.frames[1].channels[0], kChannelValueMid);
}

TEST(CrsfParserTest, BadCrcIsDroppedAndTheNextGoodFrameStillDecodes)
{
    Bytes corrupted = kAllCentre;
    corrupted.back() ^= 0xFFU;

    Bytes stream = corrupted;
    stream.insert(stream.end(), kDistinct.begin(), kDistinct.end());

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    ASSERT_EQ(sink.frames.size(), 1U);
    EXPECT_EQ(sink.frames[0].channels[0], kDistinctValues[0]);
}

TEST(CrsfParserTest, RejectsOutOfRangeLengthBytes)
{
    for (const std::uint8_t bad_length : {std::uint8_t{0}, std::uint8_t{1}, std::uint8_t{2},
                                          static_cast<std::uint8_t>(kMaxLengthByte + 1)}) {
        Bytes stream{kSyncByte, bad_length, 0x16, 0x00, 0x00};
        stream.insert(stream.end(), kAllCentre.begin(), kAllCentre.end());

        RecordingSink sink;
        CrsfParser parser;
        parser.parse(stream.data(), stream.size(), sink);

        EXPECT_EQ(sink.frames.size(), 1U) << "length byte " << static_cast<int>(bad_length);
    }
}

TEST(CrsfParserTest, IgnoresFramesAddressedElsewhere)
{
    // 0xEA is the handset. Valid CRSF, correct CRC, simply not for us - and it must be consumed
    // rather than resynchronised into, so the frame behind it still decodes.
    Bytes other = kAllCentre;
    other[0] = 0xEA;

    Bytes stream = other;
    stream.insert(stream.end(), kAllCentre.begin(), kAllCentre.end());

    RecordingSink sink;
    CrsfParser parser;
    parser.parse(stream.data(), stream.size(), sink);

    EXPECT_EQ(sink.frames.size(), 1U);
}

TEST(CrsfParserTest, ResetDropsAPartialFrame)
{
    RecordingSink sink;
    CrsfParser parser;

    parser.parse(kAllCentre.data(), 7U, sink);
    EXPECT_GT(parser.bufferedBytes(), 0U);

    parser.reset();
    EXPECT_EQ(parser.bufferedBytes(), 0U);

    // The remaining bytes of the abandoned frame must not join onto anything.
    parser.parse(kAllCentre.data() + 7, kAllCentre.size() - 7U, sink);
    EXPECT_TRUE(sink.frames.empty());
}

TEST(CrsfParserTest, BufferStaysBoundedWhenFramesNeverComplete)
{
    RecordingSink sink;
    CrsfParser parser;

    // A sync byte plus a large length that never completes, repeated forever.
    const Bytes teaser{kSyncByte, kMaxLengthByte, 0x16, 0x00};

    for (int i = 0; i < 5000; ++i) {
        parser.parse(teaser.data(), teaser.size(), sink);
    }

    EXPECT_LE(parser.bufferedBytes(), 2U * kMaxFrameSize);
    EXPECT_TRUE(sink.frames.empty());
}

TEST(CrsfParserTest, SurvivesRandomBytesAndStillDecodesAfterwards)
{
    RecordingSink sink;
    CrsfParser parser;

    std::mt19937 rng(20250917);
    std::uniform_int_distribution<int> byte_dist(0, 255);

    for (int chunk = 0; chunk < 1024; ++chunk) {
        Bytes noise(64);

        for (std::uint8_t & byte : noise) {
            byte = static_cast<std::uint8_t>(byte_dist(rng));
        }

        parser.parse(noise.data(), noise.size(), sink);
    }

    EXPECT_LE(parser.bufferedBytes(), 2U * kMaxFrameSize);

    // Whatever the noise did, a real frame after it must still decode.
    parser.reset();
    const std::size_t before = sink.frames.size();
    parser.parse(kDistinct.data(), kDistinct.size(), sink);

    ASSERT_EQ(sink.frames.size(), before + 1U);
    EXPECT_EQ(sink.frames.back().channels[7], kDistinctValues[7]);
}

TEST(CrsfParserTest, NullAndEmptyInputAreSafe)
{
    RecordingSink sink;
    CrsfParser parser;

    parser.parse(nullptr, 0U, sink);
    parser.parse(nullptr, 16U, sink);
    parser.parse(kAllCentre.data(), 0U, sink);

    EXPECT_TRUE(sink.frames.empty());
    EXPECT_EQ(parser.bufferedBytes(), 0U);
}

}  // namespace
}  // namespace rover_crsf_teleop::crsf
