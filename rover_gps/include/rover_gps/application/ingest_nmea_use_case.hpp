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

#ifndef ROVER_GPS_APPLICATION_INGEST_NMEA_USE_CASE_HPP_
#define ROVER_GPS_APPLICATION_INGEST_NMEA_USE_CASE_HPP_

#include <ctime>
#include <memory>
#include <string_view>

#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"
#include "rover_gps/domain/ports/nmea_output_port.hpp"

namespace rover_gps::application
{

/** @brief What one sentence turned into. */
enum class IngestResult
{
    Published,       // parsed and produced at least one message
    NoOutput,        // parsed, but this sentence produces nothing on its own (e.g. GST)
    ChecksumFailed,  // the trailing *XX did not match
    Unparsed,        // unknown talker or sentence type
};

/** @brief Counters for diagnostics and logging. */
struct IngestStatistics
{
    std::size_t published{0};
    std::size_t no_output{0};
    std::size_t checksum_failed{0};
    std::size_t unparsed{0};
};

/**
 * @brief Validates, parses and assembles NMEA sentences, then pushes the results to the output port.
 * @details Not thread-safe; the driver node drives it from its receive thread only.
 */
class IngestNmeaUseCase
{
public:
    IngestNmeaUseCase(
        const domain::nmea::FixAssemblerConfig & config,
        std::shared_ptr<domain::NmeaOutputPort> output);

    /**
     * @brief Handles one sentence.
     * @param raw      One sentence, already trimmed of CR/LF and surrounding whitespace.
     * @param stamp_s  Receive time in seconds, used as the message header stamp.
     * @param now_utc  Current UTC time, used to date NMEA's bare time-of-day fields.
     */
    IngestResult onSentence(std::string_view raw, double stamp_s, std::time_t now_utc);

    const IngestStatistics & statistics() const { return statistics_; }

    /** @brief Clears the latched fix validity, receiver error estimate and counters. */
    void reset();

private:
    domain::nmea::NmeaFixAssembler assembler_;
    std::shared_ptr<domain::NmeaOutputPort> output_;
    IngestStatistics statistics_{};
};

}  // namespace rover_gps::application

#endif  // ROVER_GPS_APPLICATION_INGEST_NMEA_USE_CASE_HPP_
