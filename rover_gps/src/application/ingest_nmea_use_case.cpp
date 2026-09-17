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

#include "rover_gps/application/ingest_nmea_use_case.hpp"

#include <stdexcept>
#include <utility>

#include "rover_gps/domain/nmea/nmea_checksum.hpp"
#include "rover_gps/domain/nmea/nmea_parser.hpp"

namespace rover_gps::application
{

IngestNmeaUseCase::IngestNmeaUseCase(
    const domain::nmea::FixAssemblerConfig & config,
    std::shared_ptr<domain::NmeaOutputPort> output)
: assembler_(config)
, output_(std::move(output))
{
    if (output_ == nullptr) {
        throw std::invalid_argument("IngestNmeaUseCase requires a non-null output port");
    }
}

void IngestNmeaUseCase::reset()
{
    assembler_.reset();
    statistics_ = IngestStatistics{};
}

IngestResult IngestNmeaUseCase::onSentence(
    std::string_view raw, double stamp_s, std::time_t now_utc)
{
    if (!domain::nmea::verifyChecksum(raw)) {
        ++statistics_.checksum_failed;
        return IngestResult::ChecksumFailed;
    }

    const auto parsed = domain::nmea::parseSentence(raw, now_utc);
    if (!parsed.has_value()) {
        ++statistics_.unparsed;
        return IngestResult::Unparsed;
    }

    const domain::nmea::AssemblerOutputs outputs = assembler_.update(*parsed);
    if (outputs.empty()) {
        ++statistics_.no_output;
        return IngestResult::NoOutput;
    }

    // Fix before time reference, matching the upstream publish order.
    if (outputs.fix) {
        output_->publishFix(*outputs.fix, stamp_s);
    }
    if (outputs.time_reference) {
        output_->publishTimeReference(*outputs.time_reference, stamp_s);
    }
    if (outputs.velocity) {
        output_->publishVelocity(*outputs.velocity, stamp_s);
    }
    if (outputs.heading) {
        output_->publishHeading(*outputs.heading, stamp_s);
    }

    ++statistics_.published;
    return IngestResult::Published;
}

}  // namespace rover_gps::application
