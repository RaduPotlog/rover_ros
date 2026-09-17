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

#include "rover_gps/domain/nmea/nmea_sentence.hpp"

namespace rover_gps::domain::nmea
{

SentenceType typeOf(const ParsedSentence & sentence)
{
    struct Visitor
    {
        SentenceType operator()(const GgaData &) const { return SentenceType::Gga; }
        SentenceType operator()(const RmcData &) const { return SentenceType::Rmc; }
        SentenceType operator()(const VtgData &) const { return SentenceType::Vtg; }
        SentenceType operator()(const GstData &) const { return SentenceType::Gst; }
        SentenceType operator()(const HdtData &) const { return SentenceType::Hdt; }
    };
    return std::visit(Visitor{}, sentence);
}

const char * sentenceTypeText(SentenceType type)
{
    switch (type) {
        case SentenceType::Gga:
            return "GGA";
        case SentenceType::Rmc:
            return "RMC";
        case SentenceType::Vtg:
            return "VTG";
        case SentenceType::Gst:
            return "GST";
        case SentenceType::Hdt:
            return "HDT";
    }
    return "UNKNOWN";
}

}  // namespace rover_gps::domain::nmea
