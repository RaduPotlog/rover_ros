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

#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"

#include <cmath>

namespace rover_gps::domain::nmea
{

namespace
{

constexpr double kDegToRad = 3.14159265358979323846 / 180.0;

/** @brief One row of the GGA quality table. */
struct GpsQuality
{
    double default_epe_m{0.0};
    FixStatus status{FixStatus::NoFix};
    CovarianceType covariance_type{CovarianceType::Unknown};
};

/**
 * @brief Maps a GGA quality indicator to its default error, fix status and covariance type.
 * @details Any indicator outside the table falls back to the "unknown" row, as upstream does by
 *          rewriting the fix type to -1.
 */
GpsQuality qualityFor(int fix_type, const EpeQualityDefaults & epe)
{
    switch (fix_type) {
        case 1:  // SPS
            return {epe.quality1, FixStatus::Fix, CovarianceType::Approximated};
        case 2:  // DGPS
            return {epe.quality2, FixStatus::SbasFix, CovarianceType::Approximated};
        case 4:  // RTK fixed
            return {epe.quality4, FixStatus::GbasFix, CovarianceType::Approximated};
        case 5:  // RTK float
            return {epe.quality5, FixStatus::GbasFix, CovarianceType::Approximated};
        case 9:  // WAAS
            return {epe.quality9, FixStatus::GbasFix, CovarianceType::Approximated};
        case 0:   // invalid
        case -1:  // unknown
        default:
            return {epe.quality0, FixStatus::NoFix, CovarianceType::Unknown};
    }
}

}  // namespace

NmeaFixAssembler::NmeaFixAssembler(const FixAssemblerConfig & config)
: config_(config)
{
}

void NmeaFixAssembler::reset()
{
    valid_fix_ = false;
    using_receiver_epe_ = false;
    lat_std_dev_ = kNaN;
    lon_std_dev_ = kNaN;
    alt_std_dev_ = kNaN;
}

AssemblerOutputs NmeaFixAssembler::update(const ParsedSentence & sentence)
{
    // The branch order mirrors the upstream if/elif chain: with use_rmc set, GGA and VTG are
    // dropped on the floor rather than falling through to another handler.
    switch (typeOf(sentence)) {
        case SentenceType::Gga:
            return config_.use_rmc ? AssemblerOutputs{} : onGga(std::get<GgaData>(sentence));
        case SentenceType::Vtg:
            return config_.use_rmc ? AssemblerOutputs{} : onVtg(std::get<VtgData>(sentence));
        case SentenceType::Rmc:
            return onRmc(std::get<RmcData>(sentence));
        case SentenceType::Gst:
            onGst(std::get<GstData>(sentence));
            return {};
        case SentenceType::Hdt:
            return onHdt(std::get<HdtData>(sentence));
    }
    return {};
}

AssemblerOutputs NmeaFixAssembler::onGga(const GgaData & data)
{
    AssemblerOutputs outputs;

    const GpsQuality quality = qualityFor(data.fix_type, config_.epe);

    FixOutput fix;
    fix.status = quality.status;
    fix.covariance_type = quality.covariance_type;
    valid_fix_ = hasFix(fix.status);

    fix.latitude_deg = data.latitude_deg;
    fix.longitude_deg = data.longitude_deg;
    // GGA altitude is above the geoid; adding the geoid separation gives height above the ellipsoid.
    fix.altitude_m = data.altitude_m + data.mean_sea_level_m;

    // Use the receiver's own error estimate when a GST sentence supplied one, otherwise the
    // configured default for this fix quality.
    if (!using_receiver_epe_ || std::isnan(lon_std_dev_)) {
        lon_std_dev_ = quality.default_epe_m;
    }
    if (!using_receiver_epe_ || std::isnan(lat_std_dev_)) {
        lat_std_dev_ = quality.default_epe_m;
    }
    if (!using_receiver_epe_ || std::isnan(alt_std_dev_)) {
        alt_std_dev_ = quality.default_epe_m * 2.0;
    }

    const double hdop = data.hdop;
    fix.position_covariance[0] = (hdop * lon_std_dev_) * (hdop * lon_std_dev_);
    fix.position_covariance[4] = (hdop * lat_std_dev_) * (hdop * lat_std_dev_);
    // Kept verbatim from upstream, which carries a FIXME here: the altitude term is scaled by an
    // extra factor of two that the horizontal terms do not get. Changing it would move what
    // rover_localization consumes, so it stays until that is revisited deliberately.
    fix.position_covariance[8] = (2.0 * hdop * alt_std_dev_) * (2.0 * hdop * alt_std_dev_);

    outputs.fix = fix;

    if (!std::isnan(data.utc_time_s)) {
        outputs.time_reference = TimeRefOutput{data.utc_time_s};
    }

    return outputs;
}

AssemblerOutputs NmeaFixAssembler::onVtg(const VtgData & data) const
{
    AssemblerOutputs outputs;

    // Only report VTG velocity once a GGA sentence has confirmed a fix.
    if (valid_fix_) {
        outputs.velocity = VelocityOutput{
            data.speed_m_s * std::sin(data.true_course_rad),
            data.speed_m_s * std::cos(data.true_course_rad)};
    }

    return outputs;
}

AssemblerOutputs NmeaFixAssembler::onRmc(const RmcData & data)
{
    AssemblerOutputs outputs;

    if (config_.use_rmc) {
        FixOutput fix;
        fix.status = data.fix_valid ? FixStatus::Fix : FixStatus::NoFix;
        fix.latitude_deg = data.latitude_deg;
        fix.longitude_deg = data.longitude_deg;
        // RMC carries no altitude.
        fix.altitude_m = kNaN;
        fix.covariance_type = CovarianceType::Unknown;
        outputs.fix = fix;

        if (!std::isnan(data.utc_time_s)) {
            outputs.time_reference = TimeRefOutput{data.utc_time_s};
        }
    }

    // Velocity comes from RMC whether or not it is the fix source, since GGA has none.
    if (data.fix_valid) {
        outputs.velocity = VelocityOutput{
            data.speed_m_s * std::sin(data.true_course_rad),
            data.speed_m_s * std::cos(data.true_course_rad)};
    }

    return outputs;
}

void NmeaFixAssembler::onGst(const GstData & data)
{
    // Latch the receiver-provided error estimate; it overrides the per-quality defaults from here on.
    using_receiver_epe_ = true;
    lon_std_dev_ = data.lon_std_dev;
    lat_std_dev_ = data.lat_std_dev;
    alt_std_dev_ = data.alt_std_dev;
}

AssemblerOutputs NmeaFixAssembler::onHdt(const HdtData & data) const
{
    AssemblerOutputs outputs;

    // Upstream tests the heading for truthiness, which both publishes a NaN heading and silently
    // drops a perfectly valid 0 degrees (due north). Test for NaN instead.
    if (!std::isnan(data.heading_deg)) {
        outputs.heading = HeadingOutput{data.heading_deg * kDegToRad};
    }

    return outputs;
}

}  // namespace rover_gps::domain::nmea
