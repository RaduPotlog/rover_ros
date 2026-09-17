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
//
// Derived from nmea_navsat_driver (BSD-3-Clause, Copyright (c) 2013 Eric Perko):
// libnmea_navsat_driver/driver.py `Ros2NMEADriver.add_sentence`.

#ifndef ROVER_GPS_DOMAIN_NMEA_NMEA_FIX_ASSEMBLER_HPP_
#define ROVER_GPS_DOMAIN_NMEA_NMEA_FIX_ASSEMBLER_HPP_

#include <array>
#include <optional>

#include "rover_gps/domain/gnss_fix.hpp"
#include "rover_gps/domain/nmea/nmea_sentence.hpp"

namespace rover_gps::domain::nmea
{

/** @brief Mirrors sensor_msgs/NavSatFix COVARIANCE_TYPE_*. */
enum class CovarianceType
{
    Unknown,
    Approximated,
};

/** @brief A position fix ready to be turned into a NavSatFix. */
struct FixOutput
{
    double latitude_deg{kNaN};
    double longitude_deg{kNaN};
    double altitude_m{kNaN};
    FixStatus status{FixStatus::NoFix};
    CovarianceType covariance_type{CovarianceType::Unknown};
    // Row-major 3x3; only the diagonal is ever populated.
    std::array<double, 9> position_covariance{};
};

/**
 * @brief Ground velocity.
 * @note The axis convention is inherited from the upstream driver, which writes
 *       `x = speed * sin(course)` and `y = speed * cos(course)` - i.e. x is east and y is north,
 *       not the body-frame convention a TwistStamped usually carries. Downstream consumers are
 *       tuned to this, so it is kept as-is.
 */
struct VelocityOutput
{
    double east_m_s{0.0};
    double north_m_s{0.0};
};

/**
 * @brief True heading from an HDT sentence.
 * @note Radians clockwise from true north, exactly as the receiver sent it. It is deliberately
 *       *not* rotated into ENU here, matching upstream.
 */
struct HeadingOutput
{
    double yaw_rad{0.0};
};

/** @brief Receiver UTC, seconds since the UNIX epoch. */
struct TimeRefOutput
{
    double utc_time_s{0.0};
};

/** @brief Whatever one sentence produced; every member may be empty. */
struct AssemblerOutputs
{
    std::optional<FixOutput> fix;
    std::optional<VelocityOutput> velocity;
    std::optional<HeadingOutput> heading;
    std::optional<TimeRefOutput> time_reference;

    bool empty() const
    {
        return !fix && !velocity && !heading && !time_reference;
    }
};

/** @brief Default estimated position error [m] per GGA quality indicator. */
struct EpeQualityDefaults
{
    double quality0{1000000.0};   // invalid / unknown
    double quality1{4.0};         // SPS
    double quality2{0.1};         // DGPS
    double quality4{0.02};        // RTK fixed
    double quality5{4.0};         // RTK float
    double quality9{3.0};         // WAAS
};

/** @brief Configuration of @ref NmeaFixAssembler. */
struct FixAssemblerConfig
{
    // When true, the fix comes from RMC and GGA/VTG are ignored entirely.
    bool use_rmc{false};
    EpeQualityDefaults epe{};
};

/**
 * @brief Turns a stream of parsed NMEA sentences into fix / velocity / heading / time outputs.
 * @details Stateful: a GGA fix gates VTG velocity, and a GST sentence latches the receiver's own
 *          error estimate over the configured defaults for every later fix.
 *          Not thread-safe; the driver node calls it from its receive thread only.
 */
class NmeaFixAssembler
{
public:
    explicit NmeaFixAssembler(const FixAssemblerConfig & config);

    /** @brief Feeds one sentence and returns everything it produced. */
    AssemblerOutputs update(const ParsedSentence & sentence);

    /** @brief Drops the latched fix validity and receiver error estimate. */
    void reset();

    bool hasValidFix() const { return valid_fix_; }

    bool usingReceiverEpe() const { return using_receiver_epe_; }

private:
    AssemblerOutputs onGga(const GgaData & data);
    AssemblerOutputs onRmc(const RmcData & data);
    AssemblerOutputs onVtg(const VtgData & data) const;
    void onGst(const GstData & data);
    AssemblerOutputs onHdt(const HdtData & data) const;

    FixAssemblerConfig config_;

    bool valid_fix_{false};
    bool using_receiver_epe_{false};
    double lat_std_dev_{kNaN};
    double lon_std_dev_{kNaN};
    double alt_std_dev_{kNaN};
};

}  // namespace rover_gps::domain::nmea

#endif  // ROVER_GPS_DOMAIN_NMEA_NMEA_FIX_ASSEMBLER_HPP_
