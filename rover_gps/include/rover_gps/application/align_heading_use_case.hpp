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

#ifndef ROVER_GPS_APPLICATION_ALIGN_HEADING_USE_CASE_HPP_
#define ROVER_GPS_APPLICATION_ALIGN_HEADING_USE_CASE_HPP_

#include <memory>

#include "rover_gps/domain/gnss_fix.hpp"
#include "rover_gps/domain/heading_alignment_estimator.hpp"
#include "rover_gps/domain/ports/heading_publisher_port.hpp"

namespace rover_gps::application
{

struct AlignHeadingSettings
{
    // False: only estimate and report the alignment, never publish a heading.
    bool publish_heading{false};
    // Lower bound of the published yaw standard deviation [rad].
    double min_heading_std_rad{0.05};
};

/**
 * @brief Aligns odometry yaw to ENU from GNSS course, then republishes every odometry yaw as an
 *        absolute ENU heading (the orientation input navsat_transform_node needs).
 */
class AlignHeadingUseCase
{
public:
    AlignHeadingUseCase(
        std::shared_ptr<domain::HeadingPublisherPort> publisher,
        domain::HeadingAlignmentConfig config,
        AlignHeadingSettings settings);

    void onFix(const domain::GnssFix & fix);

    void onOdometry(const domain::OdometrySample & sample);

    void onTick();

    void reset();

    bool aligned() const {return estimator_.aligned();}

private:
    std::shared_ptr<domain::HeadingPublisherPort> publisher_;
    domain::HeadingAlignmentEstimator estimator_;
    AlignHeadingSettings settings_;
};

}  // namespace rover_gps::application

#endif  // ROVER_GPS_APPLICATION_ALIGN_HEADING_USE_CASE_HPP_
