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

#include "rover_gps_heading/application/align_heading_use_case.hpp"

#include <algorithm>
#include <memory>
#include <optional>
#include <stdexcept>
#include <utility>

namespace rover_gps_heading::application
{

AlignHeadingUseCase::AlignHeadingUseCase(
    std::shared_ptr<domain::HeadingPublisherPort> publisher,
    domain::HeadingAlignmentConfig config,
    AlignHeadingSettings settings)
: publisher_(std::move(publisher))
, estimator_(config)
, settings_(settings)
{
    if (!publisher_) {
        throw std::invalid_argument("AlignHeadingUseCase requires a publisher.");
    }
    if (!(settings_.min_heading_std_rad > 0.0)) {
        throw std::invalid_argument("min_heading_std_rad must be positive.");
    }
}

void AlignHeadingUseCase::onFix(const domain::GnssFix & fix)
{
    estimator_.addFix(fix);
}

void AlignHeadingUseCase::onOdometry(const domain::OdometrySample & sample)
{
    estimator_.addOdometry(sample);

    if (!settings_.publish_heading) {
        return;
    }

    const std::optional<double> enu_yaw = estimator_.enuYaw(sample.yaw_rad);
    if (!enu_yaw) {
        return;
    }

    const domain::AlignmentStatus status = estimator_.status();
    domain::EnuHeading heading;
    heading.stamp_s = sample.stamp_s;
    heading.yaw_rad = *enu_yaw;
    heading.yaw_std_rad =
        std::max(settings_.min_heading_std_rad, status.offset_std_rad.value_or(0.0));
    publisher_->publishHeading(heading);
}

void AlignHeadingUseCase::onTick()
{
    publisher_->publishAlignmentStatus(estimator_.status());
}

void AlignHeadingUseCase::reset()
{
    estimator_.reset();
    publisher_->publishAlignmentStatus(estimator_.status());
}

}  // namespace rover_gps_heading::application
