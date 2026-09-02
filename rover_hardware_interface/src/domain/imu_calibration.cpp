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

#include "rover_hardware_interface/domain/imu_calibration.hpp"

#include <cmath>

namespace rover_hardware_interface
{

bool isVectorFinite(const Vector3 & vec)
{
    return std::isfinite(vec.x) && std::isfinite(vec.y) && std::isfinite(vec.z);
}

bool isMagnitudeSynchronizedWithAccelerationAndGyration(const Vector3 & mag_compensated)
{
    return isVectorFinite(mag_compensated);
}

bool ImuCalibrationGate::update(const Vector3 & mag_compensated)
{
    if (calibrated_) {
        return true;
    }

    calibrated_ = isVectorFinite(mag_compensated);

    return calibrated_;
}

}  // namespace rover_hardware_interface
