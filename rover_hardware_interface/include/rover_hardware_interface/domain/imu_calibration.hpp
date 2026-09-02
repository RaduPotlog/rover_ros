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

#ifndef ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_CALIBRATION_HPP_
#define ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_CALIBRATION_HPP_

#include "rover_hardware_interface/domain/vector3.hpp"

namespace rover_hardware_interface
{

bool isVectorFinite(const Vector3 & vec);

// Named separately from isVectorFinite for call-site readability, even though it currently
// delegates to it: a magnetometer reading is "synchronized" with the accel/gyro sample once it's
// a real (finite) value rather than the sensor's not-yet-available sentinel.
bool isMagnitudeSynchronizedWithAccelerationAndGyration(const Vector3 & mag_compensated);

// Latches true the first time a finite magnetometer reading is observed, and stays true for the
// life of the gate (mirrors ErrorFilter's once-set-stays-set style, but for calibration success
// instead of failure).
class ImuCalibrationGate
{

public:

    bool isCalibrated() const { return calibrated_; }

    bool update(const Vector3 & mag_compensated);

private:

    bool calibrated_ = false;
};

}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_DOMAIN_IMU_CALIBRATION_HPP_
