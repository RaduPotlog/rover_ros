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

#ifndef ROVER_HARDWARE_INTERFACE_TEST_ROVER_SAFETY_CONTROLLER_FAKE_ROVER_MODBUS_HPP_
#define ROVER_HARDWARE_INTERFACE_TEST_ROVER_SAFETY_CONTROLLER_FAKE_ROVER_MODBUS_HPP_

#include <mutex>
#include <vector>

#include "rover_hardware_interface/rover_modbus/rover_modbus_interface.hpp"

namespace rover_hardware_interface
{
namespace test
{

struct CoilWrite
{
    Coil coil;
    bool state;

    bool operator==(const CoilWrite & other) const
    {
        return coil == other.coil && state == other.state;
    }
};

// Thread-safe in-memory fake of RoverModbusInterface: records every coil write (so tests can
// assert what ContactCoilHandler/RoverSafetyController sent) and returns a configurable canned
// value for reads. No real Modbus/network I/O - safe to construct and drive from a unit test,
// including from ContactCoilHandler's background poll thread.
class FakeRoverModbus : public RoverModbusInterface
{

public:

    uint16_t readDiscreteContact(const ContactInfo & contact) override
    {
        (void)contact;
        std::lock_guard<std::mutex> lock(mutex_);
        return contact_read_value_;
    }

    uint16_t readDiscreteCoil(const CoilInfo & coil) override
    {
        (void)coil;
        std::lock_guard<std::mutex> lock(mutex_);
        return coil_read_value_;
    }

    void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) override
    {
        std::lock_guard<std::mutex> lock(mutex_);
        writes_.push_back({coil.coil, coil_state});
    }

    // --- Test-only helpers below; not part of RoverModbusInterface. ---

    std::vector<CoilWrite> writesSnapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return writes_;
    }

    bool hasWrite(const CoilWrite & write) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & recorded : writes_) {
            if (recorded == write) {
                return true;
            }
        }
        return false;
    }

    void setContactReadValue(const uint16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        contact_read_value_ = value;
    }

    void setCoilReadValue(const uint16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        coil_read_value_ = value;
    }

private:

    mutable std::mutex mutex_;
    std::vector<CoilWrite> writes_;
    uint16_t contact_read_value_ = 0;
    uint16_t coil_read_value_ = 0;
};

}  // namespace test
}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_TEST_ROVER_SAFETY_CONTROLLER_FAKE_ROVER_MODBUS_HPP_
