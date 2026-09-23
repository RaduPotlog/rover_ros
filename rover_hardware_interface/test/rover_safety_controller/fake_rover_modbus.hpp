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

#include <atomic>
#include <chrono>
#include <cstdint>
#include <map>
#include <mutex>
#include <stdexcept>
#include <thread>
#include <vector>

#include "rover_hardware_interface/rover_safety_controller/rover_safety_controller_types.hpp"

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

// A coil write with the steady_clock time it was accepted, so timing tests can measure the
// interval between consecutive heartbeat toggles.
struct TimedCoilWrite
{
    Coil coil;
    bool state;
    std::chrono::steady_clock::time_point at;
};

// Thread-safe in-memory fake of the driver's DiscreteIoPort: records every coil write (so tests can
// assert what ContactCoilHandler/RoverSafetyController sent) and returns a configurable canned
// value for reads. No real Modbus/network I/O - safe to construct and drive from a unit test,
// including from ContactCoilHandler's background threads.
//
// It models two behaviours of the real ModbusDiscreteIoClient that tests depend on:
//
//  * `is_coil_engage_allowed` - a write to a read-only coil is refused and recorded separately,
//    exactly as ModbusDiscreteIoClient::writeDiscreteCoil() does. The fake used to ignore the
//    flag, which let test_contact_coil_handler assert that COIL_0/COIL_5 were written when in
//    production those writes were rejected and logged as errors.
//
//  * read latency and faults - `setReadDelay()` stalls each read the way a slow or timing-out
//    Modbus round-trip does, and `setFailReadsWithException()` throws from a read. Both exist so
//    the heartbeat's independence from the IO poll can be tested directly rather than inferred.
class FakeRoverModbus : public DiscreteIoPort
{

public:

    uint16_t readDiscreteContact(const ContactInfo & contact) override
    {
        (void)contact;
        applyTransactionDelay();
        throwIfReadsFail();
        std::lock_guard<std::mutex> lock(mutex_);
        return contact_read_value_;
    }

    uint16_t readDiscreteCoil(const CoilInfo & coil) override
    {
        (void)coil;
        applyTransactionDelay();
        throwIfReadsFail();
        std::lock_guard<std::mutex> lock(mutex_);

        const auto override_it = coil_read_overrides_.find(coil.coil);

        return (override_it != coil_read_overrides_.end()) ? override_it->second
                                                           : coil_read_value_;
    }

    void writeDiscreteCoil(const CoilInfo & coil, const bool coil_state) override
    {
        // Mirrors ModbusDiscreteIoClient::writeDiscreteCoil(): the guard is checked before any
        // transaction happens, so a refused write costs no time on the wire.
        if (!coil.is_coil_engage_allowed) {
            std::lock_guard<std::mutex> lock(mutex_);
            refused_writes_.push_back({coil.coil, coil_state});
            return;
        }

        std::lock_guard<std::mutex> lock(mutex_);
        writes_.push_back({coil.coil, coil_state});
        timed_writes_.push_back({coil.coil, coil_state, std::chrono::steady_clock::now()});
    }

    // Batched reads: one "transaction" (one delay, one counted read) for the whole range, each
    // bit derived from the same canned values the single reads return.
    std::vector<bool> readDiscreteContacts(const Contact first, const uint16_t count) override
    {
        (void)first;
        applyTransactionDelay();
        throwIfReadsFail();
        std::lock_guard<std::mutex> lock(mutex_);
        read_transactions_++;

        return std::vector<bool>(count, toBit(contact_read_value_));
    }

    std::vector<bool> readDiscreteCoils(const Coil first, const uint16_t count) override
    {
        applyTransactionDelay();
        throwIfReadsFail();
        std::lock_guard<std::mutex> lock(mutex_);
        read_transactions_++;
        coil_read_requests_.push_back({static_cast<uint16_t>(first), count});

        // With a PLC area map set, behave like the Portenta PLC IDE: a read is served from the
        // area its first address falls in, and bits past that area's end come back false.
        const uint16_t first_address = static_cast<uint16_t>(first);
        uint16_t served_end = UINT16_MAX;

        for (const auto & area : coil_areas_) {
            if (first_address >= area.first && first_address < area.first + area.count) {
                served_end = static_cast<uint16_t>(area.first + area.count);
            }
        }

        std::vector<bool> bits(count);

        for (uint16_t i = 0; i < count; ++i) {
            const uint16_t address = static_cast<uint16_t>(first_address + i);

            if (address >= served_end) {
                bits[i] = false;
                continue;
            }

            const auto override_it = coil_read_overrides_.find(static_cast<Coil>(address));

            bits[i] = toBit(
                (override_it != coil_read_overrides_.end()) ? override_it->second : coil_read_value_);
        }

        return bits;
    }

    // --- Test-only helpers below; not part of DiscreteIoPort. ---

    struct CoilRange
    {
        uint16_t first;
        uint16_t count;
    };

    // The PLC's coil memory areas. Unset (the default), coils form one flat array.
    void setCoilAreas(const std::vector<CoilRange> & areas)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        coil_areas_ = areas;
    }

    // (first, count) of every batched coil read, in order.
    std::vector<CoilRange> coilReadRequests() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return coil_read_requests_;
    }

    // Number of read transactions served by the batched reads - how many round-trips one IO
    // sweep costs.
    uint64_t readTransactionCount() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return read_transactions_;
    }

    std::vector<CoilWrite> writesSnapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return writes_;
    }

    std::vector<CoilWrite> refusedWritesSnapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return refused_writes_;
    }

    std::vector<TimedCoilWrite> timedWritesSnapshot() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return timed_writes_;
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

    bool hasRefusedWrite(const CoilWrite & write) const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        for (const auto & recorded : refused_writes_) {
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

    // Per-coil read value, for tests that need two coils to disagree (e.g. latch asserted while
    // the contactor is still engaged).
    void setCoilReadValueFor(const Coil coil, const uint16_t value)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        coil_read_overrides_[coil] = value;
    }

    // Stalls every *read*, standing in for a slow link or a response timeout during the IO
    // poll. Writes stay fast on purpose: the question these tests ask is whether a slow poll can
    // delay the heartbeat write, so slowing the write too would mask the answer.
    void setReadDelay(const std::chrono::milliseconds delay)
    {
        read_delay_ms_ = static_cast<uint64_t>(delay.count());
    }

    void setFailReadsWithException(const bool fail)
    {
        fail_reads_ = fail;
    }

private:

    // Same reading of a canned value as the controller applied to single reads: the 255
    // "unavailable" sentinel is inactive, otherwise the low byte decides.
    static bool toBit(const uint16_t value)
    {
        return value != rover::transport::modbus::kDiscreteReadUnavailable && (value & 0xFFU) != 0;
    }

    void applyTransactionDelay() const
    {
        const uint64_t delay_ms = read_delay_ms_.load();

        if (delay_ms > 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(delay_ms));
        }
    }

    void throwIfReadsFail() const
    {
        if (fail_reads_.load()) {
            throw std::runtime_error("fake modbus read failure");
        }
    }

    mutable std::mutex mutex_;
    std::vector<CoilWrite> writes_;
    std::vector<CoilWrite> refused_writes_;
    std::vector<TimedCoilWrite> timed_writes_;
    std::map<Coil, uint16_t> coil_read_overrides_;
    uint16_t contact_read_value_ = 0;
    uint16_t coil_read_value_ = 0;
    uint64_t read_transactions_ = 0;
    std::vector<CoilRange> coil_areas_;
    std::vector<CoilRange> coil_read_requests_;

    std::atomic_uint64_t read_delay_ms_ {0};
    std::atomic_bool fail_reads_ {false};
};

}  // namespace test
}  // namespace rover_hardware_interface

#endif  // ROVER_HARDWARE_INTERFACE_TEST_ROVER_SAFETY_CONTROLLER_FAKE_ROVER_MODBUS_HPP_
