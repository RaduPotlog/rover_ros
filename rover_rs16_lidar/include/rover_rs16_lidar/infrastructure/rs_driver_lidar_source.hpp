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

#ifndef ROVER_RS16_LIDAR_INFRASTRUCTURE_RS_DRIVER_LIDAR_SOURCE_HPP_
#define ROVER_RS16_LIDAR_INFRASTRUCTURE_RS_DRIVER_LIDAR_SOURCE_HPP_

#include <memory>

#include "rover_rs16_lidar/domain/lidar_settings.hpp"
#include "rover_rs16_lidar/domain/ports/lidar_source_port.hpp"

namespace rover_rs16_lidar::infrastructure
{

/**
 * @brief The RS16 itself, behind LidarSourcePort.
 * @details PImpl on purpose: rs_driver is a large header-only library and this is the only
 *          translation unit in the package allowed to include it.
 *
 *          rs_driver hands finished revolutions back through a pair of callbacks - one asking
 *          for a buffer, one returning a filled buffer - so buffers are recycled through a
 *          free queue and, after the first few revolutions, nothing is allocated per frame.
 *          A single worker thread drains the filled queue and invokes the frame callback, so
 *          the decoding threads are never blocked by whatever the consumer does.
 */
class RsDriverLidarSource : public domain::LidarSourcePort
{
public:
    explicit RsDriverLidarSource(domain::SensorSettings settings);

    ~RsDriverLidarSource() override;

    RsDriverLidarSource(const RsDriverLidarSource &) = delete;
    RsDriverLidarSource & operator=(const RsDriverLidarSource &) = delete;

    void setFrameCallback(FrameCallback callback) override;

    void setErrorCallback(ErrorCallback callback) override;

    /** @throws std::runtime_error when the driver cannot open its sockets or the pcap file. */
    void start() override;

    void stop() override;

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace rover_rs16_lidar::infrastructure

#endif  // ROVER_RS16_LIDAR_INFRASTRUCTURE_RS_DRIVER_LIDAR_SOURCE_HPP_
