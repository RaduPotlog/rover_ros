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

#ifndef ROVER_RS16_LIDAR_DOMAIN_PORTS_LIDAR_SOURCE_PORT_HPP_
#define ROVER_RS16_LIDAR_DOMAIN_PORTS_LIDAR_SOURCE_PORT_HPP_

#include <functional>
#include <string>

#include "rover_rs16_lidar/domain/point_cloud_frame.hpp"

namespace rover_rs16_lidar::domain
{

/**
 * @brief Input port: something that produces point-cloud frames.
 * @details Implemented over rs_driver in production and over a stub in tests, which is what
 *          lets the whole pipeline be exercised without a sensor on the network.
 *          The callbacks are invoked on the source's own thread, not on the ROS executor.
 */
class LidarSourcePort
{
public:
    using FrameCallback = std::function<void(const PointCloudFrame &)>;
    /// Reports a driver-level problem (socket error, wrong packet, dropped frame).
    using ErrorCallback = std::function<void(const std::string &)>;

    virtual ~LidarSourcePort() = default;

    /** @brief Must be called before start(). */
    virtual void setFrameCallback(FrameCallback callback) = 0;

    virtual void setErrorCallback(ErrorCallback callback) = 0;

    /** @throws std::runtime_error when the source cannot be opened. */
    virtual void start() = 0;

    /** @brief Idempotent; no callback fires once it returns. */
    virtual void stop() = 0;
};

}  // namespace rover_rs16_lidar::domain

#endif  // ROVER_RS16_LIDAR_DOMAIN_PORTS_LIDAR_SOURCE_PORT_HPP_
