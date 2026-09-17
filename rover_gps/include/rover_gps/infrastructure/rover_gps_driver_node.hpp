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

#ifndef ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_DRIVER_NODE_HPP_
#define ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_DRIVER_NODE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

#include "rclcpp_lifecycle/lifecycle_node.hpp"

#include "rover_gps/application/ingest_nmea_use_case.hpp"
#include "rover_gps/domain/nmea/nmea_fix_assembler.hpp"
#include "rover_gps/infrastructure/ros2_nmea_publisher.hpp"
#include "rover_gps/infrastructure/udp_nmea_receiver.hpp"

namespace rover_gps
{

/**
 * @brief NMEA-over-UDP GNSS driver: the C++ replacement for nmea_navsat_driver's nmea_socket_driver.
 * @details Lifecycle-managed because it owns the UDP socket: the socket is bound on activate and
 *          released on deactivate, so the port can be freed for debugging without killing the
 *          process. Parameter names are kept identical to the Python driver so existing
 *          configuration and launch files work unchanged.
 *
 *          Threading: a single receive thread blocks in recvfrom and feeds the use case directly.
 *          The socket's receive timeout bounds that block, so the thread notices a deactivate
 *          request promptly even when the link has gone quiet.
 */
class RoverGpsDriverNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

    explicit RoverGpsDriverNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

    ~RoverGpsDriverNode() override;

    CallbackReturn on_configure(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_activate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_deactivate(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_cleanup(const rclcpp_lifecycle::State & state) override;

    CallbackReturn on_shutdown(const rclcpp_lifecycle::State & state) override;

    /** @brief The UDP port actually bound; differs from the requested one when `port` is 0. */
    uint16_t boundPort() const { return receiver_.boundPort(); }

    /** @brief Sentence counters, for tests and diagnostics. */
    application::IngestStatistics statistics() const;

private:
    /** @brief Resolves frame_id against tf_prefix, reproducing the upstream get_frame_id(). */
    std::string resolveFrameId() const;

    void receiveLoop();

    /** @brief Splits one datagram into sentences and feeds each to the use case. */
    void handleDatagram(std::string_view payload);

    void stopReceiving();

    std::string listen_ip_;
    uint16_t listen_port_{10110};
    std::size_t buffer_size_{4096};
    double timeout_s_{2.0};
    std::string frame_id_;
    std::string time_ref_source_;
    domain::nmea::FixAssemblerConfig assembler_config_;

    std::shared_ptr<infrastructure::Ros2NmeaPublisher> publisher_;
    std::unique_ptr<application::IngestNmeaUseCase> ingest_;

    infrastructure::UdpNmeaReceiver receiver_;
    std::thread receive_thread_;
    std::atomic<bool> running_{false};
};

}  // namespace rover_gps

#endif  // ROVER_GPS_INFRASTRUCTURE_ROVER_GPS_DRIVER_NODE_HPP_
