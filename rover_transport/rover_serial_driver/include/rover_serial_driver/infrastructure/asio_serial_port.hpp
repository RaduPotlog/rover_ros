// Copyright 2021 LeoDrive, Copyright 2021 The Autoware Foundation
// Copyright 2021 Trimble (c)
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
// Modified 2026 by Mechatronics Academy: SerialPort ->
// AsioSerialPort implementing rover::transport::ByteStreamPort
// (ros-drivers/transport_drivers v1.2.0). SerialDriver, a one-member shared_ptr holder,
// was dropped in favour of the makeSerialPort() factory below.

#ifndef ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_PORT_HPP_
#define ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_PORT_HPP_

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <asio.hpp>

#include "rover_io_context/domain/ports.hpp"
#include "rover_io_context/infrastructure/io_context.hpp"
#include "rover_serial_driver/domain/serial_port_config.hpp"

namespace rover::transport::serial
{

class AsioSerialPort : public ByteStreamPort
{

public:

    AsioSerialPort(
        const IoContext & ctx,
        const std::string & device_name,
        const SerialPortConfig & serial_port_config);

    ~AsioSerialPort() override;

    AsioSerialPort(const AsioSerialPort &) = delete;
    AsioSerialPort & operator=(const AsioSerialPort &) = delete;

    std::string deviceName() const;

    SerialPortConfig serialPortConfig() const;

    // ByteStreamPort
    void open() override;
    void close() override;
    bool isOpen() const override;
    void asyncSend(const std::vector<uint8_t> & buffer) override;
    void asyncReceive(ByteReceiveCallback callback) override;

    // Blocking operations, kept off ByteStreamPort: nothing in this workspace calls them,
    // they exist for the upstream tests.
    std::size_t send(const std::vector<uint8_t> & buffer);

    std::size_t receive(std::vector<uint8_t> & buffer);

    // The port must be open first.
    bool sendBreak();

private:

    void asyncSendHandler(const asio::error_code & error, std::size_t bytes_transferred);

    void asyncReceiveHandler(const asio::error_code & error, std::size_t bytes_transferred);

    const IoContext & ctx_;
    std::string device_name_;
    asio::serial_port serial_port_;
    SerialPortConfig port_config_;
    ByteReceiveCallback callback_;
    static constexpr std::size_t kRecvBufferSize{2048};
    std::vector<uint8_t> recv_buffer_;
};

// Replaces SerialDriver::init_port(). Returns an unopened port; the caller opens it so a
// failure surfaces inside a lifecycle transition it can report on.
std::unique_ptr<ByteStreamPort> makeSerialPort(
    const IoContext & ctx,
    const std::string & device_name,
    const SerialPortConfig & config);

}  // namespace rover::transport::serial

#endif  // ROVER_SERIAL_DRIVER_INFRASTRUCTURE_ASIO_SERIAL_PORT_HPP_
