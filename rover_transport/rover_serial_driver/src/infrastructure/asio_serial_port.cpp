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
// Modified 2026 by Mechatronics Academy: relayouted from serial_driver/src/serial_port.cpp
// (ros-drivers/transport_drivers v1.2.0). Two defects fixed while moving - see asyncSend()
// and asyncReceiveHandler(). Later: close() waits for the port's handlers (AsyncOpGuard), so
// the owner can destroy what the callback uses.

#include "rover_serial_driver/infrastructure/asio_serial_port.hpp"

#include <cstddef>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include <rclcpp/logging.hpp>

#include "rover_serial_driver/infrastructure/asio_serial_options.hpp"

namespace rover::transport::serial
{

AsioSerialPort::AsioSerialPort(
    const IoContext & ctx,
    const std::string & device_name,
    const SerialPortConfig & serial_port_config)
: ctx_(ctx),
  device_name_(device_name),
  guard_(ctx.ios()),
  serial_port_(ctx.ios()),
  port_config_(serial_port_config)
{
    recv_buffer_.resize(kRecvBufferSize);
}

AsioSerialPort::~AsioSerialPort()
{
    if (isOpen()) {
        close();
    }
}

std::size_t AsioSerialPort::send(const std::vector<uint8_t> & buffer)
{
    return serial_port_.write_some(asio::buffer(buffer.data(), buffer.size()));
}

std::size_t AsioSerialPort::receive(std::vector<uint8_t> & buffer)
{
    return serial_port_.read_some(asio::mutable_buffer(buffer.data(), buffer.size()));
}

void AsioSerialPort::asyncSend(const std::vector<uint8_t> & buffer)
{
    // Upstream passed asio::buffer(buff) over a caller-owned vector that went out of scope
    // as soon as the subscriber callback returned, leaving the async write reading freed
    // memory. Owning a copy for the lifetime of the operation is the cheap, correct fix.
    auto payload = std::make_shared<std::vector<uint8_t>>(buffer);

    guard_.post(
        [this, payload]()
        {
            if (!serial_port_.is_open()) {
                return;  // Closed after this write was queued.
            }
            serial_port_.async_write_some(
                asio::buffer(*payload),
                guard_.wrap(
                    [this, payload](std::error_code error, std::size_t bytes_transferred)
                    {
                        asyncSendHandler(error, bytes_transferred);
                    }));
        });
}

void AsioSerialPort::asyncReceive(ByteReceiveCallback callback)
{
    callback_ = std::move(callback);
    guard_.post([this]() {armReceive();});
}

void AsioSerialPort::armReceive()
{
    serial_port_.async_read_some(
        asio::buffer(recv_buffer_),
        guard_.wrap(
            [this](std::error_code error, std::size_t bytes_transferred)
            {
                asyncReceiveHandler(error, bytes_transferred);
            }));
}

bool AsioSerialPort::sendBreak()
{
    bool break_sent = false;
    if (isOpen()) {
        serial_port_.send_break();
        break_sent = true;
    }
    return break_sent;
}

void AsioSerialPort::asyncSendHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred)
{
    (void)bytes_transferred;
    if (error) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("AsioSerialPort::asyncSendHandler"), error.message());
    }
}

void AsioSerialPort::asyncReceiveHandler(
    const asio::error_code & error,
    std::size_t bytes_transferred)
{
    if (error == asio::error::operation_aborted || !serial_port_.is_open()) {
        // close() cancelled the pending read, or ran after these bytes arrived but before
        // their handler did - a normal part of shutdown/cleanup. Either way the callback's
        // targets may be going away: don't deliver, don't re-arm.
        return;
    }

    if (error) {
        RCLCPP_ERROR_STREAM(
            rclcpp::get_logger("AsioSerialPort::asyncReceiveHandler"), error.message());
        // Note: the port is closed and never reopened, matching upstream. Auto-reconnect
        // would belong here, but it is a behaviour change and stays out of the relayout.
        close();
        return;
    }

    if (bytes_transferred > 0 && callback_) {
        callback_(recv_buffer_, bytes_transferred);
        armReceive();
    }
}

std::string AsioSerialPort::deviceName() const
{
    return device_name_;
}

SerialPortConfig AsioSerialPort::serialPortConfig() const
{
    return port_config_;
}

void AsioSerialPort::open()
{
    serial_port_.open(device_name_);
    serial_port_.set_option(toAsioBaudRate(port_config_));
    serial_port_.set_option(SerialPortBase::flow_control(
            toAsioFlowControl(port_config_.getFlowControl())));
    serial_port_.set_option(SerialPortBase::parity(toAsioParity(port_config_.getParity())));
    serial_port_.set_option(SerialPortBase::stop_bits(
            toAsioStopBits(port_config_.getStopBits())));
}

void AsioSerialPort::close()
{
    // Returns only once no handler of this port is running or queued, so the caller may
    // destroy the receive callback's targets (and this port) right after. From a handler
    // (asyncReceiveHandler() on a read error) it closes straight away.
    guard_.closeAndDrain(
        [this]()
        {
            asio::error_code error;
            serial_port_.close(error);
            if (error) {
                RCLCPP_ERROR_STREAM(rclcpp::get_logger("AsioSerialPort::close"), error.message());
            }
        });
}

bool AsioSerialPort::isOpen() const
{
    return serial_port_.is_open();
}

std::unique_ptr<ByteStreamPort> makeSerialPort(
    const IoContext & ctx,
    const std::string & device_name,
    const SerialPortConfig & config)
{
    return std::make_unique<AsioSerialPort>(ctx, device_name, config);
}

}  // namespace rover::transport::serial
