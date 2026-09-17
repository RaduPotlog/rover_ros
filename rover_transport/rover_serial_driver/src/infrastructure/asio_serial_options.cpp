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
// Modified 2026 by Mechatronics Academy: extracted from SerialPortConfig's get_*_asio()
// members (ros-drivers/transport_drivers v1.2.0).

#include "rover_serial_driver/infrastructure/asio_serial_options.hpp"

namespace rover::transport::serial
{

SerialPortBase::baud_rate toAsioBaudRate(const SerialPortConfig & config)
{
    return SerialPortBase::baud_rate{config.getBaudRate()};
}

SerialPortBase::flow_control::type toAsioFlowControl(FlowControl flow_control)
{
    switch (flow_control) {
        case FlowControl::HARDWARE:
            return SerialPortBase::flow_control::hardware;
        case FlowControl::SOFTWARE:
            return SerialPortBase::flow_control::software;
        case FlowControl::NONE:
        default:
            return SerialPortBase::flow_control::none;
    }
}

SerialPortBase::parity::type toAsioParity(Parity parity)
{
    switch (parity) {
        case Parity::ODD:
            return SerialPortBase::parity::odd;
        case Parity::EVEN:
            return SerialPortBase::parity::even;
        case Parity::NONE:
        default:
            return SerialPortBase::parity::none;
    }
}

SerialPortBase::stop_bits::type toAsioStopBits(StopBits stop_bits)
{
    switch (stop_bits) {
        case StopBits::ONE_POINT_FIVE:
            return SerialPortBase::stop_bits::onepointfive;
        case StopBits::TWO:
            return SerialPortBase::stop_bits::two;
        case StopBits::ONE:
        default:
            return SerialPortBase::stop_bits::one;
    }
}

}  // namespace rover::transport::serial
