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

#include "rover_serial_driver/infrastructure/ros2_byte_publisher.hpp"

#include <cstddef>
#include <utility>
#include <vector>

#include "rover_serial_driver/infrastructure/serial_msg_conversions.hpp"

namespace rover::transport::serial
{

Ros2BytePublisher::Ros2BytePublisher(PublisherPtr publisher)
: publisher_(std::move(publisher))
{
}

void Ros2BytePublisher::publish(const std::vector<uint8_t> & buffer, std::size_t length)
{
    if (!publisher_) {
        return;
    }

    std_msgs::msg::UInt8MultiArray out;
    toMsg(buffer, out, length);
    publisher_->publish(out);
}

}  // namespace rover::transport::serial
