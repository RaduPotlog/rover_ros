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

#include "rover_battery/rover_battery_node.hpp"

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

namespace rover_battery
{

using std::placeholders::_1;

RoverBatteryNode::RoverBatteryNode(
    const std::string & node_name, 
    const std::string & ns, 
    const rclcpp::NodeOptions & options
)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    
}

void RoverBatteryNode::init() 
{
    get = rover_battery::RoverBatteryNode::Data{};
    alarm = rover_battery::RoverBatteryNode::Alarm{};
    
    diagnostic_updater_->setHardwareID("RoverBattery");

    battery_subscriber_ = this->create_subscription<udp_msgs::msg::UdpPacket>(
        "/rover_battery_udp_data", 100,
        std::bind(&RoverBatteryNode::batteryUdpDataCallback, this, _1)
    );

    battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
        this->shared_from_this(), diagnostic_updater_, 2000);

    battery_read_timeout_ = this->create_wall_timer(
        std::chrono::milliseconds(10000), 
        std::bind(&RoverBatteryNode::batteryUdpDataSubsciberTimeoutCallback, this)
    );
}

void RoverBatteryNode::batteryUdpDataCallback(const udp_msgs::msg::UdpPacket::SharedPtr msg)
{
    battery_read_timeout_->reset();

    if (msg->data.size() != (sizeof(Data) + sizeof(Alarm))) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
            "Packet size mismatch! Received: %zu bytes, Expected: %zu bytes.", 
            msg->data.size(), sizeof(Data) + + sizeof(Alarm));
        return;
    }

    std::memcpy(reinterpret_cast<uint8_t*>(&this->get), msg->data.data(), sizeof(Data));
    std::memcpy(reinterpret_cast<uint8_t*>(&this->alarm), msg->data.data() + sizeof(Data), sizeof(Alarm));

    publishBatteryInfo();
}

void RoverBatteryNode::batteryUdpDataSubsciberTimeoutCallback()
{
    RCLCPP_INFO(this->get_logger(), "timeout");
}

std::uint8_t RoverBatteryNode::updateBatteryHealth()
{
    if ((alarm.levelOnePackVoltageTooLow == true) || (alarm.levelOneCellVoltageTooLow == true) || 
        (alarm.levelOneStateOfChargeTooLow == true)) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if ((alarm.levelTwoPackVoltageTooHigh == true) || (alarm.levelTwoStateOfChargeTooHigh == true)) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (alarm.levelOneChargeTempTooHigh == true || alarm.levelOneDischargeTempTooHigh == true) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (alarm.levelOneChargeTempTooLow == true || alarm.levelOneDischargeTempTooLow == true) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD;
    } else {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
}

void RoverBatteryNode::updateFailureCodes()
{
    error_msg_ = "";

    error_msg_ = alarm.levelOneCellVoltageTooHigh == true ? error_msg_ += "levelOneCellVoltageTooHigh \n" : error_msg_;
    error_msg_ = alarm.levelOneCellVoltageTooHigh == true ? error_msg_ += "levelTwoCellVoltageTooHigh \n" : error_msg_;
    error_msg_ = alarm.levelOneCellVoltageTooLow  == true ? error_msg_ += "levelOneCellVoltageTooLow  \n" : error_msg_;
    error_msg_ = alarm.levelTwoCellVoltageTooLow  == true ? error_msg_ += "levelTwoCellVoltageTooLow  \n" : error_msg_;
    error_msg_ = alarm.levelOnePackVoltageTooHigh == true ? error_msg_ += "levelOnePackVoltageTooHigh \n" : error_msg_;
    error_msg_ = alarm.levelTwoPackVoltageTooHigh == true ? error_msg_ += "levelTwoPackVoltageTooHigh \n" : error_msg_;
    error_msg_ = alarm.levelOnePackVoltageTooLow  == true ? error_msg_ += "levelOnePackVoltageTooLow  \n" : error_msg_;
    error_msg_ = alarm.levelTwoPackVoltageTooLow  == true ? error_msg_ += "levelTwoPackVoltageTooLow  \n" : error_msg_;

    error_msg_ = alarm.levelOneChargeTempTooHigh    == true ? error_msg_ += "levelOneChargeTempTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelTwoChargeTempTooHigh    == true ? error_msg_ += "levelTwoChargeTempTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelOneChargeTempTooLow     == true ? error_msg_ += "levelOneChargeTempTooLow     \n " : error_msg_;
    error_msg_ = alarm.levelTwoChargeTempTooLow     == true ? error_msg_ += "levelTwoChargeTempTooLow     \n " : error_msg_;
    error_msg_ = alarm.levelOneDischargeTempTooHigh == true ? error_msg_ += "levelOneDischargeTempTooHigh \n"  : error_msg_;
    error_msg_ = alarm.levelTwoDischargeTempTooHigh == true ? error_msg_ += "levelTwoDischargeTempTooHigh \n"  : error_msg_;
    error_msg_ = alarm.levelOneDischargeTempTooLow  == true ? error_msg_ += "levelOneDischargeTempTooLow  \n " : error_msg_;
    error_msg_ = alarm.levelTwoDischargeTempTooLow  == true ? error_msg_ += "levelTwoDischargeTempTooLow  \n " : error_msg_;

    error_msg_ = alarm.levelOneChargeCurrentTooHigh    == true ? error_msg_ += "levelOneChargeCurrentTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelTwoChargeCurrentTooHigh    == true ? error_msg_ += "levelTwoChargeCurrentTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelOneDischargeCurrentTooHigh == true ? error_msg_ += "levelOneDischargeCurrentTooHigh \n " : error_msg_;
    error_msg_ = alarm.levelTwoDischargeCurrentTooHigh == true ? error_msg_ += "levelTwoDischargeCurrentTooHigh \n " : error_msg_;
    error_msg_ = alarm.levelOneStateOfChargeTooHigh    == true ? error_msg_ += "levelOneStateOfChargeTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelTwoStateOfChargeTooHigh    == true ? error_msg_ += "levelTwoStateOfChargeTooHigh    \n"  : error_msg_;
    error_msg_ = alarm.levelOneStateOfChargeTooLow     == true ? error_msg_ += "levelOneStateOfChargeTooLow     \n " : error_msg_;
    error_msg_ = alarm.levelTwoStateOfChargeTooLow     == true ? error_msg_ += "levelTwoStateOfChargeTooLow     \n " : error_msg_;

    error_msg_ = alarm.levelOneCellVoltageDifferenceTooHigh == true ? error_msg_ += "levelOneCellVoltageDifferenceTooHigh \n"  : error_msg_;
    error_msg_ = alarm.levelTwoCellVoltageDifferenceTooHigh == true ? error_msg_ += "levelTwoCellVoltageDifferenceTooHigh \n"  : error_msg_;
    error_msg_ = alarm.levelOneTempSensorDifferenceTooHigh  == true ? error_msg_ += "levelOneTempSensorDifferenceTooHigh  \n " : error_msg_;
    error_msg_ = alarm.levelTwoTempSensorDifferenceTooHigh  == true ? error_msg_ += "levelTwoTempSensorDifferenceTooHigh  \n " : error_msg_;

    error_msg_ = alarm.chargeFETTemperatureTooHigh            == true ? error_msg_ += "chargeFETTemperatureTooHigh            \n"  : error_msg_;
    error_msg_ = alarm.dischargeFETTemperatureTooHigh         == true ? error_msg_ += "dischargeFETTemperatureTooHigh         \n"  : error_msg_;
    error_msg_ = alarm.failureOfChargeFETTemperatureSensor    == true ? error_msg_ += "failureOfChargeFETTemperatureSensor    \n " : error_msg_;
    error_msg_ = alarm.failureOfDischargeFETTemperatureSensor == true ? error_msg_ += "failureOfDischargeFETTemperatureSensor \n " : error_msg_;
    error_msg_ = alarm.failureOfChargeFETAdhesion             == true ? error_msg_ += "failureOfChargeFETAdhesion             \n"  : error_msg_;
    error_msg_ = alarm.failureOfDischargeFETAdhesion          == true ? error_msg_ += "failureOfDischargeFETAdhesion          \n"  : error_msg_;
    error_msg_ = alarm.failureOfChargeFETTBreaker             == true ? error_msg_ += "failureOfChargeFETTBreaker             \n " : error_msg_;
    error_msg_ = alarm.failureOfDischargeFETBreaker           == true ? error_msg_ += "failureOfDischargeFETBreaker           \n " : error_msg_;

    error_msg_ = alarm.failureOfAFEAcquisitionModule        == true ? error_msg_ += "failureOfAFEAcquisitionModule        \n"  : error_msg_;
    error_msg_ = alarm.failureOfVoltageSensorModule         == true ? error_msg_ += "failureOfVoltageSensorModule         \n"  : error_msg_;
    error_msg_ = alarm.failureOfTemperatureSensorModule     == true ? error_msg_ += "failureOfTemperatureSensorModule     \n " : error_msg_;
    error_msg_ = alarm.failureOfEEPROMStorageModule         == true ? error_msg_ += "failureOfEEPROMStorageModule         \n " : error_msg_;
    error_msg_ = alarm.failureOfRealtimeClockModule         == true ? error_msg_ += "failureOfRealtimeClockModule         \n"  : error_msg_;
    error_msg_ = alarm.failureOfPrechargeModule             == true ? error_msg_ += "failureOfPrechargeModule             \n"  : error_msg_;
    error_msg_ = alarm.failureOfVehicleCommunicationModule  == true ? error_msg_ += "failureOfVehicleCommunicationModule  \n " : error_msg_;
    error_msg_ = alarm.failureOfIntranetCommunicationModule == true ? error_msg_ += "failureOfIntranetCommunicationModule \n " : error_msg_;

    error_msg_ = alarm.failureOfCurrentSensorModule     == true ? error_msg_ += "failureOfCurrentSensorModule     \n"  : error_msg_;
    error_msg_ = alarm.failureOfMainVoltageSensorModule == true ? error_msg_ += "failureOfMainVoltageSensorModule \n"  : error_msg_;
    error_msg_ = alarm.failureOfShortCircuitProtection  == true ? error_msg_ += "failureOfShortCircuitProtection  \n " : error_msg_;
    error_msg_ = alarm.failureOfLowVoltageNoCharging    == true ? error_msg_ += "failureOfLowVoltageNoCharging    \n " : error_msg_;
}

void RoverBatteryNode::publishBatteryInfo()
{
    battery_state_msgs_.voltage                 = get.packVoltage;
    battery_state_msgs_.temperature             = get.tempAverage;
    battery_state_msgs_.current                 = get.packCurrent;
    battery_state_msgs_.charge                  = get.packSOC;
    battery_state_msgs_.capacity                = get.resCapacitymAh;
    battery_state_msgs_.design_capacity         = kDesignedCapacity;
    battery_state_msgs_.percentage              = get.packSOC / 100.0f;
    battery_state_msgs_.power_supply_technology = sensor_msgs::msg::BatteryState::POWER_SUPPLY_TECHNOLOGY_LIFE;
    battery_state_msgs_.present                 = true;
    battery_state_msgs_.location                = "rover";
    battery_state_msgs_.serial_number           = kSerialNumber;
    
    // Check to make sure we have a valid number of cells
    battery_state_msgs_.cell_voltage.clear();

    for (int i = 0; i < get.numberOfCells; i++) {
        battery_state_msgs_.cell_voltage.push_back(get.cellVmV[i]);
    }
        
    battery_state_msgs_.cell_temperature.clear();

    for (int i = 0; i < get.numOfTempSensors; i++) {
        battery_state_msgs_.cell_temperature.push_back(get.cellTemperature[i]);
    }

    if (get.chargeDischargeStatus == 1) {
        if (battery_state_msgs_.percentage >= 1.0f) {
            battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_FULL;
        }
        else {
            battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_CHARGING;
        }
        
        charging_state_msgs_.charging           = true;
        charging_state_msgs_.current            = get.packCurrent;
        charging_state_msgs_.current_battery    = get.packCurrent;
        charging_state_msgs_.charger_type       = rover_msgs::msg::ChargingStatus::WIRED;
    }
    else if (get.chargeDischargeStatus == 2) {
        battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        charging_state_msgs_.charging           = false;
        charging_state_msgs_.current            = get.packCurrent;
        charging_state_msgs_.current_battery    = get.packCurrent;
        charging_state_msgs_.charger_type       = rover_msgs::msg::ChargingStatus::WIRED;
    }
    else if (get.chargeDischargeStatus == 0) {
        battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_NOT_CHARGING;
        charging_state_msgs_.charging           = false;
        charging_state_msgs_.current            = get.packCurrent;
        charging_state_msgs_.current_battery    = get.packCurrent;
        charging_state_msgs_.charger_type       = rover_msgs::msg::ChargingStatus::UNKNOWN;
    }
    else {
        battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_UNKNOWN;
        charging_state_msgs_.charging        = false;
        charging_state_msgs_.current         = get.packCurrent;
        charging_state_msgs_.current_battery = get.packCurrent;
        charging_state_msgs_.charger_type    = rover_msgs::msg::ChargingStatus::UNKNOWN;
    }

    battery_state_msgs_.power_supply_health = updateBatteryHealth();
    
    updateFailureCodes();

    // Publish state
    battery_publisher_->publish(battery_state_msgs_, charging_state_msgs_, error_msg_);
}

}; // namespace rover_battery