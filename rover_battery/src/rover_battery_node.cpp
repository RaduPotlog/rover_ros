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

using namespace mn;

namespace rover_battery
{

RoverBatteryNode::RoverBatteryNode(
    const std::string & node_name, 
    const std::string & ns, 
    const rclcpp::NodeOptions & options
)
: Node(node_name, ns, options)
, diagnostic_updater_(std::make_shared<diagnostic_updater::Updater>(this))
{
    this->declare_parameter("device", "/dev/ttyUSB0");
    this->declare_parameter("baudrate", 9600);
    this->declare_parameter("receiver_rate", 50);

    serial_device_name_ = this->get_parameter("device").as_string();
    int baudrate = this->get_parameter("baudrate").as_int();
    int rate = this->get_parameter("receiver_rate").as_int();
    int period = static_cast<int>((1.0f / static_cast<float>(rate) * 1000.0f));

    RCLCPP_INFO(this->get_logger(), "Rate is %dhz (period %dms)", rate, period);
    RCLCPP_INFO(this->get_logger(), "Target serial device is: %s", serial_device_name_.c_str());
    RCLCPP_INFO(this->get_logger(), "Selected baudrate: %d", baudrate);

    serial_port_.SetDevice(serial_device_name_);
    serial_port_.SetBaudRate(baudrate);
    serial_port_.SetTimeout(period / 2);

    battery_read_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(period), 
        std::bind(&RoverBatteryNode::batteryReadTimerCallback, this)
    );

    
    battery_read_timeout_ = this->create_wall_timer(
        std::chrono::milliseconds(1000), 
        std::bind(&RoverBatteryNode::batteryReadTimeoutCallback, this)
    );
    
    battery_read_timeout_->cancel();
}

void RoverBatteryNode::init() 
{
    diagnostic_updater_->setHardwareID("RoverBattery");

    battery_publisher_ = std::make_shared<SingleBatteryPublisher>(
        this->shared_from_this(), diagnostic_updater_, 2000);
}

void RoverBatteryNode::batteryReadTimerCallback()
{
    if (serial_port_.GetState() == CppLinuxSerial::State::CLOSED) {
        try {
            serial_port_.Open();
            
        } catch(const CppLinuxSerial::Exception& e) {
            RCLCPP_WARN(this->get_logger(), "Can not open serial port: %s", serial_device_name_.c_str());
            return;
        }
    }

    (void)updateBatteryStatus();
}

void RoverBatteryNode::batteryReadTimeoutCallback()
{
    battery_read_timeout_->cancel();
    rx_timeout_reached_ = true;
    battery_state_msgs_.power_supply_health = sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_WATCHDOG_TIMER_EXPIRE;
    battery_publisher_->publish(battery_state_msgs_, charging_state_msgs_, error_msg_);
    RCLCPP_WARN(this->get_logger(), "Battery response timeout");
}

bool RoverBatteryNode::updateBatteryStatus()
{
    ReturnStatus ret = E_NOK;

    switch(current_command_state_) {
        
        case CommandState::IDLE:
            // Start transfer    
            current_command_state_ = CommandState::TRANSMIT;
            // Reset command
            current_command_ = Command::VOUT_IOUT_SOC;
            break;

        case CommandState::TRANSMIT:
            
            if (!sendCommand(current_command_)) {
                // Reset transfer    
                current_command_state_ = CommandState::IDLE;
            }
            else {
                // Wait for rx    
                current_command_state_ = CommandState::WAIT_RX;
                rx_timeout_started_ = true;
                battery_read_timeout_->reset();
            }
            
            break;

        case CommandState::WAIT_RX:

            ret = parseAllIncomingCommandBytes(current_command_);

            if (ret == ReturnStatus::E_OK) {
                rx_timeout_started_ = false;
                battery_read_timeout_->cancel();
            
                // Switch command
                current_command_ = static_cast<Command>(static_cast<uint8_t>(current_command_) + 1U);
                
                if (current_command_ > Command::FAILURE_CODES) {
                    // All commands executed
                    current_command_state_ = CommandState::RECEIVED;
                }
                else {
                    current_command_state_ = CommandState::TRANSMIT;
                }
            }
            else if (ret == ReturnStatus::E_NOK) {
                // Reset state
                current_command_state_ = CommandState::IDLE;
                rx_timeout_started_ = false;
                battery_read_timeout_->cancel();
            }
            else {
               current_command_state_ = CommandState::WAIT_RX;
            }

            if (rx_timeout_reached_ == true) {
                current_command_state_ = CommandState::TIMEOUT;
                battery_read_timeout_->cancel();
            }

            break;
    
        case CommandState::RECEIVED:
            // Reset state
            current_command_state_ = CommandState::TRANSMIT;
            // Reset command
            current_command_ = Command::VOUT_IOUT_SOC;
            publishBatteryInfo();
            break;

        case CommandState::TIMEOUT:
            // Reset state
            current_command_state_ = CommandState::TRANSMIT;
            rx_timeout_started_ = false;
            rx_timeout_reached_ = false;
            // Reset command
            current_command_ = Command::VOUT_IOUT_SOC;
            break;

        default:
            // Reset
            current_command_state_ = CommandState::IDLE;
            break;
    }

    return true;
}

uint8_t RoverBatteryNode::evalChecksum(std::vector<uint8_t> & buff)
{
    uint8_t checksum = 0U;
    
    for (size_t i = 0; i < buff.size() - 1; i++) {
        checksum += buff[i];
    }
    
    return checksum;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::checkRxBuffer()
{
    if (serial_port_.Available()) {
        rx_buffer_.clear();
        serial_port_.ReadBinary(rx_buffer_);
        
        if (rx_buffer_.at(0) != 0xA5) {
            RCLCPP_WARN(this->get_logger(), "Invalid start byte");
            return ReturnStatus::E_NOK;
        }

        if (rx_buffer_.at(1) != 0x01) {
            RCLCPP_WARN(this->get_logger(), "Invalid slave");
            return ReturnStatus::E_NOK;
        }
    }
    else {
        return ReturnStatus::E_BUSY;
    }

    return ReturnStatus::E_OK;
}

bool RoverBatteryNode::sendCommand(Command cmd_id)
{
    // Claear rx buffer
    if (serial_port_.Available()) {
        rx_buffer_.clear();
        serial_port_.ReadBinary(rx_buffer_);
        rx_buffer_.clear();
    }

    tx_buffer_.clear();
    tx_buffer_.push_back(0xA5U);
    tx_buffer_.push_back(0x40U);
    tx_buffer_.push_back(static_cast<uint8_t>(cmd_id));
    tx_buffer_.push_back(0x08U);
    
    if ((cmd_id != DISCHRG_FET) && (cmd_id != CHRG_FET) && (cmd_id != BMS_RESET)) {
        tx_buffer_.push_back(0x00U);
    }
    
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);
    tx_buffer_.push_back(0x00U);

    uint8_t checksum = evalChecksum(tx_buffer_);
    tx_buffer_.push_back(checksum);
    // Send command to BMS
    serial_port_.WriteBinary(tx_buffer_);
    
    return true;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::parseAllIncomingCommandBytes(Command cmd_id) 
{    
        switch(cmd_id) {
            case VOUT_IOUT_SOC:
                return getPackMeasurements();               // 0x90
                break;

            case MIN_MAX_CELL_VOLTAGE:
                return getMinMaxCellVoltage();              // 0x91
                break;
            
            case MIN_MAX_TEMPERATURE:
                return getPackTemp();                       // 0x92
                break;
            
            case DISCHARGE_CHARGE_MOS_STATUS:
                return getDischargeChargeMosStatus();       // 0x93
                break;
            
            case STATUS_INFO:
                return getStatusInfo();                     // 0x94
                break;

            case CELL_VOLTAGES:
                return getCellVoltages();                   // 0x95
                break;
            
            case CELL_TEMPERATURE:
                return getCellTemperature();                // 0x96
                break;

            case CELL_BALANCE_STATE:
                return getCellBalanceState();               // 0x97
                break;
            
            case FAILURE_CODES:
                return getFailureCodes();                   // 0x98
                break;
            
            default:
                RCLCPP_WARN(this->get_logger(), "Invalid command");
                break;
        }

        return ReturnStatus::E_NOK;
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
    if (get.numberOfCells >= MIN_NUMBER_CELLS && get.numberOfCells <= MAX_NUMBER_CELLS) {
        
        battery_state_msgs_.cell_voltage.clear();

        for (int i = 0; i < get.numberOfCells; i++) {
            battery_state_msgs_.cell_voltage.push_back(get.cellVmV[i]);
        }
    }

    if (get.numOfTempSensors >= MIN_NUMBER_TEMP_SENSORS && get.numOfTempSensors <= MAX_NUMBER_TEMP_SENSORS) {
        
        battery_state_msgs_.cell_temperature.clear();

        for (int i = 0; i < get.numOfTempSensors; i++) {
            battery_state_msgs_.cell_temperature.push_back(get.cellTemperature[i]);
        }
    }

    if (get.chargeDischargeStatus == "Charge") {
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
    else if (get.chargeDischargeStatus == "Discharge") {
        battery_state_msgs_.power_supply_status = sensor_msgs::msg::BatteryState::POWER_SUPPLY_STATUS_DISCHARGING;
        charging_state_msgs_.charging           = false;
        charging_state_msgs_.current            = get.packCurrent;
        charging_state_msgs_.current_battery    = get.packCurrent;
        charging_state_msgs_.charger_type       = rover_msgs::msg::ChargingStatus::WIRED;
    }
    else if (get.chargeDischargeStatus == "Stationary") {
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

    battery_state_msgs_.power_supply_health = getBatteryHealth();
    
    // Publish state
    battery_publisher_->publish(battery_state_msgs_, charging_state_msgs_, error_msg_);
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getPackMeasurements() // 0x90
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        uint8_t checksum = evalChecksum(rx_buffer_);

        if (checksum != rx_buffer_.at(rx_buffer_.size() - 1)) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum");
            return ReturnStatus::E_NOK;
        }
        
        // Pull the relevant values out of the buffer
        get.packVoltage = (static_cast<float>((this->rx_buffer_[4] << 8) | this->rx_buffer_[5])) / 10.0f;
        // The current measurement is given with a 30000 unit offset (see /docs/)
        get.packCurrent = (static_cast<float>((this->rx_buffer_[8] << 8) | this->rx_buffer_[9]) - 30000) / 10.0f;
        get.packSOC = (static_cast<float>((this->rx_buffer_[10] << 8) | this->rx_buffer_[11]) / 10.0f);
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getMinMaxCellVoltage() // 0x91
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        uint8_t checksum = evalChecksum(rx_buffer_);

        if (checksum != rx_buffer_.at(rx_buffer_.size() - 1)) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum");
            return ReturnStatus::E_NOK;
        }
        
        get.maxCellmV = (float)((this->rx_buffer_[4] << 8) | this->rx_buffer_[5]);
        get.maxCellVNum = this->rx_buffer_[6];
        get.minCellmV = (float)((this->rx_buffer_[7] << 8) | this->rx_buffer_[8]);
        get.minCellVNum = this->rx_buffer_[9];
        get.cellDiff = (get.maxCellmV - get.minCellmV);
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getPackTemp() // 0x92
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        uint8_t checksum = evalChecksum(rx_buffer_);

        if (checksum != rx_buffer_.at(rx_buffer_.size() - 1)) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum");
            return ReturnStatus::E_NOK;
        }

        // An offset of 40 is added by the BMS to avoid having to deal with negative numbers, see protocol in /docs/
        get.tempMax = (this->rx_buffer_[4] - 40);
        get.tempMin = (this->rx_buffer_[6] - 40);
        get.tempAverage = (get.tempMax + get.tempMin) / 2;
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getDischargeChargeMosStatus() // 0x93
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        uint8_t checksum = evalChecksum(rx_buffer_);

        if (checksum != rx_buffer_.at(rx_buffer_.size() - 1)) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum");
            return ReturnStatus::E_NOK;
        }

        switch (this->rx_buffer_[4])
        {
            case 0:
                get.chargeDischargeStatus = "Stationary";
                break;
            case 1:
                get.chargeDischargeStatus = "Charge";
                break;
            case 2:
                get.chargeDischargeStatus = "Discharge";
                break;
        }

        get.chargeFetState = this->rx_buffer_[5];
        get.disChargeFetState = this->rx_buffer_[6];
        get.bmsHeartBeat = this->rx_buffer_[7];
        get.resCapacitymAh = ((uint32_t)rx_buffer_[8] << 0x18) | ((uint32_t)rx_buffer_[9] << 0x10) | \
                            ((uint32_t)rx_buffer_[10] << 0x08) | (uint32_t)rx_buffer_[11];
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getStatusInfo() // 0x94
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        uint8_t checksum = evalChecksum(rx_buffer_);

        if (checksum != rx_buffer_.at(rx_buffer_.size() - 1)) {
            RCLCPP_WARN(this->get_logger(), "Invalid checksum");
            return ReturnStatus::E_NOK;
        }

        get.numberOfCells = this->rx_buffer_[4];
        get.numOfTempSensors = this->rx_buffer_[5];
        get.chargeState = this->rx_buffer_[6];
        get.loadState = this->rx_buffer_[7];

        // Parse the 8 bits into 8 booleans that represent the states of the Digital IO
        for (size_t i = 0; i < 8; i++) {
            get.dIO[i] = bitRead(this->rx_buffer_[8], i);
        }

        get.bmsCycles = ((uint16_t)this->rx_buffer_[9] << 0x08) | (uint16_t)this->rx_buffer_[10];
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getCellVoltages() // 0x95
{
    ReturnStatus ret = checkRxBuffer();
    
    if (ret == ReturnStatus::E_OK) {
    
        int cellNo = 0;

        // Check to make sure we have a valid number of cells
        if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS) {
            return ReturnStatus::E_NOK;
        }

        for (size_t i = 0; i <= ceil(get.numberOfCells / 3); i++) {
            for (size_t i = 0; i <= ceil(get.numberOfCells / 3); i++) {
                for (size_t i = 0; i < 3; i++) {
                    get.cellVmV[cellNo] = (this->rx_buffer_[5 + i + i] << 8) | this->rx_buffer_[6 + i + i];
                    cellNo++;
                    if (cellNo >= get.numberOfCells) break;
                }
            }
        }
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getCellTemperature() // 0x96
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        int sensorNo = 0;

        // Check to make sure we have a valid number of temp sensors
        if ((get.numOfTempSensors < MIN_NUMBER_TEMP_SENSORS) && (get.numOfTempSensors >= MAX_NUMBER_TEMP_SENSORS)) {
            return ReturnStatus::E_NOK;
        }

        for (size_t i = 0; i <= ceil(get.numOfTempSensors / 7); i++) {
            for (size_t j = 0; j < 7; j++) {

                get.cellTemperature[sensorNo] = (this->rx_buffer_[5 + j] - 40);
                sensorNo++;

                if (sensorNo + 1 > get.numOfTempSensors) break;
            }
        }
    }

    return ret;
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getCellBalanceState() // 0x97
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {
    
        int cellBalance = 0;
        int cellBit = 0;

        // Check to make sure we have a valid number of cells
        if (get.numberOfCells < MIN_NUMBER_CELLS && get.numberOfCells >= MAX_NUMBER_CELLS)
        {
            return ReturnStatus::E_NOK;
        }

        // We expect 6 bytes response for this command
        for (size_t i = 0; i < 6; i++) {
            // For each bit in the byte, pull out the cell balance state boolean
            for (size_t j = 0; j < 8; j++) {
                get.cellBalanceState[cellBit] = bitRead(this->rx_buffer_[i + 4], j);
                cellBit++;

                if (bitRead(this->rx_buffer_[i + 4], j)) {
                    cellBalance++;
                }
                if (cellBit >= 47) {
                    break;
                }
            }
        }

        if (cellBalance > 0) {
            get.cellBalanceActive = ReturnStatus::E_OK;
        }
        else {
            get.cellBalanceActive = ReturnStatus::E_NOK;
        }
    }
    
    return ret;
}

std::uint8_t RoverBatteryNode::getBatteryHealth()
{
    if ((alarm.levelOnePackVoltageTooLow == true) || (alarm.levelOneCellVoltageTooLow == true) || 
        (alarm.levelOneStateOfChargeTooLow == true)) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_DEAD;
    } else if ((alarm.levelTwoPackVoltageTooHigh == true) || //(alarm.levelTwoCellVoltageTooHigh == true) ||
        (alarm.levelTwoStateOfChargeTooHigh == true)) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERVOLTAGE;
    } else if (alarm.levelOneChargeTempTooHigh == true || alarm.levelOneDischargeTempTooHigh == true) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_OVERHEAT;
    } else if (alarm.levelOneChargeTempTooLow == true || alarm.levelOneDischargeTempTooLow == true) {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_COLD;
    } else {
        return sensor_msgs::msg::BatteryState::POWER_SUPPLY_HEALTH_GOOD;
    }
}

RoverBatteryNode::ReturnStatus RoverBatteryNode::getFailureCodes() // 0x98
{
    ReturnStatus ret = checkRxBuffer();

    if (ret == ReturnStatus::E_OK) {

        error_msg_ = "";

        /* 0x00 */
        alarm.levelOneCellVoltageTooHigh = bitRead(this->rx_buffer_[4], 0);
        alarm.levelTwoCellVoltageTooHigh = bitRead(this->rx_buffer_[4], 1);
        alarm.levelOneCellVoltageTooLow  = bitRead(this->rx_buffer_[4], 2);
        alarm.levelTwoCellVoltageTooLow  = bitRead(this->rx_buffer_[4], 3);
        alarm.levelOnePackVoltageTooHigh = bitRead(this->rx_buffer_[4], 4);
        alarm.levelTwoPackVoltageTooHigh = bitRead(this->rx_buffer_[4], 5);
        alarm.levelOnePackVoltageTooLow  = bitRead(this->rx_buffer_[4], 6);
        alarm.levelTwoPackVoltageTooLow  = bitRead(this->rx_buffer_[4], 7);

        error_msg_ = alarm.levelOneCellVoltageTooHigh == true ? error_msg_ += "levelOneCellVoltageTooHigh \n" : error_msg_;
        error_msg_ = alarm.levelOneCellVoltageTooHigh == true ? error_msg_ += "levelTwoCellVoltageTooHigh \n" : error_msg_;
        error_msg_ = alarm.levelOneCellVoltageTooLow  == true ? error_msg_ += "levelOneCellVoltageTooLow  \n" : error_msg_;
        error_msg_ = alarm.levelTwoCellVoltageTooLow  == true ? error_msg_ += "levelTwoCellVoltageTooLow  \n" : error_msg_;
        error_msg_ = alarm.levelOnePackVoltageTooHigh == true ? error_msg_ += "levelOnePackVoltageTooHigh \n" : error_msg_;
        error_msg_ = alarm.levelTwoPackVoltageTooHigh == true ? error_msg_ += "levelTwoPackVoltageTooHigh \n" : error_msg_;
        error_msg_ = alarm.levelOnePackVoltageTooLow  == true ? error_msg_ += "levelOnePackVoltageTooLow  \n" : error_msg_;
        error_msg_ = alarm.levelTwoPackVoltageTooLow  == true ? error_msg_ += "levelTwoPackVoltageTooLow  \n" : error_msg_;

        /* 0x01 */
        alarm.levelOneChargeTempTooHigh    = bitRead(this->rx_buffer_[5], 1);
        alarm.levelTwoChargeTempTooHigh    = bitRead(this->rx_buffer_[5], 1);
        alarm.levelOneChargeTempTooLow     = bitRead(this->rx_buffer_[5], 1);
        alarm.levelTwoChargeTempTooLow     = bitRead(this->rx_buffer_[5], 1);
        alarm.levelOneDischargeTempTooHigh = bitRead(this->rx_buffer_[5], 1);
        alarm.levelTwoDischargeTempTooHigh = bitRead(this->rx_buffer_[5], 1);
        alarm.levelOneDischargeTempTooLow  = bitRead(this->rx_buffer_[5], 1);
        alarm.levelTwoDischargeTempTooLow  = bitRead(this->rx_buffer_[5], 1);

        error_msg_ = alarm.levelOneChargeTempTooHigh    == true ? error_msg_ += "levelOneChargeTempTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelTwoChargeTempTooHigh    == true ? error_msg_ += "levelTwoChargeTempTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelOneChargeTempTooLow     == true ? error_msg_ += "levelOneChargeTempTooLow     \n " : error_msg_;
        error_msg_ = alarm.levelTwoChargeTempTooLow     == true ? error_msg_ += "levelTwoChargeTempTooLow     \n " : error_msg_;
        error_msg_ = alarm.levelOneDischargeTempTooHigh == true ? error_msg_ += "levelOneDischargeTempTooHigh \n"  : error_msg_;
        error_msg_ = alarm.levelTwoDischargeTempTooHigh == true ? error_msg_ += "levelTwoDischargeTempTooHigh \n"  : error_msg_;
        error_msg_ = alarm.levelOneDischargeTempTooLow  == true ? error_msg_ += "levelOneDischargeTempTooLow  \n " : error_msg_;
        error_msg_ = alarm.levelTwoDischargeTempTooLow  == true ? error_msg_ += "levelTwoDischargeTempTooLow  \n " : error_msg_;

        /* 0x02 */
        alarm.levelOneChargeCurrentTooHigh    = bitRead(this->rx_buffer_[6], 0);
        alarm.levelTwoChargeCurrentTooHigh    = bitRead(this->rx_buffer_[6], 1);
        alarm.levelOneDischargeCurrentTooHigh = bitRead(this->rx_buffer_[6], 2);
        alarm.levelTwoDischargeCurrentTooHigh = bitRead(this->rx_buffer_[6], 3);
        alarm.levelOneStateOfChargeTooHigh    = bitRead(this->rx_buffer_[6], 4);
        alarm.levelTwoStateOfChargeTooHigh    = bitRead(this->rx_buffer_[6], 5);
        alarm.levelOneStateOfChargeTooLow     = bitRead(this->rx_buffer_[6], 6);
        alarm.levelTwoStateOfChargeTooLow     = bitRead(this->rx_buffer_[6], 7);

        error_msg_ = alarm.levelOneChargeCurrentTooHigh    == true ? error_msg_ += "levelOneChargeCurrentTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelTwoChargeCurrentTooHigh    == true ? error_msg_ += "levelTwoChargeCurrentTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelOneDischargeCurrentTooHigh == true ? error_msg_ += "levelOneDischargeCurrentTooHigh \n " : error_msg_;
        error_msg_ = alarm.levelTwoDischargeCurrentTooHigh == true ? error_msg_ += "levelTwoDischargeCurrentTooHigh \n " : error_msg_;
        error_msg_ = alarm.levelOneStateOfChargeTooHigh    == true ? error_msg_ += "levelOneStateOfChargeTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelTwoStateOfChargeTooHigh    == true ? error_msg_ += "levelTwoStateOfChargeTooHigh    \n"  : error_msg_;
        error_msg_ = alarm.levelOneStateOfChargeTooLow     == true ? error_msg_ += "levelOneStateOfChargeTooLow     \n " : error_msg_;
        error_msg_ = alarm.levelTwoStateOfChargeTooLow     == true ? error_msg_ += "levelTwoStateOfChargeTooLow     \n " : error_msg_;

        /* 0x03 */
        alarm.levelOneCellVoltageDifferenceTooHigh = bitRead(this->rx_buffer_[7], 0);
        alarm.levelTwoCellVoltageDifferenceTooHigh = bitRead(this->rx_buffer_[7], 1);
        alarm.levelOneTempSensorDifferenceTooHigh  = bitRead(this->rx_buffer_[7], 2);
        alarm.levelTwoTempSensorDifferenceTooHigh  = bitRead(this->rx_buffer_[7], 3);

        error_msg_ = alarm.levelOneCellVoltageDifferenceTooHigh == true ? error_msg_ += "levelOneCellVoltageDifferenceTooHigh \n"  : error_msg_;
        error_msg_ = alarm.levelTwoCellVoltageDifferenceTooHigh == true ? error_msg_ += "levelTwoCellVoltageDifferenceTooHigh \n"  : error_msg_;
        error_msg_ = alarm.levelOneTempSensorDifferenceTooHigh  == true ? error_msg_ += "levelOneTempSensorDifferenceTooHigh  \n " : error_msg_;
        error_msg_ = alarm.levelTwoTempSensorDifferenceTooHigh  == true ? error_msg_ += "levelTwoTempSensorDifferenceTooHigh  \n " : error_msg_;
        
        /* 0x04 */
        alarm.chargeFETTemperatureTooHigh            = bitRead(this->rx_buffer_[8], 0);
        alarm.dischargeFETTemperatureTooHigh         = bitRead(this->rx_buffer_[8], 1);
        alarm.failureOfChargeFETTemperatureSensor    = bitRead(this->rx_buffer_[8], 2);
        alarm.failureOfDischargeFETTemperatureSensor = bitRead(this->rx_buffer_[8], 3);
        alarm.failureOfChargeFETAdhesion             = bitRead(this->rx_buffer_[8], 4);
        alarm.failureOfDischargeFETAdhesion          = bitRead(this->rx_buffer_[8], 5);
        alarm.failureOfChargeFETTBreaker             = bitRead(this->rx_buffer_[8], 6);
        alarm.failureOfDischargeFETBreaker           = bitRead(this->rx_buffer_[8], 7);

        error_msg_ = alarm.chargeFETTemperatureTooHigh            == true ? error_msg_ += "chargeFETTemperatureTooHigh            \n"  : error_msg_;
        error_msg_ = alarm.dischargeFETTemperatureTooHigh         == true ? error_msg_ += "dischargeFETTemperatureTooHigh         \n"  : error_msg_;
        error_msg_ = alarm.failureOfChargeFETTemperatureSensor    == true ? error_msg_ += "failureOfChargeFETTemperatureSensor    \n " : error_msg_;
        error_msg_ = alarm.failureOfDischargeFETTemperatureSensor == true ? error_msg_ += "failureOfDischargeFETTemperatureSensor \n " : error_msg_;
        error_msg_ = alarm.failureOfChargeFETAdhesion             == true ? error_msg_ += "failureOfChargeFETAdhesion             \n"  : error_msg_;
        error_msg_ = alarm.failureOfDischargeFETAdhesion          == true ? error_msg_ += "failureOfDischargeFETAdhesion          \n"  : error_msg_;
        error_msg_ = alarm.failureOfChargeFETTBreaker             == true ? error_msg_ += "failureOfChargeFETTBreaker             \n " : error_msg_;
        error_msg_ = alarm.failureOfDischargeFETBreaker           == true ? error_msg_ += "failureOfDischargeFETBreaker           \n " : error_msg_;

        /* 0x05 */
        alarm.failureOfAFEAcquisitionModule        = bitRead(this->rx_buffer_[9], 0);
        alarm.failureOfVoltageSensorModule         = bitRead(this->rx_buffer_[9], 1);
        alarm.failureOfTemperatureSensorModule     = bitRead(this->rx_buffer_[9], 2);
        alarm.failureOfEEPROMStorageModule         = bitRead(this->rx_buffer_[9], 3);
        alarm.failureOfRealtimeClockModule         = bitRead(this->rx_buffer_[9], 4);
        alarm.failureOfPrechargeModule             = bitRead(this->rx_buffer_[9], 5);
        alarm.failureOfVehicleCommunicationModule  = bitRead(this->rx_buffer_[9], 6);
        alarm.failureOfIntranetCommunicationModule = bitRead(this->rx_buffer_[9], 7);

        error_msg_ = alarm.failureOfAFEAcquisitionModule        == true ? error_msg_ += "failureOfAFEAcquisitionModule        \n"  : error_msg_;
        error_msg_ = alarm.failureOfVoltageSensorModule         == true ? error_msg_ += "failureOfVoltageSensorModule         \n"  : error_msg_;
        error_msg_ = alarm.failureOfTemperatureSensorModule     == true ? error_msg_ += "failureOfTemperatureSensorModule     \n " : error_msg_;
        error_msg_ = alarm.failureOfEEPROMStorageModule         == true ? error_msg_ += "failureOfEEPROMStorageModule         \n " : error_msg_;
        error_msg_ = alarm.failureOfRealtimeClockModule         == true ? error_msg_ += "failureOfRealtimeClockModule         \n"  : error_msg_;
        error_msg_ = alarm.failureOfPrechargeModule             == true ? error_msg_ += "failureOfPrechargeModule             \n"  : error_msg_;
        error_msg_ = alarm.failureOfVehicleCommunicationModule  == true ? error_msg_ += "failureOfVehicleCommunicationModule  \n " : error_msg_;
        error_msg_ = alarm.failureOfIntranetCommunicationModule == true ? error_msg_ += "failureOfIntranetCommunicationModule \n " : error_msg_;

        /* 0x06 */
        alarm.failureOfCurrentSensorModule     = bitRead(this->rx_buffer_[10], 0);
        alarm.failureOfMainVoltageSensorModule = bitRead(this->rx_buffer_[10], 1);
        alarm.failureOfShortCircuitProtection  = bitRead(this->rx_buffer_[10], 2);
        alarm.failureOfLowVoltageNoCharging    = bitRead(this->rx_buffer_[10], 3);

        error_msg_ = alarm.failureOfCurrentSensorModule     == true ? error_msg_ += "failureOfCurrentSensorModule     \n"  : error_msg_;
        error_msg_ = alarm.failureOfMainVoltageSensorModule == true ? error_msg_ += "failureOfMainVoltageSensorModule \n"  : error_msg_;
        error_msg_ = alarm.failureOfShortCircuitProtection  == true ? error_msg_ += "failureOfShortCircuitProtection  \n " : error_msg_;
        error_msg_ = alarm.failureOfLowVoltageNoCharging    == true ? error_msg_ += "failureOfLowVoltageNoCharging    \n " : error_msg_;
    }
    
    return ret;
}

bool RoverBatteryNode::setDischargeMOS(bool sw) // 0xD9 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw) {
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->tx_buffer_[4] = 0x01;
        this->sendCommand( Command::DISCHRG_FET);
        // Clear the buffer for further use
        this->tx_buffer_[4] = 0x00;
    }
    else {
        this->sendCommand(Command::DISCHRG_FET);
    }

    if (E_NOK == this->checkRxBuffer()) {
        return false;
    }

    return true;
}

bool RoverBatteryNode::setChargeMOS(bool sw) // 0xDA 0x80 First Byte 0x01=ON 0x00=OFF
{
    if (sw == true) {
        // Set the first byte of the data payload to 1, indicating that we want to switch on the MOSFET
        this->tx_buffer_[4] = 0x01;
        this->sendCommand(Command::CHRG_FET);
        // Clear the buffer for further use
        this->tx_buffer_[4] = 0x00;
    }
    else {
        this->sendCommand(Command::CHRG_FET);
    }

    if (E_NOK == this->checkRxBuffer()) {
        return false;
    }

    return true;
}

bool RoverBatteryNode::batteryReset() // 0x00 Reset the BMS
{
    this->sendCommand(Command::BMS_RESET);

    if (E_NOK == this->checkRxBuffer()) {
        return false;
    }

    return true;
}

}; // namespace rover_battery