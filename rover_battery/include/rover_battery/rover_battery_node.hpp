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

#ifndef ROVER_BATTERY_ROVER_BATTERY_NODE_HPP_
#define ROVER_BATTERY_ROVER_BATTERY_NODE_HPP_

#include <chrono>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "rover_battery/battery_publisher/battery_publisher.hpp"
#include "rover_battery/battery_publisher/single_battery_publisher.hpp"

#include <udp_msgs/msg/udp_packet.hpp>

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

namespace rover_battery
{

class RoverBatteryNode : public rclcpp::Node
{

public:

    /**
     * @brief get struct holds all the data collected from the BMS and is populated using the update() API
     * @details Comments specify precision and units where applicable
     */
    struct __attribute__((packed)) Data
    {
        // data from 0x90
        float packVoltage; // Total pack voltage (0.1 V)
        float packCurrent; // Current in (+) or out (-) of pack (0.1 A)
        float packSOC;     // State Of Charge

        // data from 0x91
        float maxCellmV; // Maximum cell voltage (mV)
        int maxCellVNum; // Number of cell with highest voltage
        float minCellmV; // Minimum cell voltage (mV)
        int minCellVNum; // Number of cell with lowest voltage
        float cellDiff;  // Difference between min and max cell voltages

        // data from 0x92
        int tempMax;       // Maximum temperature sensor reading (°C)
        int tempMin;       // Minimum temperature sensor reading (°C)
        float tempAverage; // Average of temp sensors

        // data from 0x93
        int chargeDischargeStatus;    // charge/discharge status (0 stationary, 1 charge, 2 discharge)
        bool chargeFetState;          // charging MOSFET status
        bool disChargeFetState;       // discharge MOSFET state
        int bmsHeartBeat;             // BMS life (0~255 cycles)?
        int resCapacitymAh;           // residual capacity mAH

        // data from 0x94
        int numberOfCells;    // Cell count
        int numOfTempSensors; // Temp sensor count
        bool chargeState;     // charger status 0 = disconnected 1 = connected
        bool loadState;       // Load Status 0=disconnected 1=connected
        bool dIO[8];          // No information about this
        int bmsCycles;        // charge / discharge cycles

        // data from 0x95
        float cellVmV[48]; // Store Cell Voltages (mV)

        // data from 0x96
        int cellTemperature[16]; // array of cell Temperature sensors

        // data from 0x97
        bool cellBalanceState[48]; // bool array of cell balance states
        bool cellBalanceActive;    // bool is cell balance active
    } get;

    /**
     * @brief alarm struct holds booleans corresponding to all the possible alarms
     * (aka errors/warnings) the BMS can report
     */
    struct __attribute__((packed)) Alarm {
        /* 0x00 */
        uint8_t levelOneCellVoltageTooHigh   : 1;
        uint8_t levelTwoCellVoltageTooHigh   : 1;
        uint8_t levelOneCellVoltageTooLow    : 1;
        uint8_t levelTwoCellVoltageTooLow    : 1;
        uint8_t levelOnePackVoltageTooHigh   : 1;
        uint8_t levelTwoPackVoltageTooHigh   : 1;
        uint8_t levelOnePackVoltageTooLow    : 1;
        uint8_t levelTwoPackVoltageTooLow    : 1;

        /* 0x01 */
        uint8_t levelOneChargeTempTooHigh    : 1;
        uint8_t levelTwoChargeTempTooHigh    : 1;
        uint8_t levelOneChargeTempTooLow     : 1;
        uint8_t levelTwoChargeTempTooLow     : 1;
        uint8_t levelOneDischargeTempTooHigh : 1;
        uint8_t levelTwoDischargeTempTooHigh : 1;
        uint8_t levelOneDischargeTempTooLow  : 1;
        uint8_t levelTwoDischargeTempTooLow  : 1;

        /* 0x02 */
        uint8_t levelOneChargeCurrentTooHigh    : 1;
        uint8_t levelTwoChargeCurrentTooHigh    : 1;
        uint8_t levelOneDischargeCurrentTooHigh : 1;
        uint8_t levelTwoDischargeCurrentTooHigh : 1;
        uint8_t levelOneStateOfChargeTooHigh    : 1;
        uint8_t levelTwoStateOfChargeTooHigh    : 1;
        uint8_t levelOneStateOfChargeTooLow     : 1;
        uint8_t levelTwoStateOfChargeTooLow     : 1;

        /* 0x03 */
        uint8_t levelOneCellVoltageDifferenceTooHigh : 1;
        uint8_t levelTwoCellVoltageDifferenceTooHigh : 1;
        uint8_t levelOneTempSensorDifferenceTooHigh  : 1;
        uint8_t levelTwoTempSensorDifferenceTooHigh  : 1;
        uint8_t : 4; // Padding bits to finish the 0x03 byte boundary

        /* 0x04 */
        uint8_t chargeFETTemperatureTooHigh            : 1;
        uint8_t dischargeFETTemperatureTooHigh         : 1;
        uint8_t failureOfChargeFETTemperatureSensor    : 1;
        uint8_t failureOfDischargeFETTemperatureSensor : 1;
        uint8_t failureOfChargeFETAdhesion             : 1;
        uint8_t failureOfDischargeFETAdhesion          : 1;
        uint8_t failureOfChargeFETTBreaker             : 1;
        uint8_t failureOfDischargeFETBreaker           : 1;

        /* 0x05 */
        uint8_t failureOfAFEAcquisitionModule        : 1;
        uint8_t failureOfVoltageSensorModule         : 1;
        uint8_t failureOfTemperatureSensorModule     : 1;
        uint8_t failureOfEEPROMStorageModule         : 1;
        uint8_t failureOfRealtimeClockModule         : 1;
        uint8_t failureOfPrechargeModule             : 1;
        uint8_t failureOfVehicleCommunicationModule   : 1;
        uint8_t failureOfIntranetCommunicationModule  : 1;

        /* 0x06 */
        uint8_t failureOfCurrentSensorModule     : 1;
        uint8_t failureOfMainVoltageSensorModule : 1;
        uint8_t failureOfShortCircuitProtection  : 1;
        uint8_t failureOfLowVoltageNoCharging    : 1;
        uint8_t : 4; // Padding bits to finish the 0x06 byte boundary
    } alarm;

    RoverBatteryNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    void init();
    
private:

    static constexpr float kDesignedCapacity = 40.0;
    static constexpr std::string_view kSerialNumber = "224KA141600043";

    void batteryUdpDataCallback(const udp_msgs::msg::UdpPacket::SharedPtr msg);

    void batteryUdpDataSubsciberTimeoutCallback();
    
    std::uint8_t updateBatteryHealth();

    void updateFailureCodes();
    
    void publishBatteryInfo();

    rclcpp::TimerBase::SharedPtr battery_read_timeout_;
    
    BatteryStateMsg battery_state_msgs_;
    ChargingStatusMsg charging_state_msgs_;
    std::string error_msg_;
    
    std::shared_ptr<BatteryPublisher> battery_publisher_;

    rclcpp::Subscription<udp_msgs::msg::UdpPacket>::SharedPtr battery_subscriber_;
    
    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_battery

#endif  // ROVER_BATTERY_ROVER_BATTERY_NODE_HPP_