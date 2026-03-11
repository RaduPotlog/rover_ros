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

#include "diagnostic_updater/diagnostic_updater.hpp"
#include "rclcpp/rclcpp.hpp"

#include <CppLinuxSerial/SerialPort.hpp>

using namespace std::chrono_literals;
using namespace mn;

#define bitRead(value, bit) (((value) >> (bit)) & 0x01)

#define XFER_BUFFER_LENGTH          13
#define MIN_NUMBER_CELLS            1
#define MAX_NUMBER_CELLS            8
#define MIN_NUMBER_TEMP_SENSORS     1
#define MAX_NUMBER_TEMP_SENSORS     2

namespace rover_battery
{

class RoverBatteryNode : public rclcpp::Node
{

public:

    enum ReturnStatus
    {
        E_NOK   = 0x00U,
        E_OK    = 0x01U,
        E_BUSY  = 0x03U,
    };

    enum CommandState
    {
        IDLE     = 0x00U,
        TRANSMIT = 0x01U,
        WAIT_RX  = 0x02U,
        RECEIVED = 0x03U,
        TIMEOUT  = 0xFFU,
    };

    enum Command : uint8_t
    {
        BMS_RESET                   = 0x00U,
        VOUT_IOUT_SOC               = 0x90U,
        MIN_MAX_CELL_VOLTAGE        = 0x91U,
        MIN_MAX_TEMPERATURE         = 0x92U,
        DISCHARGE_CHARGE_MOS_STATUS = 0x93U,
        STATUS_INFO                 = 0x94U,
        CELL_VOLTAGES               = 0x95U,
        CELL_TEMPERATURE            = 0x96U,
        CELL_BALANCE_STATE          = 0x97U,
        FAILURE_CODES               = 0x98U,
        DISCHRG_FET                 = 0xD9U,
        CHRG_FET                    = 0xDAU,
       
        END_OF_ENUM                 = 0xFFU,
    };

    /**
     * @brief get struct holds all the data collected from the BMS and is populated using the update() API
     * @details Comments specify precision and units where applicable
     */
    struct
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
        std::string chargeDischargeStatus; // charge/discharge status (0 stationary, 1 charge, 2 discharge)
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

        // debug data string
        std::string aDebug;
    } get;

    /**
     * @brief alarm struct holds booleans corresponding to all the possible alarms
     * (aka errors/warnings) the BMS can report
     */

    struct
    {
        // data from 0x98
        /* 0x00 */
        bool levelOneCellVoltageTooHigh;
        bool levelTwoCellVoltageTooHigh;
        bool levelOneCellVoltageTooLow;
        bool levelTwoCellVoltageTooLow;
        bool levelOnePackVoltageTooHigh;
        bool levelTwoPackVoltageTooHigh;
        bool levelOnePackVoltageTooLow;
        bool levelTwoPackVoltageTooLow;

        /* 0x01 */
        bool levelOneChargeTempTooHigh;
        bool levelTwoChargeTempTooHigh;
        bool levelOneChargeTempTooLow;
        bool levelTwoChargeTempTooLow;
        bool levelOneDischargeTempTooHigh;
        bool levelTwoDischargeTempTooHigh;
        bool levelOneDischargeTempTooLow;
        bool levelTwoDischargeTempTooLow;

        /* 0x02 */
        bool levelOneChargeCurrentTooHigh;
        bool levelTwoChargeCurrentTooHigh;
        bool levelOneDischargeCurrentTooHigh;
        bool levelTwoDischargeCurrentTooHigh;
        bool levelOneStateOfChargeTooHigh;
        bool levelTwoStateOfChargeTooHigh;
        bool levelOneStateOfChargeTooLow;
        bool levelTwoStateOfChargeTooLow;

        /* 0x03 */
        bool levelOneCellVoltageDifferenceTooHigh;
        bool levelTwoCellVoltageDifferenceTooHigh;
        bool levelOneTempSensorDifferenceTooHigh;
        bool levelTwoTempSensorDifferenceTooHigh;

        /* 0x04 */
        bool chargeFETTemperatureTooHigh;
        bool dischargeFETTemperatureTooHigh;
        bool failureOfChargeFETTemperatureSensor;
        bool failureOfDischargeFETTemperatureSensor;
        bool failureOfChargeFETAdhesion;
        bool failureOfDischargeFETAdhesion;
        bool failureOfChargeFETTBreaker;
        bool failureOfDischargeFETBreaker;

        /* 0x05 */
        bool failureOfAFEAcquisitionModule;
        bool failureOfVoltageSensorModule;
        bool failureOfTemperatureSensorModule;
        bool failureOfEEPROMStorageModule;
        bool failureOfRealtimeClockModule;
        bool failureOfPrechargeModule;
        bool failureOfVehicleCommunicationModule;
        bool failureOfIntranetCommunicationModule;

        /* 0x06 */
        bool failureOfCurrentSensorModule;
        bool failureOfMainVoltageSensorModule;
        bool failureOfShortCircuitProtection;
        bool failureOfLowVoltageNoCharging;
    } alarm;

    RoverBatteryNode(
        const std::string & node_name, const std::string & ns = "/",
        const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    
    void init();
    
    /*
     * 0x00 Reset the BMS
     */
    bool batteryReset();
    
    /*
     * 0xDA 0x80 First Byte 0x01=ON 0x00=OFF
     */
    bool setChargeMOS(bool sw);
    
    /*
     * 0xD9 0x80 First Byte 0x01=ON 0x00=OFF
     */
    bool setDischargeMOS(bool sw);
    
private:

    static constexpr float kDesignedCapacity = 40.0;
    static constexpr std::string_view kSerialNumber = "224KA141600043";
    
    void batteryReadTimerCallback();
    void batteryReadTimeoutCallback();

    std::uint8_t getBatteryHealth();

    bool updateBatteryStatus();

    uint8_t evalChecksum(std::vector<uint8_t> & buff);
    bool sendCommand(Command cmd_id);
    
    ReturnStatus parseAllIncomingCommandBytes(Command cmd_id);
    ReturnStatus checkRxBuffer();
    ReturnStatus getPackMeasurements();         // 0x90
    ReturnStatus getMinMaxCellVoltage();        // 0x91
    ReturnStatus getPackTemp();                 // 0x92
    ReturnStatus getDischargeChargeMosStatus(); // 0x93
    ReturnStatus getStatusInfo();               // 0x94
    ReturnStatus getCellVoltages();             // 0x95
    ReturnStatus getCellTemperature();          // 0x96
    ReturnStatus getCellBalanceState();         // 0x97
    ReturnStatus getFailureCodes();             // 0x98
    
    void publishBatteryInfo();

    std::vector<uint8_t> tx_buffer_;
    std::vector<uint8_t> rx_buffer_;
    
    bool wait_for_receiver_{false};

    CppLinuxSerial::SerialPort serial_port_;

    std::string serial_device_name_;
    
    CommandState current_command_state_{CommandState::IDLE};

    Command current_command_{Command::VOUT_IOUT_SOC};
    
    rclcpp::TimerBase::SharedPtr battery_read_timer_;
    
    bool rx_timeout_started_{false};
    bool rx_timeout_reached_{false};
    
    rclcpp::TimerBase::SharedPtr battery_read_timeout_;
    
    BatteryStateMsg battery_state_msgs_;
    ChargingStatusMsg charging_state_msgs_;
    std::string error_msg_;
    
    std::shared_ptr<BatteryPublisher> battery_publisher_;

    std::shared_ptr<diagnostic_updater::Updater> diagnostic_updater_;
};

}  // namespace rover_battery

#endif  // ROVER_BATTERY_ROVER_BATTERY_NODE_HPP_