/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "MotorStatus.h"

#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <miosix.h>

/**
 * NOTE: Some fields in the telemetry message are currently commented out since
 * they caused issues with the main and motor boards. Once the code of the main
 * and motor boards is completed they will need to be updated to include the
 * missing fields.
 */
using namespace Boardcore;

namespace Common
{
void MotorStatus::handleCanMessage(const Canbus::CanMessage& msg)
{
    auto type = static_cast<CanConfig::PrimaryType>(msg.getPrimaryType());

    miosix::Lock<miosix::FastMutex> lock(mutex);

    switch (type)
    {
        case CanConfig::PrimaryType::SENSORS:
        {
            handleSensors(msg);
            break;
        }

        case CanConfig::PrimaryType::STATUS:
        {
            lastStatus  = Clock::now();
            data.device = deviceStatusFromCanMessage(msg);
            break;
        }

        case CanConfig::PrimaryType::ACTUATORS:
        {
            handleActuators(msg);
            break;
        }
        case CanConfig::PrimaryType::ALGORITHM:
        {
            handleMea(msg);
            break;
        }

        default:
            break;
    }
}

void MotorStatus::handleSensors(const Canbus::CanMessage& msg)
{
    auto sensor = static_cast<CanConfig::SensorId>(msg.getSecondaryType());

#define CASE_PRESSURE(_type, _storage)                             \
    case _type:                                                    \
        do                                                         \
        {                                                          \
            auto _data          = pressureDataFromCanMessage(msg); \
            this->data._storage = _data;                           \
            sdLogger.log(_data);                                   \
        } while (0);                                               \
        break

    switch (sensor)
    {
        CASE_PRESSURE(CanConfig::SensorId::PRZ_TANK_PRESSURE, przTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::OX_TANK_PRESSURE, oxTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::FUEL_TANK_PRESSURE,
                      fuelTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::MAIN_CC_PRESSURE, mainCCPressure);
        CASE_PRESSURE(CanConfig::SensorId::IGN_CC_PRESSURE, ignCCPressure);

        case CanConfig::SensorId::MOTOR_BOARD_VOLTAGE:
        {
            auto voltageData    = voltageDataFromCanMessage(msg);
            data.batteryVoltage = voltageData;
            sdLogger.log(voltageData);
            break;
        }

        case CanConfig::SensorId::MOTOR_BOARD_CURRENT:
        {
            auto currentData        = currentDataFromCanMessage(msg);
            data.currentConsumption = currentData;
            sdLogger.log(currentData);
            break;
        }

        default:
        {
            auto logger = Logging::getLogger("MotorStatus");
            LOG_WARN(logger, "Received data for unhandled sensor: {}", sensor);
            break;
        }
    }
}

void MotorStatus::handleActuators(const Canbus::CanMessage& msg)
{
    auto valveData   = valveDataFromCanMessage(msg);
    ServosList valve = static_cast<ServosList>(valveData.idx);
    sdLogger.log(valveData);

    switch (valve)
    {
        case ServosList::OX_VENTING_VALVE:
            data.oxVentingValveState    = valveData.open;
            data.oxVentingValvePosition = valveData.position;
            break;

        case ServosList::FUEL_VENTING_VALVE:
            data.fuelVentingValveState    = valveData.open;
            data.fuelVentingValvePosition = valveData.position;
            break;

        case ServosList::PRZ_OX_VALVE:
            data.przOxValveState    = valveData.open;
            data.przOxValvePosition = valveData.position;
            break;

        case ServosList::PRZ_FUEL_VALVE:
            data.przFuelValveState    = valveData.open;
            data.przFuelValvePosition = valveData.position;
            break;

        case ServosList::MAIN_OX_VALVE:
            data.mainOxValveState    = valveData.open;
            data.mainOxValvePosition = valveData.position;
            break;

        case ServosList::MAIN_FUEL_VALVE:
            data.mainFuelValvePosition = valveData.position;
            data.mainFuelValveState    = valveData.open;
            break;

        case ServosList::IGNITION_OX_VALVE:
            data.oxSolenoidState = valveData.open;
            break;

        case ServosList::IGNITION_FUEL_VALVE:
            data.fuelSolenoidState = valveData.open;
            break;

        default:
        {
            auto logger = Logging::getLogger("MotorStatus");
            LOG_WARN(logger, "Received data for unhandled valve: {} ({})",
                     servoToString(valve), valve);
            break;
        }
    }
}

void MotorStatus::handleMea(const Boardcore::Canbus::CanMessage& msg)
{
    meaMass.store(meaMassFromCanMessage(msg));
}

mavlink_motor_tm_t MotorStatus::getMotorTelemetry()
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    return {
        .timestamp                   = TimestampTimer::getTimestamp(),
        .prz_tank_pressure           = data.przTankPressure.pressure,
        .ox_tank_pressure            = data.oxTankPressure.pressure,
        .fuel_tank_pressure          = data.fuelTankPressure.pressure,
        .main_cc_pressure            = data.mainCCPressure.pressure,
        .ign_cc_pressure             = data.ignCCPressure.pressure,
        .battery_voltage             = data.batteryVoltage.voltage,
        .current_consumption         = data.currentConsumption.current,
        .log_number                  = data.device.logNumber,
        .ox_venting_valve_state      = data.oxVentingValveState,
        .ox_venting_valve_position   = data.oxVentingValvePosition,
        .fuel_venting_valve_state    = data.fuelVentingValveState,
        .fuel_venting_valve_position = data.fuelVentingValvePosition,
        .prz_ox_valve_state          = data.przOxValveState,
        .prz_ox_valve_position       = data.przOxValvePosition,
        .prz_fuel_valve_state        = data.przFuelValveState,
        .prz_fuel_valve_position     = data.przFuelValvePosition,
        .main_ox_valve_state         = data.mainOxValveState,
        .main_ox_valve_position      = data.mainOxValvePosition,
        .main_fuel_valve_state       = data.mainFuelValveState,
        .main_fuel_valve_position    = data.mainFuelValvePosition,
        .ox_solenoid_state           = data.oxSolenoidState,
        .fuel_solenoid_state         = data.fuelSolenoidState,
        .spark_igniter_state         = data.sparkIgniterOn,
        .firing_sequence_hsm_state   = data.device.state,
        .log_good                    = data.device.logGood,
        .hil_state                   = data.device.hil,
    };
}

}  // namespace Common
