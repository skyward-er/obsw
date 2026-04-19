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
        CASE_PRESSURE(CanConfig::SensorId::FUEL_TANK_PRESSURE,
                      fuelTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::REG_OUT_OX_PRESSURE,
                      regulatorOutOxPressure);
        CASE_PRESSURE(CanConfig::SensorId::REG_OUT_FUEL_PRESSURE,
                      regulatorOutFuelPressure);
        CASE_PRESSURE(CanConfig::SensorId::PRZ_TANK_PRESSURE, przTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::OX_TANK_PRESSURE, oxTankPressure);
        CASE_PRESSURE(CanConfig::SensorId::COMBUSTION_CHAMBER_PRESSURE,
                      combustionChamberPressure);
        CASE_PRESSURE(CanConfig::SensorId::IGNITER_PRESSURE, igniterPressure);

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
    auto valve     = static_cast<ServosList>(msg.getSecondaryType());
    auto valveData = servoFeedbackFromCanMessage(msg);

    switch (valve)
    {
        case ServosList::OX_VENTING_VALVE:
            data.oxVentingValveOpen = valveData.open;
            break;

        case ServosList::FUEL_VENTING_VALVE:
            data.fuelVentingValveOpen = valveData.open;
            break;

        case ServosList::PRZ_OX_VALVE:
            data.przOxValveOpen = valveData.open;
            break;

        case ServosList::PRZ_FUEL_VALVE:
            data.przFuelValveOpen = valveData.open;
            break;

        case ServosList::MAIN_OX_VALVE:
            data.mainOxValveOpen = valveData.open;
            break;

        case ServosList::MAIN_FUEL_VALVE:
            data.mainFuelValveOpen = valveData.open;
            break;

        case ServosList::IGNITION_OX_VALVE:
            data.ignitionOxValveOpen = valveData.open;
            break;

        case ServosList::IGNITION_FUEL_VALVE:
            data.ignitionFuelValveOpen = valveData.open;
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

mavlink_motor_tm_t MotorStatus::getMotorTelemetry()
{
    miosix::Lock<miosix::FastMutex> lock(mutex);

    return {
        .timestamp              = TimestampTimer::getTimestamp(),
        .prz_tank_pressure      = data.przTankPressure.pressure,
        .ox_reg_out_pressure    = data.regulatorOutOxPressure.pressure,
        .fuel_reg_out_pressure  = data.regulatorOutFuelPressure.pressure,
        .ox_tank_pressure       = data.oxTankPressure.pressure,
        .fuel_tank_pressure     = data.fuelTankPressure.pressure,
        .battery_voltage        = data.batteryVoltage.voltage,
        .current_consumption    = data.currentConsumption.current,
        .log_number             = data.device.logNumber,
        .ox_venting_valve_state = data.oxVentingValveOpen,
        .log_good               = data.device.logGood,
        .hil_state              = data.device.hil,
    };
}

}  // namespace Common
