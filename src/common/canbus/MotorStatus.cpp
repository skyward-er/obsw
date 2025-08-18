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
        CASE_PRESSURE(CanConfig::SensorId::N2_TANK_PRESSURE, n2TankPressure);
        CASE_PRESSURE(CanConfig::SensorId::REGULATOR_OUT_PRESSURE,
                      regulatorOutPressure);
        CASE_PRESSURE(CanConfig::SensorId::OX_TANK_TOP_PRESSURE,
                      oxTankTopPressure);
        CASE_PRESSURE(CanConfig::SensorId::OX_TANK_BOTTOM_0_PRESSURE,
                      oxTankBottom0Pressure);
        CASE_PRESSURE(CanConfig::SensorId::OX_TANK_BOTTOM_1_PRESSURE,
                      oxTankBottom1Pressure);
        CASE_PRESSURE(CanConfig::SensorId::COMBUSTION_CHAMBER_PRESSURE,
                      combustionChamberPressure);

        case CanConfig::SensorId::THERMOCOUPLE_TEMPERATURE:
        {
            auto temperatureData         = temperatureDataFromCanMessage(msg);
            data.thermocoupleTemperature = temperatureData;
            sdLogger.log(temperatureData);
            break;
        }

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

        case ServosList::MAIN_VALVE:
            data.mainValveOpen = valveData.open;
            break;

        case ServosList::NITROGEN_VALVE:
            data.nitrogenValveOpen = valveData.open;
            break;

        case ServosList::N2_QUENCHING_VALVE:
            data.n2QuenchingValveOpen = valveData.open;
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
        .timestamp                   = TimestampTimer::getTimestamp(),
        .n2_tank_pressure            = data.n2TankPressure.pressure,
        .reg_out_pressure            = data.regulatorOutPressure.pressure,
        .ox_tank_top_pressure        = data.oxTankTopPressure.pressure,
        .ox_tank_bot_0_pressure      = data.oxTankBottom0Pressure.pressure,
        .ox_tank_bot_1_pressure      = data.oxTankBottom1Pressure.pressure,
        .combustion_chamber_pressure = data.combustionChamberPressure.pressure,
        .thermocouple_temperature    = data.thermocoupleTemperature.temperature,
        .battery_voltage             = data.batteryVoltage.voltage,
        .current_consumption         = data.currentConsumption.current,
        .log_number                  = data.device.logNumber,
        .n2_quenching_valve_state    = data.n2QuenchingValveOpen,
        .ox_venting_valve_state      = data.oxVentingValveOpen,
        .nitrogen_valve_state        = data.nitrogenValveOpen,
        .main_valve_state            = data.mainValveOpen,
        .log_good                    = data.device.logGood,
        .hil_state                   = data.device.hil,
    };
}

}  // namespace Common
