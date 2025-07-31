/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include "CanHandler.h"

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>

using namespace miosix;
using namespace RIGv2;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;

CanHandler::CanHandler()
    : driver(CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage& msg) { handleMessage(msg); },
          BoardScheduler::canHandlerPriority())
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MOTOR),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::PAYLOAD),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

bool CanHandler::isStarted() { return started; }

bool CanHandler::start()
{
    if (!driver.init(Common::CanConfig::CAN_SYNC_TIMEOUT))
    {
        LOG_ERR(logger, "Failed to initialize CanbusDriver");
        return false;
    }

    TaskScheduler& scheduler = getModule<BoardScheduler>()->canHandler();

    uint8_t result = scheduler.addTask(
        [this]()
        {
            LoggerStats stats = sdLogger.getStats();

            GroundModeManagerState state =
                getModule<GroundModeManager>()->getState();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::RIG),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
                DeviceStatus{
                    TimestampTimer::getTimestamp(),
                    static_cast<int16_t>(stats.logNumber),
                    static_cast<uint8_t>(state),
                    state == GroundModeManagerState::ARMED,
                    false,
                    stats.lastWriteError == 0,
                });
        },
        Config::CanHandler::STATUS_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add periodicMessageTask");
        return false;
    }

    if (!protocol.start())
    {
        LOG_ERR(logger, "Failed to start CanProtocol");
        return false;
    }

    started = true;
    return true;
}

void CanHandler::sendEvent(CanConfig::EventId event)
{
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(),
                          static_cast<uint8_t>(CanConfig::Board::RIG),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event)});

    protocol.enqueueEvent(static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
                          static_cast<uint8_t>(CanConfig::PrimaryType::EVENTS),
                          static_cast<uint8_t>(CanConfig::Board::RIG),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event));
}

void CanHandler::sendServoOpenCommand(ServosList servo, uint32_t openingTime)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::RIG),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(servo),
        ServoCommand{TimestampTimer::getTimestamp(), openingTime});
}

void CanHandler::sendServoCloseCommand(ServosList servo)
{
    // Closing a servo means opening it for 0s
    sendServoOpenCommand(servo, 0);
}

CanHandler::CanStatus CanHandler::getCanStatus()
{
    Lock<FastMutex> lock{statusMutex};
    return status;
}

void CanHandler::handleMessage(const Canbus::CanMessage& msg)
{
    CanConfig::PrimaryType type =
        static_cast<CanConfig::PrimaryType>(msg.getPrimaryType());

    switch (type)
    {
        case CanConfig::PrimaryType::EVENTS:
        {
            handleEvent(msg);
            break;
        }

        case CanConfig::PrimaryType::SENSORS:
        {
            handleSensor(msg);
            break;
        }

        case CanConfig::PrimaryType::STATUS:
        {
            handleStatus(msg);
            break;
        }

        case CanConfig::PrimaryType::ACTUATORS:
        {
            handleActuator(msg);
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported message type: {}", type);
            break;
        }
    }
}

void CanHandler::handleEvent(const Canbus::CanMessage& msg)
{
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(), msg.getSource(),
                          msg.getDestination(), msg.getSecondaryType()});

    // Dispatch the event, so that the logger can log it
    Events event = canEventToEvent(msg.getSecondaryType());
    if (event != LAST_EVENT)
    {
        EventBroker::getInstance().post(event, TOPIC_CAN);
    }
    else
    {
        LOG_WARN(logger, "Received unsupported event: {}",
                 msg.getSecondaryType());
    }
}

void CanHandler::handleSensor(const Canbus::CanMessage& msg)
{
    CanConfig::SensorId sensor =
        static_cast<CanConfig::SensorId>(msg.getSecondaryType());

    Sensors* sensors = getModule<Sensors>();
    switch (sensor)
    {
        case CanConfig::SensorId::COMBUSTION_CHAMBER_PRESSURE:
        {
            CanPressureData data = pressureDataFromCanMessage(msg);
            sdLogger.log(data);
            sensors->setCanCombustionChamberPressure(data);
            break;
        }

        case CanConfig::SensorId::OX_TANK_TOP_PRESSURE:
        {
            CanPressureData data = pressureDataFromCanMessage(msg);
            sdLogger.log(data);
            sensors->setCanOxTankTopPressure(data);
            break;
        }

        case CanConfig::SensorId::OX_TANK_BOTTOM_0_PRESSURE:
        {
            CanPressureData data = pressureDataFromCanMessage(msg);
            sdLogger.log(data);
            sensors->setCanOxTankBottomPressure(data);
            break;
        }

        case CanConfig::SensorId::THERMOCOUPLE_TEMPERATURE:
        {
            CanTemperatureData data = temperatureDataFromCanMessage(msg);
            sdLogger.log(data);
            sensors->setCanOxTankTemperature(data);
            break;
        }

        case CanConfig::SensorId::MOTOR_BOARD_VOLTAGE:
        {
            CanVoltageData data = voltageDataFromCanMessage(msg);
            sdLogger.log(data);
            sensors->setCanMotorBatteryVoltage(data);
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported sensor data: {}", sensor);
        }
    }
}

void CanHandler::handleActuator(const Canbus::CanMessage& msg)
{
    ServosList servo      = static_cast<ServosList>(msg.getSecondaryType());
    CanServoFeedback data = servoFeedbackFromCanMessage(msg);
    sdLogger.log(data);

    getModule<Actuators>()->setCanServoOpen(servo, data.open);
}

void CanHandler::handleStatus(const Canbus::CanMessage& msg)
{
    CanConfig::Board source = static_cast<CanConfig::Board>(msg.getSource());
    CanDeviceStatus deviceStatus = deviceStatusFromCanMessage(msg);

    Lock<FastMutex> lock{statusMutex};

    switch (source)
    {
        case CanConfig::Board::MAIN:
        {
            status.mainLastStatus = getTime();
            status.mainArmed      = deviceStatus.armed;
            status.mainState      = deviceStatus.state;
            break;
        }

        case CanConfig::Board::PAYLOAD:
        {
            status.payloadLastStatus = getTime();
            status.payloadArmed      = deviceStatus.armed;
            status.payloadState      = deviceStatus.state;
            break;
        }

        case CanConfig::Board::MOTOR:
        {
            status.motorLastStatus = getTime();
            status.motorState      = deviceStatus.state;

            status.motorLogNumber = deviceStatus.logNumber;
            status.motorLogGood   = deviceStatus.logGood;
            status.motorHil       = deviceStatus.hil;

            // Tell sensors module to switch to can sensors
            getModule<Sensors>()->switchToCanSensors();
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported status: {}", source);
        }
    }
}
