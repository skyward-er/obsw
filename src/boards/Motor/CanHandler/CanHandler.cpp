/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/Actuators/Actuators.h>
#include <Motor/Configs/CanHandlerConfig.h>
#include <Motor/Configs/SchedulerConfig.h>
#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventData.h>

using namespace Motor;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;

CanHandler::CanHandler()
    : driver(CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage &msg) { handleMessage(msg); },
          Config::Scheduler::CAN_PRIORITY)
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::RIG),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::PAYLOAD),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

bool CanHandler::start()
{
    driver.init();

    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getCanBusScheduler();

    uint8_t result = scheduler.addTask(
        [this]()
        {
            LoggerStats stats = sdLogger.getStats();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
                DeviceStatus{
                    TimestampTimer::getTimestamp(),
                    static_cast<int16_t>(stats.logNumber),
                    static_cast<uint8_t>(initStatus.load()),
                    false,
                    PersistentVars::getHilMode(),
                    stats.lastWriteError == 0,
                });
        },
        Config::CanHandler::STATUS_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert status update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors *sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::CC_PRESSURE),
                static_cast<PressureData>(sensors->getCCPressLastSample()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::TOP_TANK_PRESSURE),
                static_cast<PressureData>(
                    sensors->getTopTankPressLastSample()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::BOTTOM_TANK_PRESSURE),
                static_cast<PressureData>(
                    sensors->getBottomTankPressLastSample()));
        },
        Config::CanHandler::PRESSURE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert pressure update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors *sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::TANK_TEMPERATURE),
                static_cast<TemperatureData>(sensors->getTankTempLastSample()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_VOLTAGE),
                static_cast<VoltageData>(
                    sensors->getBatteryVoltageLastSample()));
        },
        Config::CanHandler::TEMPERATURE_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert temperature update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Actuators *actuators = getModule<Actuators>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::MAIN_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getServoPosition(ServosList::MAIN_VALVE),
                    actuators->isServoOpen(ServosList::MAIN_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::VENTING_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getServoPosition(ServosList::VENTING_VALVE),
                    actuators->isServoOpen(ServosList::VENTING_VALVE)});
        },
        Config::CanHandler::ACTUATORS_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert actuators update");
        return false;
    }

    if (!protocol.start())
    {
        LOG_ERR(logger, "Failed to start CanProtocol");
        return false;
    }

    return true;
}

void CanHandler::setInitStatus(InitStatus status) { initStatus = status; }

void CanHandler::handleMessage(const Canbus::CanMessage &msg)
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

        case CanConfig::PrimaryType::COMMAND:
        {
            handleCommand(msg);
            break;
        }

        default:
        {
            LOG_WARN(logger, "Received unsupported message type: {}", type);
            break;
        }
    }
}

void CanHandler::handleEvent(const Canbus::CanMessage &msg)
{
    CanConfig::EventId event =
        static_cast<CanConfig::EventId>(msg.getSecondaryType());

    if (event == Common::CanConfig::EventId::ENTER_HIL_MODE)
    {
        PersistentVars::setHilMode(true);
        miosix::reboot();
    }
    else if (event == Common::CanConfig::EventId::EXIT_HIL_MODE)
    {
        if (PersistentVars::getHilMode())
        {
            PersistentVars::setHilMode(false);
            miosix::reboot();
        }
    }

    // Log the event
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(), msg.getSource(),
                          msg.getDestination(), msg.getSecondaryType()});
}

void CanHandler::handleCommand(const Canbus::CanMessage &msg)
{
    ServosList servo        = static_cast<ServosList>(msg.getSecondaryType());
    CanServoCommand command = servoCommandFromCanMessage(msg);
    sdLogger.log(command);

    if (command.openingTime == 0)
    {
        getModule<Actuators>()->closeServo(servo);
    }
    else
    {
        getModule<Actuators>()->openServoWithTime(servo, command.openingTime);
    }
}