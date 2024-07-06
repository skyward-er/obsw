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

using namespace Motor;
using namespace Boardcore;
using namespace Boardcore::Canbus;
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
}

bool CanHandler::start()
{
    driver.init();

    TaskScheduler &scheduler =
        getModule<BoardScheduler>()->getCanBusScheduler();

    uint8_t result;

    result = scheduler.addTask(
        [this]()
        {
            protocol.enqueueSimplePacket(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0,
                initStatus);
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
                static_cast<PressureData>(sensors->getCCPress()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::BOTTOM_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getBottomTopTankPress()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::TOP_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getTopTankPress()));
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
                static_cast<TemperatureData>(sensors->getTankTemp()));

            /*protocol.enqueueData(
                    static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                    static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                    static_cast<uint8_t>(CanConfig::Board::MOTOR),
                    static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                    static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_VOLTAGE),
                    static_cast<VoltageData>(
                        sensors->getBatteryVoltage()));*/
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
                ServoData{TimestampTimer::getTimestamp(), 0, 0,
                          actuators->getServoPosition(ServosList::MAIN_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::VENTING_VALVE),
                ServoData{
                    TimestampTimer::getTimestamp(), 0, 0,
                    actuators->getServoPosition(ServosList::VENTING_VALVE)});
        },
        Config::CanHandler::ACTUATORS_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert actuators update");
        return false;
    }

    return protocol.start();
}

void CanHandler::setInitStatus(uint8_t status) { initStatus = status; }

void CanHandler::handleMessage(const Boardcore::Canbus::CanMessage &msg)
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

void CanHandler::handleEvent(const Boardcore::Canbus::CanMessage &msg)
{
    // TODO: Log event
    LOG_INFO(logger, "Received event");
}

void CanHandler::handleCommand(const Boardcore::Canbus::CanMessage &msg)
{
    ServosList servo = static_cast<ServosList>(msg.getSecondaryType());

    uint64_t payload         = msg.payload[0];
    uint32_t openingTime     = static_cast<uint32_t>(payload);
    uint32_t maxApertureBits = static_cast<uint32_t>(payload >> 32);

    float maxAperture = 0.0f;
    std::memcpy(&maxAperture, &maxApertureBits, sizeof(maxApertureBits));

    if (openingTime == 0)
    {
        getModule<Actuators>()->closeServo(servo);
    }
    else
    {
        getModule<Actuators>()->openServoWithApertureAndTime(servo, maxAperture,
                                                             openingTime);
    }
}