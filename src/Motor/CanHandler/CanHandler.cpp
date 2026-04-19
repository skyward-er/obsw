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
#include <common/CanConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/EventData.h>

using namespace miosix;
using namespace Motor;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;

CanHandler::CanHandler()
    : driver(CAN1, CanConfig::CONFIG, CanConfig::BIT_TIMING),
      protocol(
          &driver, [this](const CanMessage& msg) { handleMessage(msg); },
          getModule<BoardScheduler>()->canHandlerPriority())
{
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::RIG),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
    protocol.addFilter(static_cast<uint8_t>(CanConfig::Board::MAIN),
                       static_cast<uint8_t>(CanConfig::Board::BROADCAST));
}

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

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::STATUS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST), 0x00,
                DeviceStatus{
                    .timestamp = TimestampTimer::getTimestamp(),
                    .logNumber = static_cast<int16_t>(stats.logNumber),
                    .state     = initStatus.load(),
                    .armed     = false,
                    .hil       = PersistentVars::getHilMode(),
                    .logGood   = stats.lastWriteError == 0,
                });
        },
        CanConfig::STATUS_SEND_PERIOD);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert status update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::OX_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getOxTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(
                    CanConfig::SensorId::COMBUSTION_CHAMBER_PRESSURE),
                static_cast<PressureData>(sensors->getCCPressure()));
        },
        Config::CanHandler::CRITICAL_PRESSURE_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert pressure update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::FUEL_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getFuelTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::PRZ_TANK_PRESSURE),
                static_cast<PressureData>(sensors->getPrzTankPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::REG_OUT_OX_PRESSURE),
                static_cast<PressureData>(
                    sensors->getRegulatorOutOxPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(
                    CanConfig::SensorId::REG_OUT_FUEL_PRESSURE),
                static_cast<PressureData>(
                    sensors->getRegulatorOutFuelPressure()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::IGNITER_PRESSURE),
                static_cast<PressureData>(sensors->getIgniterPressure()));
        },
        Config::CanHandler::SECONDARY_PRESSURE_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert pressure update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Sensors* sensors = getModule<Sensors>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_VOLTAGE),
                static_cast<VoltageData>(sensors->getBatteryVoltage()));

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::MEDIUM),
                static_cast<uint8_t>(CanConfig::PrimaryType::SENSORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(CanConfig::SensorId::MOTOR_BOARD_CURRENT),
                static_cast<CurrentData>(sensors->getCurrentConsumption()));
        },
        Config::CanHandler::SENSORS_SEND_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to insert temperature update");
        return false;
    }

    result = scheduler.addTask(
        [this]()
        {
            Actuators* actuators = getModule<Actuators>();

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::OX_VENTING_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::OX_VENTING_VALVE),
                    actuators->isValveOpen(ServosList::OX_VENTING_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::FUEL_VENTING_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::FUEL_VENTING_VALVE),
                    actuators->isValveOpen(ServosList::FUEL_VENTING_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::MAIN_OX_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::MAIN_OX_VALVE),
                    actuators->isValveOpen(ServosList::MAIN_OX_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::MAIN_FUEL_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::MAIN_FUEL_VALVE),
                    actuators->isValveOpen(ServosList::MAIN_FUEL_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::PRZ_OX_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::PRZ_OX_VALVE),
                    actuators->isValveOpen(ServosList::PRZ_OX_VALVE)});

            protocol.enqueueData(
                static_cast<uint8_t>(CanConfig::Priority::HIGH),
                static_cast<uint8_t>(CanConfig::PrimaryType::ACTUATORS),
                static_cast<uint8_t>(CanConfig::Board::MOTOR),
                static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                static_cast<uint8_t>(ServosList::PRZ_FUEL_VALVE),
                ServoFeedback{
                    TimestampTimer::getTimestamp(),
                    actuators->getValvePosition(ServosList::PRZ_FUEL_VALVE),
                    actuators->isValveOpen(ServosList::PRZ_FUEL_VALVE)});
        },
        Config::CanHandler::VALVE_STATE_SEND_RATE);

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

void CanHandler::setInitStatus(uint8_t status) { initStatus = status; }

void CanHandler::sendServoOpenCommand(ServosList servo, uint32_t openingTime)
{
    protocol.enqueueData(
        static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
        static_cast<uint8_t>(CanConfig::PrimaryType::COMMAND),
        static_cast<uint8_t>(CanConfig::Board::MOTOR),
        static_cast<uint8_t>(CanConfig::Board::BROADCAST),
        static_cast<uint8_t>(servo),
        ServoCommand{TimestampTimer::getTimestamp(), openingTime});
}

void CanHandler::sendServoCloseCommand(ServosList servo)
{
    // Closing a servo means opening it for 0s
    sendServoOpenCommand(servo, 0);
}

void CanHandler::sendEvent(CanConfig::EventId event)
{
    sdLogger.log(CanEvent{TimestampTimer::getTimestamp(),
                          static_cast<uint8_t>(CanConfig::Board::MOTOR),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event)});

    protocol.enqueueEvent(static_cast<uint8_t>(CanConfig::Priority::CRITICAL),
                          static_cast<uint8_t>(CanConfig::PrimaryType::EVENTS),
                          static_cast<uint8_t>(CanConfig::Board::MOTOR),
                          static_cast<uint8_t>(CanConfig::Board::BROADCAST),
                          static_cast<uint8_t>(event));
}

void CanHandler::handleMessage(const Canbus::CanMessage& msg)
{
    auto source = static_cast<CanConfig::Board>(msg.getSource());
    if (source == CanConfig::Board::MAIN)
    {
        getModule<Common::MainStatus>()->handleCanMessage(msg);
        return;
    }

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

void CanHandler::handleSensor(const Canbus::CanMessage& msg)
{
    auto sensor = static_cast<CanConfig::SensorId>(msg.getSecondaryType());
    LOG_WARN(logger, "Received unsupported sensor data: {}", sensor);
}

void CanHandler::handleEvent(const Canbus::CanMessage& msg)
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

void CanHandler::handleActuator(const Canbus::CanMessage& msg)
{
    CanServoFeedback data = servoFeedbackFromCanMessage(msg);
    sdLogger.log(data);
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

        default:
        {
            LOG_WARN(logger, "Received unsupported status: {}", source);
        }
    }
}

