/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Alberto Nidasio, Matteo Pignataro
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

#include <RIG/BoardScheduler.h>
#include <RIG/Buses.h>
#include <RIG/CanHandler/CanHandlerData.h>
#include <RIG/Configs/CanHandlerConfig.h>
#include <RIG/Sensors/Sensors.h>
#include <RIG/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIG/StatesMonitor/StatesMonitor.h>
#include <common/CanConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;
using namespace CanConfig;
using namespace RIG::CanHandlerConfig;

namespace RIG
{
CanHandler::CanHandler(TaskScheduler *sched) : scheduler(sched)
{
    // Configure the CAN driver
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;

    CanbusDriver::CanbusConfig config;

    // NOTE configure the peripheral CAN1 due to shared configs
    driver = new CanbusDriver(CAN1, config, bitTiming);

    // Create the protocol with the defined driver
    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1),
                        miosix::PRIORITY_MAX - 1);

    // Accept messages only from the main and RIG board
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::MOTOR),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::PAYLOAD),
                        static_cast<uint8_t>(Board::BROADCAST));
    driver->init();
}

bool CanHandler::start()
{
    // 0 if fail
    uint8_t result = scheduler->addTask(  // status
        [&]()
        {
            GroundModeManagerState state = ModuleManager::getInstance()
                                               .get<GroundModeManager>()
                                               ->getStatus()
                                               .state;
            protocol->enqueueSimplePacket(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::STATUS),
                static_cast<uint8_t>(Board::RIG),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(state),
                (state == GroundModeManagerState::ARMED) ? 0x01 : 0x00);
        },
        STATUS_TRANSMISSION_PERIOD);
    // TODO look at the priorities of the CAN protocol threads
    return protocol->start() && result != 0;
}

bool CanHandler::isStarted()
{
    return protocol->isStarted() && scheduler->isRunning();
}

void CanHandler::sendEvent(EventId event)
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::RIG),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(event));
}

void CanHandler::sendCanServoCommand(ServosList servo, bool targetState,
                                     uint32_t delay)
{
    uint64_t payload = delay;
    payload          = payload << 8;
    payload          = payload | targetState;
    protocol->enqueueSimplePacket(static_cast<uint8_t>(Priority::CRITICAL),
                                  static_cast<uint8_t>(PrimaryType::COMMAND),
                                  static_cast<uint8_t>(Board::RIG),
                                  static_cast<uint8_t>(Board::BROADCAST),
                                  static_cast<uint8_t>(servo), payload);
}

void CanHandler::handleCanMessage(const CanMessage &msg)
{
    PrimaryType msgType = static_cast<PrimaryType>(msg.getPrimaryType());

    // Depending on the received message, call the handling method
    switch (msgType)
    {
        case PrimaryType::EVENTS:
        {
            handleCanEvent(msg);
            break;
        }
        case PrimaryType::SENSORS:
        {
            handleCanSensor(msg);
            break;
        }
        case PrimaryType::STATUS:
        {
            handleCanStatus(msg);
            break;
        }
        default:
        {
            LOG_WARN(logger, "Received unsupported message type: type={}",
                     msgType);
            break;
        }
    }
}

void CanHandler::handleCanEvent(const CanMessage &msg)
{
    EventId eventId = static_cast<EventId>(msg.getSecondaryType());
    auto it         = eventToEvent.find(eventId);

    // Check if the event is valid
    if (it != eventToEvent.end())
    {
        EventBroker::getInstance().post(it->second, TOPIC_CAN);
    }
    else
    {
        LOG_WARN(logger, "Received unsupported event: id={}", eventId);
    }
}

void CanHandler::handleCanSensor(const CanMessage &msg)
{
    SensorId sensorId      = static_cast<SensorId>(msg.getSecondaryType());
    ModuleManager &modules = ModuleManager::getInstance();

    // Depending on the sensor call the corresponding setter on the Sensors
    // module
    switch (sensorId)
    {
        case SensorId::CC_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            modules.get<Sensors>()->setCCPressure(data);

            // Log the data
            CanPressureSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.pressure  = data.pressure;
            log.sensorId  = static_cast<uint8_t>(SensorId::CC_PRESSURE);
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::BOTTOM_TANK_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            modules.get<Sensors>()->setBottomTankPressure(data);

            // Log the data
            CanPressureSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.pressure  = data.pressure;
            log.sensorId = static_cast<uint8_t>(SensorId::BOTTOM_TANK_PRESSURE);
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::TOP_TANK_PRESSURE:
        {
            PressureData data = pressureDataFromCanMessage(msg);
            modules.get<Sensors>()->setTopTankPressure(data);

            // Log the data
            CanPressureSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.pressure  = data.pressure;
            log.sensorId  = static_cast<uint8_t>(SensorId::TOP_TANK_PRESSURE);
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::TANK_TEMPERATURE:
        {
            TemperatureData data = temperatureDataFromCanMessage(msg);
            modules.get<Sensors>()->setTankTemperature(data);

            // Log the data
            CanTemperatureSensor log;
            log.timestamp   = TimestampTimer::getTimestamp();
            log.temperature = data.temperature;
            log.sensorId    = static_cast<uint8_t>(SensorId::TANK_TEMPERATURE);

            break;
        }
        case SensorId::MOTOR_ACTUATORS_CURRENT:
        {
            CurrentData data = currentDataFromCanMessage(msg);
            modules.get<Sensors>()->setMotorCurrent(data);

            // Log the data
            CanCurrentSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.sensorId =
                static_cast<uint8_t>(SensorId::MOTOR_ACTUATORS_CURRENT);
            log.current = data.current;
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::MAIN_BOARD_CURRENT:
        {
            CurrentData data = currentDataFromCanMessage(msg);
            modules.get<Sensors>()->setMainCurrent(data);

            // Log the data
            CanCurrentSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.sensorId  = static_cast<uint8_t>(SensorId::MAIN_BOARD_CURRENT);
            log.current   = data.current;
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::PAYLOAD_BOARD_CURRENT:
        {
            CurrentData data = currentDataFromCanMessage(msg);
            modules.get<Sensors>()->setPayloadCurrent(data);

            // Log the data
            CanCurrentSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.sensorId =
                static_cast<uint8_t>(SensorId::PAYLOAD_BOARD_CURRENT);
            log.current = data.current;
            Logger::getInstance().log(log);

            break;
        }
        case SensorId::MOTOR_BOARD_VOLTAGE:
        {
            BatteryVoltageSensorData data = voltageDataFromCanMessage(msg);
            modules.get<Sensors>()->setMotorVoltage(data);

            // Log the data
            CanVoltageSensor log;
            log.timestamp = TimestampTimer::getTimestamp();
            log.sensorId  = static_cast<uint8_t>(SensorId::MOTOR_BOARD_VOLTAGE);
            log.voltage   = data.batVoltage;
            Logger::getInstance().log(log);

            break;
        }
        default:
        {
            LOG_WARN(logger, "Received unsupported sensor data: id={}",
                     sensorId);
        }
    }
}

void CanHandler::handleCanStatus(const CanMessage &msg)
{
    Board source = static_cast<Board>(msg.getSource());
    // uint8_t status = msg.getSecondaryType();
    bool armed = msg.payload[0];

    // Set the state inside the monitor
    ModuleManager::getInstance().get<StatesMonitor>()->setBoardStatus(
        source, armed ? 2 : 1);
}

}  // namespace RIG
