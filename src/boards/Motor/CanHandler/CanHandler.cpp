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

#include <Motor/Actuators/Actuators.h>
#include <Motor/Configs/CanHandlerConfig.h>
#include <Motor/Sensors/Sensors.h>
#include <common/CanConfig.h>
#include <common/Events.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;
using namespace CanConfig;
using namespace Motor::CanHandlerConfig;

namespace Motor
{

CanHandler::CanHandler(TaskScheduler *sched) : scheduler(sched)
{
    // Configure the CAN driver
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;
    CanbusDriver::CanbusConfig config;
    driver = new CanbusDriver(CAN1, config, bitTiming);
    driver = new CanbusDriver(CAN2, config, bitTiming);

    // Create the protocol with the defined driver
    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1));

    // Accept messages only from the main and RIG board
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::RIG),
                        static_cast<uint8_t>(Board::BROADCAST));
    driver->init();
}

bool CanHandler::start()
{
    // 0 if fail
    auto result1 = scheduler->addTask(
        [&]()
        {
            auto sensors = ModuleManager::getInstance().get<Sensors>();

            protocol->enqueueData(static_cast<uint8_t>(Priority::MEDIUM),
                                  static_cast<uint8_t>(PrimaryType::SENSORS),
                                  static_cast<uint8_t>(Board::MOTOR),
                                  static_cast<uint8_t>(Board::BROADCAST),
                                  static_cast<uint8_t>(SensorId::CC_PRESSURE),
                                  static_cast<PressureData>(
                                      sensors->getChamberPressureSensorData()));

            protocol->enqueueData(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::SENSORS),
                static_cast<uint8_t>(Board::MOTOR),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(SensorId::BOTTOM_TANK_PRESSURE),
                static_cast<PressureData>(
                    sensors->getTankPressureSensor1Data()));

            protocol->enqueueData(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::SENSORS),
                static_cast<uint8_t>(Board::MOTOR),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(SensorId::TOP_TANK_PRESSURE),
                static_cast<PressureData>(
                    sensors->getTankPressureSensor2Data()));
        },
        PRESSURES_TRANSMISSION_PERIOD);

    auto result2 = scheduler->addTask(
        [&]()
        {
            auto sensors = ModuleManager::getInstance().get<Sensors>();

            protocol->enqueueData(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::SENSORS),
                static_cast<uint8_t>(Board::MOTOR),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(SensorId::TANK_TEMPERATURE),
                static_cast<TemperatureData>(sensors->getMAX31856Data()));

            protocol->enqueueData(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::SENSORS),
                static_cast<uint8_t>(Board::MOTOR),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(SensorId::MOTOR_ACTUATORS_CURRENT),
                static_cast<CurrentData>(sensors->getServoCurrentData()));
        },
        TEMPERATURE_TRANSMISSION_PERIOD);

    // TODO: look at the priorities of the CAN protocol threads
    return protocol->start() && result1 != 0 && result2 != 0;
}

bool CanHandler::isStarted()
{
    return protocol->isStarted() && scheduler->isRunning();
}

void CanHandler::sendEvent(EventId event)
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::MOTOR),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(event));
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
        case PrimaryType::COMMAND:
        {
            handleCanCommand(msg);
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

    switch (eventId)
    {
        case EventId::CALIBRATE:
        {
            auto sensors = ModuleManager::getInstance().get<Sensors>();
            sensors->calibrate();

            break;
        }
        default:
        {
            LOG_WARN(logger, "Received unsupported event: id={}", eventId);
            break;
        }
    }
}

void CanHandler::handleCanCommand(const CanMessage &msg)
{
    uint64_t payload = msg.payload[0];
    ServosList servo = static_cast<ServosList>(msg.getSecondaryType());
    bool targetState = static_cast<uint8_t>(payload);
    uint32_t delay   = static_cast<uint32_t>(payload >> 8);

    auto actuators = ModuleManager::getInstance().get<Actuators>();

    if (actuators != nullptr)
    {
        if (targetState)
        {
            printf("Opening %d for %ld\n", msg.getSecondaryType(), delay);
            actuators->openServoAtomic(servo, delay);
        }
        else
        {
            printf("Closing %d for %ld\n", msg.getSecondaryType(), delay);
            actuators->closeServoAtomic(servo, delay);
        }
    }
}

}  // namespace Motor
