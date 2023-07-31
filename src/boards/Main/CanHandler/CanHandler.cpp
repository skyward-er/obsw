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

#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/Configs/CanHandlerConfig.h>
#include <Main/Sensors/Sensors.h>
#include <common/CanConfig.h>
#include <common/Events.h>
#include <events/EventBroker.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;
using namespace CanConfig;
using namespace Main::CanHandlerConfig;

namespace Main
{
CanHandler::CanHandler(TaskScheduler *sched) : scheduler(sched)
{
    // Configure the CAN driver
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;

    CanbusDriver::CanbusConfig config;
    config.nart = 1;

    // NOTE configure the peripheral CAN1 due to shared configs
    // TODO solve this thing
    driver = new CanbusDriver(CAN1, config, bitTiming);
    driver = new CanbusDriver(CAN2, config, bitTiming);

    // Create the protocol with the defined driver
    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1));

    // Accept messages only from the main and RIG board
    protocol->addFilter(static_cast<uint8_t>(Board::PAYLOAD),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::RIG),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::MOTOR),
                        static_cast<uint8_t>(Board::BROADCAST));
    driver->init();
}

bool CanHandler::start()
{
    // 0 if fail
    uint8_t result = scheduler->addTask(  // status
        [&]()
        {
            // FlightModeManagerState state = ModuleManager::getInstance()
            //                                    .get<FlightModeManager>()
            //                                    ->getStatus()
            //                                    .state;
            protocol->enqueueSimplePacket(
                static_cast<uint8_t>(Priority::MEDIUM),
                static_cast<uint8_t>(PrimaryType::STATUS),
                static_cast<uint8_t>(Board::MAIN),
                static_cast<uint8_t>(Board::BROADCAST), static_cast<uint8_t>(3),
                0x123456789ABCDEF);  // TODO add if electronics is armed or not
                                     // and FMM state
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
                           static_cast<uint8_t>(Board::MAIN),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(event));
}

void CanHandler::sendCanCommand(ServosList servo, bool targetState,
                                uint32_t delay)
{
    uint64_t payload = delay;
    payload          = payload << 8;
    payload          = payload | targetState;
    protocol->enqueueSimplePacket(static_cast<uint8_t>(Priority::CRITICAL),
                                  static_cast<uint8_t>(PrimaryType::COMMAND),
                                  static_cast<uint8_t>(Board::MAIN),
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
        case SensorId::PITOT:
        {
            modules.get<Sensors>()->setPitot(pitotDataFromCanMessage(msg));
            break;
        }
        case SensorId::CC_PRESSURE:
        {
            modules.get<Sensors>()->setCCPressure(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::BOTTOM_TANK_PRESSURE:
        {
            modules.get<Sensors>()->setBottomTankPressure(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::TOP_TANK_PRESSURE:
        {
            modules.get<Sensors>()->setTopTankPressure(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::TANK_TEMPERATURE:
        {
            modules.get<Sensors>()->setTankTemperature(
                temperatureDataFromCanMessage(msg));
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
    Board source  = static_cast<Board>(msg.getSource());
    uint8_t state = msg.getSecondaryType();
    bool isArmed  = msg.payload[0];
}

void CanHandler::handleCanCommand(const CanMessage &msg)
{
    uint64_t payload    = msg.payload[0];
    ServosList servo    = static_cast<ServosList>(msg.getSecondaryType());
    uint8_t targetState = static_cast<uint8_t>(payload);
    uint32_t delay      = static_cast<uint8_t>(payload >> 8);
}

}  // namespace Main
