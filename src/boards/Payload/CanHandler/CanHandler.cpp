/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Alberto Nidasio
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

#include <Payload/Actuators/Actuators.h>
#include <Payload/BoardScheduler.h>
#include <Payload/Configs/CanHandlerConfig.h>
#include <Payload/Configs/SensorsConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
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
using namespace Payload::SensorsConfig;
using namespace Payload::CanHandlerConfig;

namespace Payload
{

bool CanHandler::start()
{
    // Scheduler with main priority -1 (Sensors & FMM)
    scheduler =
        ModuleManager::getInstance().get<BoardScheduler>()->getScheduler(
            miosix::MAIN_PRIORITY - 1);
    // Add a task to periodically send the pitot data
    scheduler->addTask(  // sensor template
        [&]()
        {
            protocol->enqueueData(static_cast<uint8_t>(Priority::CRITICAL),
                                  static_cast<uint8_t>(PrimaryType::SENSORS),
                                  static_cast<uint8_t>(Board::PAYLOAD),
                                  static_cast<uint8_t>(Board::BROADCAST),
                                  static_cast<uint8_t>(SensorId::PITOT),
                                  ModuleManager::getInstance()
                                      .get<Sensors>()
                                      .getPitotLastSample());
        },
        PITOT_TRANSMISSION_PERIOD);

    scheduler->addTask(  // status
        [&]()
        {
            FlightModeManagerState state = ModuleManager::getInstance()
                                               .get<FlightModeManager>()
                                               ->getStatus()
                                               .state;
            protocol->enqueueEvent(
                static_cast<uint8_t>(Priority::HIGH),
                static_cast<uint8_t>(PrimaryType::STATUS),
                static_cast<uint8_t>(Board::PAYLOAD),
                static_cast<uint8_t>(Board::BROADCAST),
                static_cast<uint8_t>(state),
                ((state == FlightModeManagerState::ARMED) ? 0x01 : 0x00));
        },
        STATUS_TRANSMISSION_PERIOD);
    return protocol->start();
}

bool CanHandler::isStarted() { return protocol->isStarted(); }

void CanHandler::sendEvent(EventId event)
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(event));
}

void CanHandler::sendCanCommand(ServoID servo, bool targetState, uint32_t delay)
{
    uint64_t payload = delay;
    payload          = payload << 8;
    payload          = payload | targetState;
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::HIGH),
                           static_cast<uint8_t>(PrimaryType::COMMAND),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(servo), payload);
}

CanHandler::CanHandler()
{
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;
    driver                = new CanbusDriver(CAN1, {}, bitTiming);

    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1));

    // Accept messages only from the main and RIG board
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::RIG),
                        static_cast<uint8_t>(Board::BROADCAST));
    driver->init();
}

void CanHandler::handleCanMessage(const CanMessage &msg)
{
    PrimaryType msgType = static_cast<PrimaryType>(msg.getPrimaryType());

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

    auto it = eventToEvent.find(eventId);

    if (it != eventToEvent.end())
        EventBroker::getInstance().post(it->second, TOPIC_CAN);
    else
        LOG_WARN(logger, "Received unsupported event: id={}", eventId);
}

void CanHandler::handleCanSensor(const CanMessage &msg)
{
    SensorId sensorId = static_cast<SensorId>(msg.getSecondaryType());

    switch (sensorId)
    {
        case SensorId::PITOT:
        {
            Sensors::getInstance().setPitotData(pitotDataFromCanMessage(msg));
            break;
        }
        case SensorId::CC_PRESSURE:
        {  // name
            Sensors::getInstance().setCCPressureData(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::BOTTOM_TANK_PRESSURE:
        {  // name
            Sensors::getInstance().setBottomTankPressureData(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::TOP_TANK_PRESSURE:
        {  // name
            Sensors::getInstance().setTopTankPressureData(
                pressureDataFromCanMessage(msg));
            break;
        }
        case SensorId::TANK_TEMPERATURE:
        {  // name
            Sensors::getInstance().setTankTemperatureData(
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
    ServoID servo       = static_cast<ServoID>(msg.getSecondaryType());
    uint8_t targetState = static_cast<uint8_t>(payload);
    uint32_t delay      = static_cast<uint8_t>(payload >> 8);
}

}  // namespace Payload
