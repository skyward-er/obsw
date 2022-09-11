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
#include <common/CanConfig.h>
#include <common/events/Events.h>
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

bool CanHandler::start() { return EventHandler::start() && protocol->start(); }

bool CanHandler::isStarted() { return protocol->isStarted(); }

void CanHandler::sendArmEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::ARM));
}

void CanHandler::sendDisarmEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::DISARM));
}

void CanHandler::sendCalibrateEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::CALIBRATE));
}

void CanHandler::sendCamOnEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::MEDIUM),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::CAM_ON));
}

void CanHandler::sendCamOffEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::MEDIUM),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::PAYLOAD),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::CAM_OFF));
}

CanHandler::CanHandler()
{
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;
    driver                = new CanbusDriver(CAN1, {}, bitTiming);

    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1));

    // Accept messages only from the main board
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::PAYLOAD));
    driver->init();

    // Add a task to periodically send the pitot data
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]()
        {
            protocol->enqueueData(static_cast<uint8_t>(Priority::CRITICAL),
                                  static_cast<uint8_t>(PrimaryType::SENSORS),
                                  static_cast<uint8_t>(Board::PAYLOAD),
                                  static_cast<uint8_t>(Board::MAIN),
                                  static_cast<uint8_t>(SensorId::PITOT),
                                  Sensors::getInstance().getPitotLastSample());
        },
        PITOT_TRANSMISSION_PERIOD);

    EventBroker::getInstance().subscribe(this, TOPIC_TMTC);
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

        default:
        {
            break;
        }
    }
}

void CanHandler::handleCanEvent(const CanMessage &msg)
{
    EventId eventId = static_cast<EventId>(msg.getSecondaryType());
    auto it         = eventToEvent.find(eventId);

    if (it != eventToEvent.end())
        EventBroker::getInstance().post(it->second, TOPIC_TMTC, this);
    else
        LOG_DEBUG(logger, "Received unsupported event: id={}", eventId);
}

void CanHandler::handleEvent(const Event &event)
{
    auto it = eventToFunction.find(static_cast<Events>(event));

    if (it != eventToFunction.end())
        it->second(this);
    else
        LOG_DEBUG(logger, "Received unsupported event: id={}", event);
}

}  // namespace Payload
