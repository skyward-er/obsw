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

#include <common/CanConfig.h>
#include <common/events/Events.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace Boardcore;
using namespace Canbus;
using namespace Common;
using namespace CanConfig;

namespace Main
{

bool CanHandler::start() { return protocol->start(); }

bool CanHandler::isStarted() { return protocol->isStarted(); }

void CanHandler::sendArmEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::MAIN),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::ARM));
}

void CanHandler::sendDisarmEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::MAIN),
                           static_cast<uint8_t>(Board::BROADCAST),
                           static_cast<uint8_t>(EventId::DISARM));
}

void CanHandler::sendCamOnEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::MAIN),
                           static_cast<uint8_t>(Board::AUXILIARY),
                           static_cast<uint8_t>(EventId::CAM_ON));
}

void CanHandler::sendCamOffEvent()
{
    protocol->enqueueEvent(static_cast<uint8_t>(Priority::CRITICAL),
                           static_cast<uint8_t>(PrimaryType::EVENTS),
                           static_cast<uint8_t>(Board::MAIN),
                           static_cast<uint8_t>(Board::AUXILIARY),
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

    // protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
    //                     static_cast<uint8_t>(Board::BROADCAST));
    // protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
    //                     static_cast<uint8_t>(Board::AUXILIARY));
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

        default:
        {
            break;
        }
    }
}

void CanHandler::handleCanEvent(const CanMessage &msg)
{
    EventId eventId = static_cast<EventId>(msg.getSecondaryType());

    switch (eventId)
    {
        case EventId::ARM:
        {
            EventBroker::getInstance().post(TMTC_ARM, TOPIC_TMTC);
            break;
        }
        case EventId::DISARM:
        {
            EventBroker::getInstance().post(TMTC_DISARM, TOPIC_TMTC);
            break;
        }

        default:
        {
            LOG_DEBUG(logger, "Received unsupported event: id={}", eventId);
        }
    }
}

void CanHandler::handleCanSensor(const CanMessage &msg)
{
    SensorId sensorId = static_cast<SensorId>(msg.getSecondaryType());

    switch (sensorId)
    {
        case SensorId::PITOT:
        {
            break;
        }

        default:
        {
            LOG_DEBUG(logger, "Received unsupported sensor data: id={}",
                      sensorId);
        }
    }
}

}  // namespace Main
