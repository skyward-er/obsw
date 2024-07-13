/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/CanHandlerConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/FlightModeManager/FlightModeManager.h>
#include <common/Events.h>
#include <events/EventBroker.h>

#include <functional>

using namespace Boardcore;
using namespace Canbus;
using namespace Common;
using namespace CanConfig;
using namespace Payload::CanHandlerConfig;

namespace Payload
{

CanHandler::CanHandler()
{
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;

    CanbusDriver::CanbusConfig config;
    driver = new CanbusDriver(CAN2, config, bitTiming);

    protocol = new CanProtocol(
        driver, [this](auto msg) { handleCanMessage(msg); },
        miosix::PRIORITY_MAX - 1);

    // Accept messages only from the main and RIG board
    protocol->addFilter(static_cast<uint8_t>(Board::MAIN),
                        static_cast<uint8_t>(Board::BROADCAST));
    protocol->addFilter(static_cast<uint8_t>(Board::RIG),
                        static_cast<uint8_t>(Board::BROADCAST));
}

bool CanHandler::start()
{
    auto& scheduler = getModule<BoardScheduler>()->canHandler();
    bool result;

    // Add a task to periodically send the pitot data
    result = scheduler.addTask(  // sensor template
                 [&]()
                 {
                     protocol->enqueueData(
                         static_cast<uint8_t>(Priority::MEDIUM),
                         static_cast<uint8_t>(PrimaryType::SENSORS),
                         static_cast<uint8_t>(Board::PAYLOAD),
                         static_cast<uint8_t>(Board::BROADCAST),
                         static_cast<uint8_t>(SensorId::PITOT),
                         getModule<Sensors>()->getPitotLastSample());
                 },
                 PITOT_TRANSMISSION_PERIOD) != 0;

    result =
        result &&
        scheduler.addTask(  // status
            [&]()
            {
                FlightModeManagerState state =
                    getModule<FlightModeManager>()->getStatus().state;
                protocol->enqueueSimplePacket(
                    static_cast<uint8_t>(Priority::MEDIUM),
                    static_cast<uint8_t>(PrimaryType::STATUS),
                    static_cast<uint8_t>(Board::PAYLOAD),
                    static_cast<uint8_t>(Board::BROADCAST),
                    static_cast<uint8_t>(state),
                    ((state == FlightModeManagerState::ARMED) ? 0x01 : 0x00));
            },
            STATUS_TRANSMISSION_PERIOD) != 0;

    driver->init();

    return protocol->start() && result;
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

void CanHandler::handleCanMessage(const CanMessage& msg)
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
            LOG_WARN(logger, "Received unsupported message type: type={}",
                     msgType);
            break;
        }
    }
}

void CanHandler::handleCanEvent(const Boardcore::Canbus::CanMessage& msg)
{
    EventId eventId = static_cast<EventId>(msg.getSecondaryType());

    auto it = eventToEvent.find(eventId);

    if (it != eventToEvent.end())
    {
        EventBroker::getInstance().post(it->second, TOPIC_CAN);
    }
    else
        LOG_WARN(logger, "Received unsupported event: id={}", eventId);
}

}  // namespace Payload
