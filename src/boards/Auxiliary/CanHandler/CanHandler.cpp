/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli, Alberto Nidasio
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

#include <Auxiliary/Actuators/Actuators.h>
#include <common/CanConfig.h>

#include <functional>

using namespace std;
using namespace placeholders;
using namespace Boardcore;
using namespace Canbus;
using namespace Common::CanConfig;

namespace Auxiliary
{

bool CanHandler::start() { return protocol->start(); }

bool CanHandler::isStarted() { return protocol->isStarted(); }

CanHandler::CanHandler()
{
    CanbusDriver::AutoBitTiming bitTiming;
    bitTiming.baudRate    = BAUD_RATE;
    bitTiming.samplePoint = SAMPLE_POINT;
    driver                = new CanbusDriver(CAN1, {}, bitTiming);

    protocol =
        new CanProtocol(driver, bind(&CanHandler::handleCanMessage, this, _1));

    protocol->addFilter(static_cast<uint8_t>(Board::Main),
                        static_cast<uint8_t>(Board::Broadcast));
    protocol->addFilter(static_cast<uint8_t>(Board::Main),
                        static_cast<uint8_t>(Board::Auxiliary));
    driver->init();

    printf("Init done\n");
}

void CanHandler::handleCanMessage(const CanMessage &msg)
{
    PrimaryType msgType = static_cast<PrimaryType>(msg.getPrimaryType());

    printf("Received message\n");

    switch (msgType)
    {
        case PrimaryType::Events:
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

    printf("Handling event\n");

    switch (eventId)
    {
        case EventId::Armed:
        case EventId::CamOn:
        {
            Actuators::getInstance().ledOn();
            // Actuators::getInstance().camOn();
            LOG_DEBUG(logger, "Cameras and leds turned on");
            break;
        }
        case EventId::Disarmed:
        case EventId::CamOff:
        {
            Actuators::getInstance().ledOff();
            // Actuators::getInstance().camOff();
            LOG_DEBUG(logger, "Cameras and leds turned off");
            break;
        }

        default:
        {
            LOG_DEBUG(logger, "Received unsupported event: id={}", eventId);
        }
    }
}

}  // namespace Auxiliary
