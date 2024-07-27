/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include "PinHandler.h"

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/PinHandlerConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "PinData.h"

using namespace Boardcore;
using namespace Common;
namespace config = Payload::Config::PinHandler;
namespace hwmap  = miosix::sense;

namespace Payload
{

bool PinHandler::start()
{
    using namespace std::chrono;

    auto& scheduler = getModule<BoardScheduler>()->pinHandler();

    pinObserver = std::make_unique<PinObserver>(
        scheduler, milliseconds{config::PinObserver::PERIOD}.count());

    bool rampPinDetachResult = pinObserver->registerPinCallback(
        hwmap::detachRamp::getPin(),
        [this](auto t) { onRampDetachTransition(t); },
        config::RampDetach::DETECTION_THRESHOLD);

    if (!rampPinDetachResult)
    {
        LOG_ERR(logger,
                "Failed to register pin callback for the detach ramp pin");
        return false;
    }

    bool noseconeDetachResult = pinObserver->registerPinCallback(
        hwmap::detachPayload::getPin(),
        [this](auto t) { onNoseconeDetachTransition(t); },
        config::NoseconeDetach::DETECTION_THRESHOLD);

    if (!noseconeDetachResult)
    {
        LOG_ERR(logger,
                "Failed to register pin callback for the detach payload pin");
        return false;
    }

    started = true;
    return true;
}

bool PinHandler::isStarted() { return started; }

void PinHandler::onRampDetachTransition(PinTransition transition)
{
    if (transition == config::RampDetach::TRIGGERING_TRANSITION)
    {
        EventBroker::getInstance().post(FLIGHT_LAUNCH_PIN_DETACHED,
                                        TOPIC_FLIGHT);

        auto pinData       = getPinData(PinList::RAMP_DETACH_PIN);
        auto pinChangeData = PinChangeData{
            .timestamp    = TimestampTimer::getTimestamp(),
            .pinID        = static_cast<uint8_t>(PinList::RAMP_DETACH_PIN),
            .changesCount = pinData.changesCount,
        };
        Logger::getInstance().log(pinChangeData);
    }
}

void PinHandler::onNoseconeDetachTransition(PinTransition transition)
{
    if (transition == config::NoseconeDetach::TRIGGERING_TRANSITION)
    {
        EventBroker::getInstance().post(FLIGHT_NC_DETACHED, TOPIC_FLIGHT);

        auto pinData       = getPinData(PinList::NOSECONE_DETACH_PIN);
        auto pinChangeData = PinChangeData{
            .timestamp    = TimestampTimer::getTimestamp(),
            .pinID        = static_cast<uint8_t>(PinList::NOSECONE_DETACH_PIN),
            .changesCount = pinData.changesCount,
        };
        Logger::getInstance().log(pinChangeData);
    }
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::RAMP_DETACH_PIN:
            return pinObserver->getPinData(hwmap::detachRamp::getPin());
        case PinList::NOSECONE_DETACH_PIN:
            return pinObserver->getPinData(hwmap::detachPayload::getPin());
        default:
            return PinData{};
    }
}

}  // namespace Payload
