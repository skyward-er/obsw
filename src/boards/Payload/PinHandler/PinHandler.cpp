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
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include <functional>

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

    bool detachPayloadResult = pinObserver->registerPinCallback(
        hwmap::detachPayload::getPin(),
        [this](auto t) { onDetachPayloadTransition(t); },
        config::NoseconeDetach::DETECTION_THRESHOLD);

    if (!detachPayloadResult)
    {
        LOG_ERR(logger,
                "Failed to register pin callback for the detach payload pin");
        return false;
    }

    bool expSenseResult = pinObserver->registerPinCallback(
        hwmap::expulsionSense::getPin(),
        [this](auto t) { onExpulsionSenseTransition(t); },
        config::ExpulsionSense::DETECTION_THRESHOLD);

    if (!expSenseResult)
    {
        LOG_ERR(logger,
                "Failed to register pin callback for the expulsion sense pin");
        return false;
    }

    started = true;
    return true;
}

bool PinHandler::isStarted() { return started; }

void PinHandler::onDetachPayloadTransition(PinTransition transition)
{
    // TODO: send event
    LOG_INFO(logger, "onDetachPayloadTransition {}",
             static_cast<int>(transition));
}

void PinHandler::onExpulsionSenseTransition(PinTransition transition)
{
    // TODO: send event
    LOG_INFO(logger, "onExpulsionSenseTransition {}",
             static_cast<int>(transition));
}

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::DETACH_PAYLOAD_PIN:
            return pinObserver->getPinData(hwmap::detachPayload::getPin());
        case PinList::EXPULSION_SENSE:
            return pinObserver->getPinData(hwmap::expulsionSense::getPin());
        default:
            return PinData{};
    }
}

}  // namespace Payload
