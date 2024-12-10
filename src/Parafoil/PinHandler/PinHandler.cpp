
/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/PinHandlerConfig.h>
#include <common/Events.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <interfaces-impl/hwmapping.h>

#include "PinData.h"

using namespace Boardcore;
using namespace Common;
namespace config = Parafoil::Config::PinHandler;
namespace hwmap  = miosix::inputs;

namespace Parafoil
{

const decltype(PinHandler::PIN_LIST) PinHandler::PIN_LIST = {
    PinList::NOSECONE_PIN,
};

bool PinHandler::start()
{
    using namespace std::chrono;

    auto& scheduler = getModule<BoardScheduler>()->pinHandler();

    pinObserver = std::make_unique<PinObserver>(
        scheduler, milliseconds{config::PinObserver::PERIOD}.count());

    bool expulsionPinDetachResult = pinObserver->registerPinCallback(
        hwmap::expulsion::getPin(),
        [this](auto t) { onExpulsionPinTransition(t); },
        config::Expulsion::DETECTION_THRESHOLD);

    if (!expulsionPinDetachResult)
    {
        LOG_ERR(logger,
                "Failed to register pin callback for the detach ramp pin");
        return false;
    }

    started = true;
    return true;
}

bool PinHandler::isStarted() { return started; }

PinData PinHandler::getPinData(PinList pin)
{
    switch (pin)
    {
        case PinList::NOSECONE_PIN:
            return pinObserver->getPinData(hwmap::expulsion::getPin());
        default:
            return PinData{};
    }
}

void PinHandler::logPin(PinList pin)
{
    auto pinData       = getPinData(pin);
    auto pinChangeData = PinChangeData{
        .timestamp    = TimestampTimer::getTimestamp(),
        .pinId        = static_cast<uint8_t>(pin),
        .lastState    = pinData.lastState,
        .changesCount = pinData.changesCount,
    };
    Logger::getInstance().log(pinChangeData);
}

void PinHandler::onExpulsionPinTransition(PinTransition transition)
{
    logPin(PinList::NOSECONE_PIN);
    LOG_INFO(logger, "Expulsion Pin transition detected: {}",
             static_cast<int>(transition));

    if (transition == config::Expulsion::TRIGGERING_TRANSITION)
        EventBroker::getInstance().post(FLIGHT_NC_DETACHED, TOPIC_FLIGHT);
}

}  // namespace Parafoil
