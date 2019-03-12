/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "PinObserverWrapper.h"
#include <events/EventBroker.h>
#include <functional>
#include "DeathStack/Events.h"
#include "DeathStack/LogProxy/LogProxy.h"

using std::bind;

namespace DeathStackBoard
{

PinObserverWrapper::PinObserverWrapper()
    : pin_obs(PIN_POLL_INTERVAL), logger(LoggerProxy::getInstance())
{
    // Used for _1, _2. See std::bind cpp reference
    using namespace std::placeholders;

    // Callback function: callback_launch(unsigned int, unsigned char)
    PinObserver::PinCallbackFn callback_launch =
        bind(&PinObserverWrapper::callbackLaunchPin, this, _1, _2);

    pin_obs.observePin(PORT_LAUNCH_PIN, NUM_LAUNCH_PIN, TRIGGER_LAUNCH_PIN,
                       callback_launch, THRESHOLD_LAUNCH_PIN);

    // Callback function: callback_nosecone(unsigned int, unsigned char)
    PinObserver::PinCallbackFn callback_nosecone =
        bind(&PinObserverWrapper::callbackNoseconePin, this, _1, _2);

    pin_obs.observePin(PORT_NC_DETACH_PIN, NUM_NC_DETACH_PIN,
                       TRIGGER_NC_DETACH_PIN, callback_nosecone,
                       THRESHOLD_NC_DETACH_PIN);
}

void PinObserverWrapper::callbackLaunchPin(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    sEventBroker->post(Event{EV_UMBILICAL_DETACHED}, TOPIC_FMM);

    ++status_pin_launch.num_triggered;
    logger->log(status_pin_launch);
}

void PinObserverWrapper::callbackNoseconePin(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    sEventBroker->post(Event{EV_NC_DETACHED}, TOPIC_FMM);

    ++status_pin_nosecone.num_triggered;
    logger->log(status_pin_nosecone);
}

}  // namespace DeathStackBoard