/* Copyright (c) 2019-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

//#include <LoggerService/LoggerService.h>
#include <Payload/PinHandler/PinHandler.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <events/Events.h>
//#include <Payload/PayloadBoard.h>

#include <functional>

using std::bind;
using namespace Boardcore;

namespace PayloadBoard
{

PinHandler::PinHandler()
    : pin_obs(PIN_POLL_INTERVAL)  //, logger(LoggerService::getInstance())
{
    // Used for _1, _2. See std::bind cpp reference
    using namespace std::placeholders;

    // Noseconse pin callbacks registration
    PinObserver::OnTransitionCallback nc_transition_cb =
        bind(&PinHandler::onNCPinTransition, this, _1, _2);

    PinObserver::OnStateChangeCallback nc_statechange_cb =
        bind(&PinHandler::onNCPinStateChange, this, _1, _2, _3);

    pin_obs.observePin(nosecone_pin.getPort(), nosecone_pin.getNumber(),
                       TRIGGER_NC_DETACH_PIN, nc_transition_cb,
                       THRESHOLD_NC_DETACH_PIN, nc_statechange_cb);
}

void PinHandler::onNCPinTransition(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    sEventBroker.post(Event{EV_NC_DETACHED}, TOPIC_FLIGHT_EVENTS);

    LOG_INFO(log, "Nosecone detached!");

    status_pin_nosecone.last_detection_time =
        TimestampTimer::getInstance().getTimestamp();
    // logger->log(status_pin_nosecone);
}

void PinHandler::onNCPinStateChange(unsigned int p, unsigned char n, int state)
{
    UNUSED(p);
    UNUSED(n);

    status_pin_nosecone.state = (uint8_t)state;
    status_pin_nosecone.last_state_change =
        TimestampTimer::getInstance().getTimestamp();
    status_pin_nosecone.num_state_changes += 1;

    LOG_INFO(log, "Nosecone pin state change at time {}: new state = {}",
             status_pin_nosecone.last_state_change, status_pin_nosecone.state);

    // logger->log(status_pin_nosecone);
}

}  // namespace PayloadBoard