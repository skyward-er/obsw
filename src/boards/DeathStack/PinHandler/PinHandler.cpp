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

#include "PinHandler.h"
#include <events/EventBroker.h>
#include <functional>
#include "DeathStack/events/Events.h"
#include "DeathStack/LoggerService/LoggerService.h"
#include "Debug.h"

using std::bind;

namespace DeathStackBoard
{

PinHandler::PinHandler()
    : pin_obs(PIN_POLL_INTERVAL), logger(LoggerService::getInstance())
{
    // Used for _1, _2. See std::bind cpp reference
    using namespace std::placeholders;

    // Launch pin callbacks registration
    PinObserver::OnTransitionCallback launch_transition_cb =
        bind(&PinHandler::onLaunchPinTransition, this, _1, _2);

    PinObserver::OnStateChangeCallback launch_statechange_cb =
        bind(&PinHandler::onLaunchPinStateChange, this, _1, _2, _3);

    pin_obs.observePin(PORT_LAUNCH_PIN, NUM_LAUNCH_PIN, TRIGGER_LAUNCH_PIN,
                       launch_transition_cb, THRESHOLD_LAUNCH_PIN,
                       launch_statechange_cb);

    // Noseconse pin callbacks registration
    PinObserver::OnTransitionCallback nc_transition_cb =
        bind(&PinHandler::onNCPinTransition, this, _1, _2);

    PinObserver::OnStateChangeCallback nc_statechange_cb =
        bind(&PinHandler::onNCPinStateChange, this, _1, _2, _3);

    pin_obs.observePin(PORT_NC_DETACH_PIN, NUM_NC_DETACH_PIN,
                       TRIGGER_NC_DETACH_PIN, nc_transition_cb,
                       THRESHOLD_NC_DETACH_PIN, nc_statechange_cb);

    // Motor pin callbacks registration
    PinObserver::OnTransitionCallback motor_transition_cb =
        bind(&PinHandler::onMotorPinTransition, this, _1, _2);

    PinObserver::OnStateChangeCallback motor_statechange_cb =
        bind(&PinHandler::onMotorPinStateChange, this, _1, _2, _3);
    

    pin_obs.observePin(PORT_MOTOR_PIN, NUM_MOTOR_PIN, TRIGGER_MOTOR_PIN,
                       motor_transition_cb, THRESHOLD_MOTOR_PIN,
                       motor_statechange_cb);
}

void PinHandler::onLaunchPinTransition(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    sEventBroker->post(Event{EV_UMBILICAL_DETACHED}, TOPIC_FLIGHT_EVENTS);

    status_pin_launch.last_detection_time = miosix::getTick();
    logger->log(status_pin_launch);
}

void PinHandler::onNCPinTransition(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    sEventBroker->post(Event{EV_NC_DETACHED}, TOPIC_FLIGHT_EVENTS);

    status_pin_nosecone.last_detection_time = miosix::getTick();
    logger->log(status_pin_nosecone);
}

void PinHandler::onMotorPinTransition(unsigned int p, unsigned char n)
{
    UNUSED(p);
    UNUSED(n);
    
    status_pin_motor.last_detection_time = miosix::getTick();
    logger->log(status_pin_motor);
}

void PinHandler::onLaunchPinStateChange(unsigned int p, unsigned char n,
                                                int state)
{
    UNUSED(p);
    UNUSED(n);

    status_pin_launch.state             = (uint8_t)state;
    status_pin_launch.last_state_change = miosix::getTick();
    status_pin_launch.num_state_changes += 1;
    logger->log(status_pin_launch);   
}

void PinHandler::onNCPinStateChange(unsigned int p, unsigned char n,
                                            int state)
{
    UNUSED(p);
    UNUSED(n);

    status_pin_nosecone.state             = (uint8_t)state;
    status_pin_nosecone.last_state_change = miosix::getTick();
    status_pin_nosecone.num_state_changes += 1;
    logger->log(status_pin_nosecone);
}

void PinHandler::onMotorPinStateChange(unsigned int p, unsigned char n,
                                                int state)
{
    UNUSED(p);
    UNUSED(n);
    status_pin_motor.state             = (uint8_t)state;
    status_pin_motor.last_state_change = miosix::getTick();
    status_pin_motor.num_state_changes += 1;
    logger->log(status_pin_motor);   
}

}  // namespace DeathStackBoard