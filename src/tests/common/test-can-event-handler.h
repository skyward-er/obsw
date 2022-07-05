/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Federico Mandelli
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
#pragma once

#include <common/canbus/CanHandler.h>

#include "events/EventBroker.h"
#include "events/EventHandler.h"

class MyEventHandler : public EventHandler
{
public:
    MyEventHandler()
        : EventHandler(),  // call parent constructor
          last_event(0)
    {
        // make this object to subscribe to TOPIC_CAN_EVENTS
        EventBroker::getInstance().subscribe(this, CanTopics::TOPIC_CAN_EVENTS);
        EventBroker::getInstance().subscribe(this,
                                             CanTopics::TOPIC_CAN_COMMAND);
    }

    ~MyEventHandler()
    {
        // unsubscribe from all the topics this object was subscribed to
        EventBroker::getInstance().unsubscribe(this);
    }

protected:
    void handleEvent(const Event& ev) override
    {
        switch (ev)
        {
            case CanEv::EV_LIFTOFF:
                TRACE("Received EV_LIFTOFF \n");
                break;
            case CanEv::EV_APOGEE:
                TRACE("Received EV_APOGEE \n");
                break;
            case CanEv::EV_ARMED:
                TRACE("Received EV_ARMED \n");
                break;
            case CanEv::EV_AEROBRAKE:
                TRACE("Received EV_AEROBRAKE \n");
                break;
            default:
                TRACE("Invalid event \n");
        }

        last_event = ev;
    }

private:
    uint8_t last_event;
};
