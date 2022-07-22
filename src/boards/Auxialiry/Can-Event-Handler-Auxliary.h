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

#include <common/canbus/CanConfig.h>
#include <events/EventBroker.h>
#include <events/EventHandler.h>

class MyEventHandler : public Boardcore::EventHandler
{
public:
    MyEventHandler()
        : EventHandler(),  // call parent constructor
          last_event(0)
    {
        // make this object to subscribe to TOPIC_CAN_EVENTS
        Boardcore::EventBroker::getInstance().subscribe(
            this, common::CanTopics::TOPIC_CAN_EVENTS);
    }

    ~MyEventHandler()
    {
        // unsubscribe from all the topics this object was subscribed to
        Boardcore::EventBroker::getInstance().unsubscribe(this);
    }

protected:
    void handleEvent(const Boardcore::Event& ev) override
    {
        switch (ev)
        {
            case common::CanEvent::EV_LIFTOFF:
                // Led.status(common::CanEvent::EV_LIFTOFF);
                break;
            case common::CanEvent::EV_APOGEE:

                break;
            case common::CanEvent::EV_ARMED:
                // Led.status(common::CanEvent::EV_ARMED);
                // Runcam.startRecording();//check if the cam1 is recording
                break;
            default:
                TRACE("Invalid event \n");
        }

        last_event = ev;
    }

private:
    uint8_t last_event;
};
