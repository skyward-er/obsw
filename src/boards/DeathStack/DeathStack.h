/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de'Faveri Tron
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

#pragma once

#include <Common.h>
#include <events/EventBroker.h>
#include <events/utils/EventSniffer.h>

#include <functional>
#include <stdexcept>
#include <vector>

#include "DeathStackStatus.h"
#include "LoggerService/LoggerService.h"
#include "Main/Actuators.h"
#include "Main/Bus.h"
#include "Main/Radio.h"
#include "Main/Sensors.h"
#include "System/StackLogger.h"
#include "events/EventData.h"
#include "events/Events.h"
#include "events/Topics.h"

using std::bind;

namespace DeathStackBoard
{

// Add heres the event that you don't want to be TRACEd in DeathStack.logEvent()
static const std::vector<uint8_t> TRACE_EVENT_BLACKLIST{
    EV_SEND_HR_TM, EV_SEND_LR_TM, EV_SEND_TEST_TM, EV_SEND_TUNNEL_TM};
/**
 * This file provides a simplified way to initialize and monitor all
 * the components of the DeathStack.
 */
class DeathStack : public Singleton<DeathStack>
{
    friend class Singleton<DeathStack>;

public:
    // Shared Components
    EventBroker* broker;
    LoggerService* logger;

    // PinHandler* pin_observer;
    EventSniffer* sniffer;

    Bus* bus;
    Sensors* sensors;
    Radio* radio;
    Actuators* actuators;

    void start()
    {
        // Start threads
        try
        {
            logger->start();
        }
        catch (const std::runtime_error& re)
        {
            TRACE("Logger init error\n");
            status.setError(&DeathStackStatus::logger);
        }

        logger->log(logger->getLogger().getLogStats());

        if (!broker->start())
        {
            status.setError(&DeathStackStatus::ev_broker);
        }

        radio->start();
        sensors->start();

        logger->log(status);

        // If there was an error, signal it to the FMM and light a LED.
        if (status.death_stack != COMP_OK)
        {
            sEventBroker->post(Event{EV_INIT_ERROR}, TOPIC_FLIGHT_EVENTS);
        }
        else
        {
            sEventBroker->post(Event{EV_INIT_OK}, TOPIC_FLIGHT_EVENTS);
        }
    }

private:
    /**
     * Initialize Everything
     */
    DeathStack()
    {
        /* Shared components */
        logger = Singleton<LoggerService>::getInstance();
        broker = sEventBroker;

        // Bind the logEvent function to the event sniffer in order to log every
        // event
        {
            using namespace std::placeholders;
            sniffer = new EventSniffer(
                *broker, TOPIC_LIST, bind(&DeathStack::logEvent, this, _1, _2));
        }

        bus       = new Bus();
        radio     = new Radio(*bus->spi2);
        sensors   = new Sensors(*bus->spi1);
        actuators = new Actuators();

        TRACE("[DS] Init finished\n");
    }

    /**
     * Helpers for debugging purposes
     */
    void logEvent(uint8_t event, uint8_t topic)
    {
        EventData ev{miosix::getTick(), event, topic};
        logger->log(ev);

#ifdef DEBUG
        // Don't TRACE if event is in the blacklist to avoid cluttering the
        // serial terminal
        for (uint8_t bl_ev : TRACE_EVENT_BLACKLIST)
        {
            if (bl_ev == event)
            {
                return;
            }
        }
        TRACE("[%.3f] %s on %s\n", (miosix::getTick() / 1000.0f),
              getEventString(event).c_str(), getTopicString(topic).c_str());
#endif
    }

    inline void postEvent(Event ev, uint8_t topic) { broker->post(ev, topic); }

private:
    DeathStackStatus status{};
};

} /* namespace DeathStackBoard */