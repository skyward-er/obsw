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
#include <Debug.h>
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
#include "Main/StateMachines.h"
#include "PinHandler/PinHandler.h"
#include "System/StackLogger.h"
#include "events/EventData.h"
#include "events/EventInjector.h"
#include "events/Events.h"
#include "events/Topics.h"
using std::bind;

namespace DeathStackBoard
{

// Add heres the event that you don't want to be TRACEd in DeathStack.logEvent()
static const std::vector<uint8_t> TRACE_EVENT_BLACKLIST{
    EV_SEND_HR_TM, EV_SEND_LR_TM, EV_SEND_TEST_TM, EV_SEND_SENS_TM};
/**
 * This file provides a simplified way to initialize and monitor all
 * the components of the DeathStack.
 */
class DeathStack : public Singleton<DeathStack>
{
    friend class Singleton<DeathStack>;

public:
    // PrintLogger log = Logging::getLogger("deathstack");

    // Shared Components
    EventBroker* broker;
    LoggerService* logger;

    EventSniffer* sniffer;
    StateMachines* state_machines;

    Bus* bus;
    Sensors* sensors;
    Radio* radio;
    Actuators* actuators;

    PinHandler* pin_handler;

    TaskScheduler* scheduler;

    void start()
    {
        logger->log(logger->getLogger().getLogStats());

        if (!broker->start())
        {
            TRACE("Error starting eventbroker\n");

            status.setError(&DeathStackStatus::ev_broker);
        }

        radio->start();
        sensors->start();

        logger->log(status);
        state_machines->start();
        injector->start();

        pin_handler->start();

        // If there was an error, signal it to the FMM and light a LED.
        // if (status.death_stack != COMP_OK)
        // {
        //     sEventBroker->post(Event{EV_INIT_ERROR}, TOPIC_FLIGHT_EVENTS);
        // }
        // else
        // {
        //     sEventBroker->post(Event{EV_INIT_OK}, TOPIC_FLIGHT_EVENTS);
        // }
    }

private:
    /**
     * Initialize Everything
     */
    DeathStack()
    {
        /* Shared components */
        logger = Singleton<LoggerService>::getInstance();
        // Start threads
        try
        {
            logger->start();
        }
        catch (const std::runtime_error& re)
        {
            TRACE("SD Logger init error\n");
            status.setError(&DeathStackStatus::logger);
        }

        TimestampTimer::enableTimestampTimer();

        broker = sEventBroker;

        // Bind the logEvent function to the event sniffer in order to log every
        // event
        {
            using namespace std::placeholders;
            sniffer = new EventSniffer(
                *broker, TOPIC_LIST, bind(&DeathStack::logEvent, this, _1, _2));
        }

        scheduler = new TaskScheduler();
        bus       = new Bus();
        radio     = new Radio(*bus->spi2);
        sensors   = new Sensors(*bus->spi1, scheduler);
        actuators = new Actuators();
        state_machines =
            new StateMachines(*sensors->imu_bmx160, *sensors->press_digital,
                              *sensors->gps_ublox, scheduler);
        pin_handler = new PinHandler();

        injector = new EventInjector();
        // LOG_INFO(log, "Init finished");
        TRACE("Init finished\n");
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
        // LOG_DEBUG(log, "{:s} on {:s}", getEventString(event),
        // getTopicString(topic));
        TRACE("%s on %s", getEventString(event).c_str(),
              getTopicString(topic).c_str());
#endif
    }

    inline void postEvent(Event ev, uint8_t topic) { broker->post(ev, topic); }

private:
    EventInjector* injector;
    DeathStackStatus status{};
};

} /* namespace DeathStackBoard */
