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

#include <functional>
#include <stdexcept>
#include <vector>

#include <Common.h>

#include <events/EventBroker.h>
#include <utils/EventSniffer.h>

#include "DeathStack/events/Events.h"
#include "DeathStack/LoggerService/TmRepository.h"
#include "DeathStack/events/Topics.h"
#include "DeathStackStatus.h"

#include "DeathStack/LoggerService/LoggerService.h"

#include "DeathStack/ADA/ADA.h"
#include "DeathStack/DeploymentController/Deployment.h"
#include "DeathStack/FlightModeManager/FlightModeManager.h"
#include "DeathStack/PinHandler/PinHandler.h"
#include "DeathStack/SensorManager/SensorManager.h"
#include "DeathStack/events/EventData.h"
#include "DeathStack/TMTCManager/TMTCManager.h"

using std::bind;

namespace DeathStackBoard
{

// Add heres the event that you don't want to be TRACEd in DeathStack.logEvent()
static const std::vector<uint8_t> TRACE_EVENT_BLACKLIST{EV_SEND_HR_TM,
                                                        EV_SEND_LR_TM};
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
    PinHandler* pin_observer;
    EventSniffer* sniffer;

    // FSMs
    ADA* ada;
    SensorManager* sensor_manager;
    TMTCManager* tmtc;
    FlightModeManager* fmm;
    DeploymentController* dpl;

private:
    /**
     * Initialize Everything
     */
    DeathStack()
    {
        memset(&status, 0, sizeof(status));

        /* Shared components */
        broker       = sEventBroker;
        logger       = Singleton<LoggerService>::getInstance();
        pin_observer = new PinHandler();

        // Bind the logEvent function to the event sniffer in order to log every
        // event
        {
            using namespace std::placeholders;
            sniffer = new EventSniffer(
                *broker, TOPIC_LIST, bind(&DeathStack::logEvent, this, _1, _2));
        }

        ada            = new ADA();
        sensor_manager = new SensorManager(ada);
        tmtc           = new TMTCManager();
        fmm            = new FlightModeManager();
        dpl            = new DeploymentController();

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

        if (!broker->start())
        {
            status.setError(&DeathStackStatus::ev_broker);
        }

        if (!pin_observer->start())
        {
            status.setError(&DeathStackStatus::pin_obs);
        }

        /* State Machines */
        if (!fmm->start())
        {
            status.setError(&DeathStackStatus::fmm);
        }
        if (!tmtc->start())
        {
            status.setError(&DeathStackStatus::tmtc);
        }
        if (!dpl->start())
        {
            status.setError(&DeathStackStatus::dpl);
        }
        if (!ada->start())
        {
            status.setError(&DeathStackStatus::ada);
        }
        if (!sensor_manager->start())
        {
            status.setError(&DeathStackStatus::sensor_manager);
        }
        TRACE("[DS] Sensor init status: %02X %d\n",
              sensor_manager->getStatus().sensor_status,
              sensor_manager->getStatus().sensor_status);
        if (sensor_manager->getStatus().sensor_status !=
            NOMINAL_SENSOR_INIT_VALUE)
        {
            status.setError(&DeathStackStatus::sensor_manager);
        }

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
    DeathStackStatus status;
};

} /* namespace DeathStackBoard */