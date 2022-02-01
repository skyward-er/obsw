/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Alvise de'Faveri Tron
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

#include <Payload/PayloadStatus.h>
//#include <LoggerService/LoggerService.h>
#include <Payload/Main/Actuators.h>
#include <Payload/Main/Bus.h>
#include <Payload/Main/Radio.h>
#include <Payload/Main/Sensors.h>
//#include <Payload/Main/StateMachines.h>
#include <Payload/PinHandler/PinHandler.h>
#include <System/StackLogger.h>
#include <System/TaskID.h>
#include <events/EventBroker.h>
#include <events/EventData.h>
#include <events/Events.h>
#include <events/Topics.h>
#include <events/utils/EventInjector.h>
#include <events/utils/EventSniffer.h>

#include <functional>
#include <stdexcept>
#include <vector>

using std::bind;

namespace PayloadBoard
{

// Add heres the events that you don't want to be TRACEd in
// Payload.logEvent()
static const std::vector<uint8_t> TRACE_EVENT_BLACKLIST{
    EV_SEND_HR_TM, EV_SEND_LR_TM, EV_SEND_TEST_TM, EV_SEND_SENS_TM};
/**
 * This file provides a simplified way to initialize and monitor all
 * the components of the Payload.
 */
class Payload : public Boardcore::Singleton<Payload>
{
    friend class Boardcore::Singleton<Payload>;

public:
    // Shared Components
    Boardcore::EventBroker* broker;
    // LoggerService* logger;

    Boardcore::EventSniffer* sniffer;

    // StateMachines* state_machines;
    Bus* bus;
    Sensors* sensors;
    Radio* radio;
    Actuators* actuators;

    PinHandler* pin_handler;

    Boardcore::TaskScheduler* scheduler;

    void start()
    {
        if (!broker->start())
        {
            LOG_ERR(log, "Error starting EventBroker");
            status.setError(&PayloadStatus::ev_broker);
        }

        /*if (!radio->start())
        {
            LOG_ERR(log, "Error starting radio module");
            status.setError(&PayloadStatus::radio);
        }*/

        if (!sensors->start())
        {
            LOG_ERR(log, "Error starting sensors");
            status.setError(&PayloadStatus::sensors);
        }

        /*if (!state_machines->start())
        {
            LOG_ERR(log, "Error starting state machines");
            status.setError(&PayloadStatus::state_machines);
        }*/

        if (!pin_handler->start())
        {
            LOG_ERR(log, "Error starting PinObserver");
            status.setError(&PayloadStatus::pin_obs);
        }

#ifdef DEBUG
        injector->start();
#endif

        // logger->log(status);

        // If there was an error, signal it to the FMM and light a LED.
        if (status.payload_board != COMP_OK)
        {
            LOG_ERR(log, "Initalization failed\n");
            sEventBroker.post(Boardcore::Event{EV_INIT_ERROR},
                              TOPIC_FLIGHT_EVENTS);
        }
        else
        {
            LOG_INFO(log, "Initalization ok");
            sEventBroker.post(Boardcore::Event{EV_INIT_OK},
                              TOPIC_FLIGHT_EVENTS);
        }
    }

    /*void startLogger()
    {
        try
        {
            logger->start();
            LOG_INFO(log, "Logger started");
        }
        catch (const std::runtime_error& re)
        {
            LOG_ERR(log, "SD Logger init error");
            status.setError(&PayloadStatus::logger);
        }

        logger->log(logger->getLogger().getLoggerStats());
    }*/

private:
    /**
     * @brief Initialize Everything.
     */
    Payload()
    {
        /* Shared components */
        // logger = Singleton<LoggerService>::getInstance();
        // startLogger();

        broker = &sEventBroker;

        // Bind the logEvent function to the event sniffer in order to log every
        // event
        /*{
            using namespace std::placeholders;
            sniffer = new EventSniffer(
                *broker, TOPIC_LIST, bind(&Payload::logEvent, this, _1, _2));
        }*/

        scheduler = new Boardcore::TaskScheduler();

        bus       = new Bus();
        radio     = new Radio(*bus->spi2);
        sensors   = new Sensors(*bus->spi1, scheduler);
        actuators = new Actuators();

        pin_handler = new PinHandler();

#ifdef DEBUG
        injector = new Boardcore::EventInjector();
#endif

        LOG_INFO(log, "Init finished");
    }

    /**
     * @brief Helpers for debugging purposes.
     */
    /*void logEvent(uint8_t event, uint8_t topic)
    {
        EventData ev{(long long)TimestampTimer::getInstance().getTimestamp(),
event, topic}; logger->log(ev);

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
        LOG_DEBUG(log, "{:s} on {:s}",
getEventString(event), getTopicString(topic)); #endif
    }*/

    inline void postEvent(Boardcore::Event ev, uint8_t topic)
    {
        broker->post(ev, topic);
    }

    /*void addSchedulerStatsTask()
    {
        // add lambda to log scheduler tasks statistics
        scheduler.add(
            [&]() {
                std::vector<TaskStatsResult> scheduler_stats =
                    scheduler.getTaskStats();

                for (TaskStatsResult stat : scheduler_stats)
                {
                    logger->log(stat);
                }

                StackLogger::getInstance().updateStack(THID_TASK_SCHEDULER);
            },
            1000,  // 1 hz
            TASK_SCHEDULER_STATS_ID, miosix::getTick());
    }*/

    Boardcore::EventInjector* injector;
    PayloadStatus status{};

    Boardcore::PrintLogger log = Boardcore::Logging::getLogger("Payload");
};

}  // namespace PayloadBoard
