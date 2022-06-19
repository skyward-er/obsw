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

#include <DeathStackStatus.h>
#include <LoggerService/LoggerService.h>
#include <Main/Actuators.h>
#include <Main/Bus.h>
#include <Main/Radio.h>
#include <Main/Sensors.h>
#include <Main/StateMachines.h>
#include <PinHandler/PinHandler.h>
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

#ifdef HARDWARE_IN_THE_LOOP
#include <hardware_in_the_loop/HIL.h>
#endif

using std::bind;

namespace DeathStackBoard
{

// Add heres the events that you don't want to be TRACEd in
// DeathStack.logEvent()
static const std::vector<uint8_t> TRACE_EVENT_BLACKLIST{
    EV_SEND_HR_TM, EV_SEND_LR_TM, EV_SEND_TEST_TM, EV_SEND_SENS_TM};
/**
 * This file provides a simplified way to initialize and monitor all
 * the components of the DeathStack.
 */
class DeathStack : public Boardcore::Singleton<DeathStack>
{
    friend class Singleton<DeathStack>;

public:
    // Shared Components
    LoggerService* logger;

    Boardcore::EventSniffer* sniffer;
    StateMachines* state_machines;

    Bus* bus;
    Sensors* sensors;
    Radio* radio;
    Actuators* actuators;

    PinHandler* pin_handler;

    Boardcore::TaskScheduler* scheduler;

#ifdef HARDWARE_IN_THE_LOOP
    HIL* hil;
#endif

    void start()
    {
        if (!sEventBroker.start())
        {
            LOG_ERR(log, "Error starting EventBroker");
            status.setError(&DeathStackStatus::ev_broker);
        }

        if (!radio->start())
        {
            LOG_ERR(log, "Error starting radio module");
            status.setError(&DeathStackStatus::radio);
        }

        if (!sensors->start())
        {
            LOG_ERR(log, "Error starting sensors");
            status.setError(&DeathStackStatus::sensors);
        }

        if (!state_machines->start())
        {
            LOG_ERR(log, "Error starting state machines");
            status.setError(&DeathStackStatus::state_machines);
        }

        if (!pin_handler->start())
        {
            LOG_ERR(log, "Error starting PinObserver");
            status.setError(&DeathStackStatus::pin_obs);
        }

#ifdef DEBUG
        injector->start();
#endif

        logger->log(status);

#ifdef HARDWARE_IN_THE_LOOP
        hil->start();
#endif

        // If there was an error, signal it to the FMM and light a LED.
        if (status.death_stack != COMP_OK)
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

    void startLogger()
    {
        try
        {
            logger->start();
            LOG_INFO(log, "Logger started");
        }
        catch (const std::runtime_error& re)
        {
            LOG_ERR(log, "SD Logger init error");
            status.setError(&DeathStackStatus::logger);
        }

        logger->log(logger->getLogger().getLoggerStats());
    }

private:
    /**
     * @brief Initialize Everything.
     */
    DeathStack()
    {
        /* Shared components */
        logger = &LoggerService::getInstance();
        startLogger();

        // Bind the logEvent function to the event sniffer in order to log every
        // event
        {
            using namespace std::placeholders;
            sniffer = new Boardcore::EventSniffer(
                sEventBroker, TOPIC_LIST,
                bind(&DeathStack::logEvent, this, _1, _2));
        }

#ifdef HARDWARE_IN_THE_LOOP
        hil = HIL::getInstance();
#endif

        scheduler = new Boardcore::TaskScheduler();
        addSchedulerStatsTask();

        bus       = new Bus();
        radio     = new Radio(*bus->spi2);
        sensors   = new Sensors(*bus->spi1, scheduler);
        actuators = new Actuators();

#ifdef HARDWARE_IN_THE_LOOP
        state_machines =
            new StateMachines(*sensors->hil_imu, *sensors->hil_baro,
                              *sensors->hil_gps, scheduler);
#elif defined(USE_MOCK_SENSORS)
        state_machines =
            new StateMachines(*sensors->mock_imu, *sensors->mock_baro,
                              *sensors->mock_gps, scheduler);
#else
        state_machines = new StateMachines(*sensors->imu_bmx160_with_correction,
                                           *sensors->press_static_port,
                                           *sensors->gps_ublox, scheduler);
#endif

        pin_handler = new PinHandler();

#ifdef DEBUG
        injector = new Boardcore::EventInjector();
#endif

        LOG_INFO(log, "Init finished");
    }

    /**
     * @brief Helpers for debugging purposes.
     */
    void logEvent(uint8_t event, uint8_t topic)
    {
        Boardcore::EventData ev{
            (long long)Boardcore::TimestampTimer::getTimestamp(), event, topic};
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
        LOG_DEBUG(log, "{:s} on {:s}", getEventString(event),
                  getTopicString(topic));
#endif
    }

    inline void postEvent(Boardcore::Event ev, uint8_t topic)
    {
        sEventBroker.post(ev, topic);
    }

    void addSchedulerStatsTask()
    {
        // add lambda to log scheduler tasks statistics
        scheduler->addTask(
            [&]()
            {
                std::vector<Boardcore::TaskStatsResult> scheduler_stats =
                    scheduler->getTaskStats();

                for (Boardcore::TaskStatsResult stat : scheduler_stats)
                    logger->log(stat);

                Boardcore::StackLogger::getInstance().updateStack(
                    THID_TASK_SCHEDULER);
            },
            1000, TASK_SCHEDULER_STATS_ID);
    }

    Boardcore::EventInjector* injector;
    DeathStackStatus status{};

    Boardcore::PrintLogger log = Boardcore::Logging::getLogger("deathstack");
};

}  // namespace DeathStackBoard
