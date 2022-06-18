/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Payload/Main/Radio.h>
#include <Payload/Main/Sensors.h>
#include <Payload/PayloadStatus.h>
#include <drivers/spi/SPIDriver.h>
#include <events/EventBroker.h>
#include <scheduler/TaskScheduler.h>

namespace Payload
{

enum ThreadIds : uint8_t
{
    THID_ENTRYPOINT = Boardcore::THID_FIRST_AVAILABLE_ID,
    THID_FMM_FSM,
    THID_TMTC_FSM,
    THID_STATS_FSM,
    THID_ADA_FSM,
    THID_NAS_FSM,
    THID_TASK_SCHEDULER
};

enum TaskIDs : uint8_t
{
    TASK_SCHEDULER_STATS_ID = 0,
    TASK_SENSORS_6_MS_ID    = 1,
    TASK_SENSORS_15_MS_ID   = 2,
    TASK_SENSORS_20_MS_ID   = 3,
    TASK_SENSORS_24_MS_ID   = 4,
    TASK_SENSORS_40_MS_ID   = 5,
    TASK_SENSORS_1000_MS_ID = 6,
    TASK_ADA_ID             = 7,
    TASK_NAS_ID             = 9
};

/**
 * @brief This class represents the main singleton that keeps all the project
 * objects. It has all the instances and its duty is to initialize them.
 *
 */
class Payload : public Boardcore::Singleton<Payload>
{
    friend class Boardcore::Singleton<Payload>;

public:
    // Event broker used all around the code for the event pattern
    Boardcore::EventBroker* broker;

    // Task scheduler to schedule the sensors sampling over time
    Boardcore::TaskScheduler* sensorsScheduler;

    // Task scheduler to schedule the algorithm processing
    Boardcore::TaskScheduler* algorithmScheduler;

    // Task scheduler to schedule the radio minor tasks about automatic
    // telemetry frequency change
    Boardcore::TaskScheduler* radioScheduler;

    // General purpose scheduler
    Boardcore::TaskScheduler* generalScheduler;

    // Collection of all the sensors. It samples it automatically through the
    // task scheduler
    Sensors* sensors;

    // Radio class that manages all the comunication stuff and handles all the
    // ground station requests
    Radio* radio;

    /**
     * @brief Method to start all the macro obsw elements
     */
    void start()
    {
        // First thing is starting the SDlogger
        // TODO: Verify difference betweeen if and try catch
        if (!SDlogger->start())
        {
            LOG_ERR(logger, "Error starting the SD logger");
            status.setError(&PayloadStatus::logger);
        }

        // Log the SDlogger status
        SDlogger->log(SDlogger->getStats());

        // Start the general purpose scheduler
        if (!generalScheduler->start())
        {
            LOG_ERR(logger, "Error starting the general purpose scheduler");
        }

        // Start the sensors scheduler
        if (!sensorsScheduler->start())
        {
            LOG_ERR(logger, "Error starting the sensors scheduler");
        }

        // Start the event broker
        if (!broker->start())
        {
            LOG_ERR(logger, "Error starting the EventBroker");
            status.setError(&PayloadStatus::eventBroker);
        }

        // Starting the sensors sampling
        if (!sensors->start())
        {
            LOG_ERR(logger, "Error starting sensors sampling");
            status.setError(&PayloadStatus::radio);
        }

        if (!radio->start())
        {
            LOG_ERR(logger, "Error starting radio communication");
            status.setError(&PayloadStatus::radio);
        }

        // At the end of initialize i log the status
        SDlogger->log(status);

        // Depending on the initialization status i activate or not the FSMs
        if (status.payloadTest != OK)
        {
            LOG_ERR(logger, "Initialization failed");
            LOG_ERR(logger, "Sensors: {:d}, Radio: {:d}", status.sensors,
                    status.radio);
            // TODO add event to inhibit the state machines
        }
        else
        {
            LOG_INFO(logger, "Initialization OK");
            // TODO add the event to start the state machines
        }
    }

private:
    // Printlogger for debug and errors
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Payload");

    // SD logger accessible to everyone
    Boardcore::Logger* SDlogger;

    // Struct that resumes if at macro level there were some problems
    PayloadStatus status{};

    // SPI busses
    Boardcore::SPIBusInterface* spiInterface1;
    Boardcore::SPIBusInterface* spiInterface2;

    /**
     * @brief private constructor used by Singleton getInstance
     */
    Payload()
    {
        // Extract all the singletons
        SDlogger = &Boardcore::Logger::getInstance();
        broker   = &Boardcore::EventBroker::getInstance();

        // Instantiate all the task schedulers
        sensorsScheduler   = new Boardcore::TaskScheduler();
        algorithmScheduler = new Boardcore::TaskScheduler();
        radioScheduler     = new Boardcore::TaskScheduler();
        generalScheduler   = new Boardcore::TaskScheduler();

        // Add the task logging to the general purpose scheduler
        addSchedulerStatsTask();

        // Instantiate all the SPIs
        spiInterface1 = new Boardcore::SPIBus(SPI1);
        spiInterface2 = new Boardcore::SPIBus(SPI2);

        // Instantiate all the macro obsw objects
        sensors = new Sensors(*spiInterface1, sensorsScheduler);
        radio   = new Radio(*spiInterface2, radioScheduler);
    }

    void addSchedulerStatsTask()
    {
        // add lambda to log scheduler tasks statistics
        generalScheduler->addTask(
            [&]()
            {
                std::vector<Boardcore::TaskStatsResult> scheduler_stats =
                    generalScheduler->getTaskStats();

                for (Boardcore::TaskStatsResult stat : scheduler_stats)
                {
                    SDlogger->log(stat);
                }

                Boardcore::StackLogger::getInstance().updateStack(
                    THID_TASK_SCHEDULER);
            },
            1000,  // 1 hz
            TASK_SCHEDULER_STATS_ID, Boardcore::TaskScheduler::Policy::SKIP,
            miosix::getTick());
    }
};

}  // namespace Payload