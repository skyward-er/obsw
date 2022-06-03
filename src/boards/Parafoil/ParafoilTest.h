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

#include <Parafoil/FlightModeManager/FMMController.h>
#include <Parafoil/Main/Radio.h>
#include <Parafoil/Main/Sensors.h>
#include <Parafoil/ParafoilTestStatus.h>
#include <Parafoil/Wing/WingController.h>
#include <drivers/spi/SPIDriver.h>
#include <events/EventBroker.h>
#include <miosix.h>

/**
 * This class is the main singleton that keeps all the project objects.
 * It has all the instances and initializes all of them.
 */
namespace Parafoil
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

class ParafoilTest : public Boardcore::Singleton<ParafoilTest>
{
    friend class Boardcore::Singleton<ParafoilTest>;

public:
    /**
     * @brief Event broker
     */
    Boardcore::EventBroker* broker;

    /**
     * @brief Sensors collection
     */
    Sensors* sensors;

    /**
     * @brief Wing algorithm controller
     */
    WingController* wingController;

    /**
     * @brief Radio that manages the interaction between
     * the xbee module and mavlink
     */
    Radio* radio;

    /**
     * @brief Main FSM
     */
    FMMController* FMM;

    /**
     * @brief Task scheduler
     */
    Boardcore::TaskScheduler* scheduler;

    /**
     * @brief SDlogger singleton for SD
     */
    Boardcore::Logger* SDlogger;

    /**
     * @brief Start method
     */
    void start()
    {
        // Start the task scheduler
        if (!scheduler->start())
        {
            LOG_ERR(log, "Error starting the main task scheduler");
        }

        // Start the broker
        if (!broker->start())
        {
            LOG_ERR(log, "Error starting EventBroker");
            status.setError(&ParafoilTestStatus::eventBroker);
        }

        // Start the sensors sampling
        if (!sensors->start())
        {
            LOG_ERR(log, "Error starting sensors");
            status.setError(&ParafoilTestStatus::sensors);
        }

        // Start the main FSM
        /*if(!FMM -> start())
        {
            LOG_ERR(log, "Error starting the main FSM");
            status.setError(&ParafoilTestStatus::FMM);
        }*/

        // Start the radio
        if (!radio->start())
        {
            LOG_ERR(log, "Error starting the radio");
            status.setError(&ParafoilTestStatus::radio);
        }

        // wingController->start();

        // After all the initializations i log the status
        SDlogger->log(status);

        // Status LED declaration
        miosix::GpioPin statusLed(GPIOG_BASE, 13);
        statusLed.mode(miosix::Mode::OUTPUT);
        statusLed.low();

        // If all is ok i can send the signal to the FSMs
        if (status.parafoil_test != OK)
        {
            LOG_ERR(log, "Initialization failed");
            // TODO add event to inibit the state machines
        }
        else
        {
            statusLed.high();
            LOG_INFO(log, "Initialization ok");
            // TODO add event to start the state machines
        }
    }

    /**
     * @brief Method to start the SDlogger singleton
     */
    void startSDlogger()
    {
        try
        {
            SDlogger->start();
            // Log in serial
            LOG_INFO(log, "SDlogger started");
        }
        catch (const std::runtime_error& error)
        {
            LOG_ERR(log, "SD SDlogger init error");
            status.setError(&ParafoilTestStatus::logger);
        }
        // Log the status
        SDlogger->log(SDlogger->getLoggerStats());
    }

private:
    /**
     * @brief SDlogger in debug mode
     */
    Boardcore::PrintLogger log = Boardcore::Logging::getLogger("ParafoilTest");

    /**
     * @brief Status memory
     */
    ParafoilTestStatus status{};

    /**
     * @brief Constructor
     */
    ParafoilTest()
    {
        // Take the singleton instance of SD logger
        SDlogger = &Boardcore::Logger::getInstance();

        // Start the logging
        startSDlogger();

        // Store the broker
        broker = &sEventBroker;

        // Create the task scheduler
        scheduler = new Boardcore::TaskScheduler();
        addSchedulerStatsTask();

        // Create the sensors
        Boardcore::SPIBusInterface* spiInterface1 = new Boardcore::SPIBus(SPI1);
        sensors = new Sensors(*spiInterface1, scheduler);

        // Create the wing controller
        wingController = new WingController(scheduler);
        wingController->addAlgorithm("/sd/servoTerni1.csv");
        wingController->addAlgorithm("/sd/servoTerni2.csv");
        wingController->addAlgorithm("/sd/servoCorta.csv");
        wingController->addAlgorithm("/sd/servoLunga.csv");
        wingController->selectAlgorithm(0);
        // Create the main FSM
        // FMM = new FMMController();

        // Create a new radio
        Boardcore::SPIBusInterface* spiInterface4 = new Boardcore::SPIBus(SPI4);
        radio = new Radio(*spiInterface4, scheduler);
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

}  // namespace Parafoil
