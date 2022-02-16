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

#include <miosix.h>
#include <ParafoilTestStatus.h>
#include <Wing/WingController.h>
#include <Main/Sensors.h>
#include <Main/Radio.h>
#include <events/EventBroker.h>
#include <drivers/spi/SPIDriver.h>
#include <FlightModeManager/FMMController.h>

using namespace Boardcore;

/**
 * This class is the main singleton that keeps all the project objects.
 * It has all the instances and initializes all of them.
 */
namespace ParafoilTestDev
{
    class ParafoilTest : public Singleton<ParafoilTest>
    {
        friend class Singleton<ParafoilTest>;

    public:

        /**
         * @brief Event broker
         */
        EventBroker* broker;

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
        TaskScheduler* scheduler;

        /**
         * @brief Start method
         */
        void start()
        {
            //Start the broker
            if(!broker -> start())
            {
                LOG_ERR(log, "Error starting EventBroker");
                status.setError(&ParafoilTestStatus::eventBroker);
            }

            //Start the sensors sampling
            /*if(!sensors -> start())
            {
                LOG_ERR(log, "Error starting sensors");
                status.setError(&ParafoilTestStatus::sensors);
            }*/

            //Start the main FSM
            if(!FMM -> start())
            {
                LOG_ERR(log, "Error starting the main FSM");
                status.setError(&ParafoilTestStatus::FMM);
            }

            //Start the radio
            if(!radio -> start())
            {
                LOG_ERR(log, "Error starting the radio");
                status.setError(&ParafoilTestStatus::radio);
            }

            //After all the initializations i log the status
            SDlogger -> log(status);

            //If all is ok i can send the signal to the FSMs
            if(status.parafoil_test != OK)
            {
                LOG_ERR(log, "Initialization failed");
                //TODO add event to inibit the state machines
            }
            else
            {
                LOG_INFO(log, "Initialization ok");
                //TODO add event to start the state machines
            }
        }

    private:
        
        /**
         * @brief SDlogger in debug mode
         */
        PrintLogger log = Logging::getLogger("ParafoilTest");

        /**
         * @brief SDlogger singleton for SD
         */
        Logger* SDlogger;

        /**
         * @brief Status memory
         */
        ParafoilTestStatus status{};

        /**
         * @brief Constructor
         */
        ParafoilTest()
        {
            //Take the singleton instance of SD logger
            SDlogger = &Logger::getInstance();

            //Start the logging
            startSDlogger();

            //Store the broker
            broker = &sEventBroker;

            //Create the task scheduler
            scheduler = new TaskScheduler();

            //Create the sensors
            SPIBusInterface *spiInterface1 = new SPIBus(SPI5);
            //sensors = new Sensors(*spiInterface1, scheduler);

            //Create the wing controller
            //wingController = new WingController(scheduler);

            //Create the main FSM
            FMM = new FMMController();

            //Create a new radio
            radio = new Radio(*spiInterface1, scheduler);
        }

        /**
         * @brief Method to start the SDlogger singleton
         */
        void startSDlogger()
        {
            try
            {
                SDlogger -> start();
                //Log in serial
                LOG_INFO(log, "SDlogger started");
            }
            catch(const std::runtime_error& error)
            {
                LOG_ERR(log, "SD SDlogger init error");
                status.setError(&ParafoilTestStatus::logger);
            }
            //Log the status
            SDlogger -> log(SDlogger -> getLoggerStats());
        }
    };
}