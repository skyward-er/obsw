/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Riccardo Musso, Emilio Corigliano, Niccolò Betto
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

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/BoardStatus.h>
#include <Groundstation/Automated/Buses.h>
#include <Groundstation/Automated/Follower/Follower.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Ports/Ethernet.h>
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <diagnostic/CpuMeter/CpuMeter.h>
#include <diagnostic/PrintLogger.h>
#include <diagnostic/StackLogger.h>
#include <drivers/interrupt/external_interrupts.h>
#include <drivers/timer/TimestampTimer.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#include <iostream>
#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "actuators/stepper/StepperPWM.h"

#define START_MODULE(name, lambda)                                  \
    do                                                              \
    {                                                               \
        std::function<bool()> _fun = lambda;                        \
        if (!_fun())                                                \
        {                                                           \
            LOG_ERR(logger, "Failed to start module " name);        \
            errorLoop();                                            \
        }                                                           \
        else                                                        \
        {                                                           \
            LOG_DEBUG(logger, "Successfully started module " name); \
        }                                                           \
    } while (0)

using namespace Groundstation;
using namespace Antennas;
using namespace Boardcore;
using namespace miosix;

GpioPin button = GpioPin(GPIOG_BASE, 10);  ///< Emergency stop button

/**
 * @brief Infinite error loop, used to blink an LED when an error occurs.
 */
void errorLoop()
{
    while (true)
    {
        userLed4::high();
        Thread::sleep(100);
        userLed4::low();
        Thread::sleep(100);
    }
}

/**
 * @brief Acquires the current rocket GPS position from the main radio.
 * @details As a side effect, this function also waits for the rocket to be
 * powered on.
 */
GPSData acquireRocketGpsState(Hub *hub)
{
    GPSData rocketGpsState;
    do
    {
        rocketGpsState = hub->getLastRocketGpsState();
        led2On();
        Thread::sleep(50);
        led2Off();
        Thread::sleep(50);
    } while (rocketGpsState.fix == 0);
    return rocketGpsState;
}

/**
 * @brief Acquires the current antenna GPS position from the VN300 IMU.
 */
GPSData acquireAntennaGpsState(Sensors *sensors)
{
    VN300Data vn300Data;
    do
    {
        vn300Data = sensors->getVN300LastSample();
        led3On();
        Thread::sleep(50);
        led3Off();
        Thread::sleep(50);
    } while (vn300Data.fix_gps == 0);

    GPSData antennaPosition;
    antennaPosition.gpsTimestamp  = vn300Data.insTimestamp;
    antennaPosition.latitude      = vn300Data.latitude;
    antennaPosition.longitude     = vn300Data.longitude;
    antennaPosition.height        = vn300Data.altitude;
    antennaPosition.velocityNorth = vn300Data.nedVelX;
    antennaPosition.velocityEast  = vn300Data.nedVelY;
    antennaPosition.velocityDown  = vn300Data.nedVelZ;
    antennaPosition.satellites    = vn300Data.fix_gps;
    antennaPosition.fix           = (vn300Data.fix_gps > 0);
    return antennaPosition;
}

/**
 * @brief Automated Antennas (SkyLink) entrypoint.
 * The entrypoint performs the following operations:
 * - Initializes software modules
 *   -> Green LED is turned on when done
 * - Waits for the rocket to be powered on and acquire a GPS fix
 *   -> Yellow LED is turned on when done
 * - Waits for the antenna to acquire a GPS fix
 *   -> Orange LED is turned on when done
 * - Initializes the follower
 * - Starts the follower task
 */
int main()
{
    ledOff();
    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("automated_antennas");
    bool ok                = true;

    button.mode(Mode::INPUT);
    // ButtonHandler
    ButtonHandler::getInstance().registerButtonCallback(
        button,
        [&](ButtonEvent bEvent)
        {
            if (bEvent == ButtonEvent::PRESSED)
            {
                ModuleManager::getInstance()
                    .get<Actuators>()
                    ->IRQemergencyStop();
            }
        });

    TaskScheduler *scheduler  = new TaskScheduler();
    Hub *hub                  = new Hub();
    Buses *buses              = new Buses();
    Serial *serial            = new Serial();
    RadioMain *radio_main     = new RadioMain();
    BoardStatus *board_status = new BoardStatus();
    Actuators *actuators      = new Actuators();
    Sensors *sensors          = new Sensors();
    Follower *follower        = new Follower();
    Ethernet *ethernet        = new Ethernet();

    // Inserting Modules
    {
        ok &= modules.insert(follower);
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(radio_main);
        ok &= modules.insert(board_status);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);
        ok &= modules.insert(ethernet);

        // If insertion failed, stop right here
        if (!ok)
        {
            LOG_ERR(logger, "Failed to insert all modules!\n");
            errorLoop();
        }
        else
        {
            LOG_DEBUG(logger, "All modules inserted successfully!\n");
        }
    }

    // Starting Modules
    {
#ifndef NO_SD_LOGGING
        START_MODULE("Logger", [&] { return Logger::getInstance().start(); });
#endif
        START_MODULE("Scheduler", [&] { return scheduler->start(); });
        START_MODULE("Serial", [&] { return serial->start(); });
        START_MODULE("Main Radio", [&] { return radio_main->start(); });
        START_MODULE("Ethernet", [&] { return ethernet->start(); });
        START_MODULE("Board Status", [&] { return board_status->start(); });
        START_MODULE("Sensors", [&] { return sensors->start(); });
        actuators->start();
    }

    // Setup success LED
    led1On();
    LOG_INFO(logger, "Modules setup successful");

    if (board_status->isMainRadioPresent())
    {
        LOG_DEBUG(logger, "Main radio is present\n");
    }

    LOG_INFO(logger, "Starting Skylink");

    // Wait for antenna GPS fix
    GPSData antennaGpsState = acquireAntennaGpsState(sensors);

    LOG_INFO(logger, "Antenna GPS position acquired !coord [{}, {}] [deg]",
             antennaGpsState.latitude, antennaGpsState.longitude);
    follower->setAntennaCoordinates(antennaGpsState);
    // Antenna GPS fix LED
    led2On();

    // Wait for rocket presence and GPS fix
    GPSData rocketGpsState = acquireRocketGpsState(hub);
    LOG_INFO(logger, "Rocket GPS position acquired [{}, {}] [deg]",
             rocketGpsState.latitude, rocketGpsState.longitude);
    follower->setInitialRocketCoordinates(rocketGpsState);
    // Rocket presence and GPS fix LED
    led3On();

    // Init the follower after GPS position was acquired
    if (!follower->init())
    {
        LOG_ERR(logger, "Failed to start follower!\n");
        errorLoop();
    }
    else
    {
        auto distance = follower->getInitialAntennaRocketDistance();
        LOG_INFO(logger, "Initial antenna - rocket distance: [{}, {}] [m]\n",
                 distance[0], distance[1]);
    }

    follower->begin();
    scheduler->addTask(std::bind(&Follower::update, follower), 200);
    LOG_INFO(logger, "Follower task started successfully");

    while (true)
    {
        Thread::wait();
    }
    return 0;
}
