/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/Ports/Ethernet.h>
#include <Groundstation/Automated/Radio/Radio.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Base/BoardStatus.h>
#include <Groundstation/Base/Buses.h>
#include <Groundstation/Base/Hub.h>
#include <Groundstation/Base/Ports/Ethernet.h>
#include <Groundstation/Base/Radio/Radio.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/DipReader.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ButtonHandler/ButtonHandler.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace miosix;
using namespace Groundstation;

void idleLoop()
{
    while (1)
    {
        Thread::wait();
    }
}

/**
 * @brief Blinking RED led at 5Hz
 */
void errorLoop()
{
    while (1)
    {
        led3On();  //< Turn on RED led (CU)
        Thread::sleep(100);
        led3Off();
        Thread::sleep(100);
    }
}

/**
 * @brief Lyra GS entrypoint.
 * This entrypoint performs the following operations:
 * - Reads the dip switch
 *
 * In case ARP (switch A on) select ARP initialization
 * - Initializes software modules
 *   -> Green LED is turned on when done
 * - Waits for the rocket to be powered on and acquire a GPS fix
 *   -> Yellow LED is turned on when done
 * - Waits for the antenna to acquire a GPS fix
 *   -> Orange LED is turned on when done
 * - Initializes the follower
 * - Starts the follower task
 *
 * Otherwise initialize the GroundstationBase
 * - Initializes software modules
 *   -> Green LED is turned on when main radio on
 *   -> Yellow LED is turned on when backup radio on
 *   -> Orange LED is turned on when ethernet radio on
 *
 * - When done the red LED is fixed. Blinks if error in modules init.
 */
int main()
{
    ledOff();

    // Read dip switch configuration
    DipStatus dipRead = DipReader::readDip();

    // TODO: Pass to DependencyManager when rebased
    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("lyra_gs");

    // ARP entry
    if (dipRead.isARP)
    {
        TaskScheduler *scheduler_low        = new TaskScheduler(0);
        TaskScheduler *scheduler_high       = new TaskScheduler();
        Antennas::Leds *leds                = new Antennas::Leds(scheduler_low);
        Antennas::Hub *hub                  = new Antennas::Hub();
        Antennas::Buses *buses              = new Antennas::Buses();
        Serial *serial                      = new Serial();
        Antennas::RadioMain *radio_main     = new Antennas::RadioMain();
        Antennas::BoardStatus *board_status = new Antennas::BoardStatus();
        Antennas::Actuators *actuators      = new Antennas::Actuators();
        Antennas::Sensors *sensors          = new Antennas::Sensors();
        Antennas::SMA *sma                  = new Antennas::SMA(scheduler_high);
        Antennas::Ethernet *ethernet        = new Antennas::Ethernet();

        bool ok = true;

        // Inserting Modules
        ok &= modules.insert(sma);
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(radio_main);
        ok &= modules.insert(board_status);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);
        ok &= modules.insert(ethernet);
        ok &= modules.insert(leds);

        // If insertion failed, stop right here
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to insert all modules!\n");
            errorLoop();
        }

        // Start the modules

        ok &= scheduler_low->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start scheduler_low!\n");
        }

        ok &= scheduler_high->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start scheduler_high!\n");
        }

        ok &= serial->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start serial!\n");
        }

        ok &= radio_main->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start radio_main!\n");
        }

        ok &= ethernet->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start ethernet!\n");
        }

        ok &= board_status->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start board_status!\n");
        }

        ok &= leds->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start leds!\n");
        }

        ok &= sensors->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start sensors!\n");
        }

        ok &= sma->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start sma!\n");
        }

        // Start ARP by posting INIT event
        led3On();  //< RED led (CU)
        led1On();  //< GREEN led (CU)
        LOG_INFO(logger, "Starting ARP");
        EventBroker::getInstance().post(Common::Events::ARP_INIT_OK,
                                        Common::Topics::TOPIC_ARP);
        idleLoop();
        return 0;
    }

    // Groundstation entry
    else
    {
        GroundstationBase::Hub *hub     = new GroundstationBase::Hub();
        GroundstationBase::Buses *buses = new GroundstationBase::Buses();
        Serial *serial                  = new Serial();
        GroundstationBase::Ethernet *ethernet =
            new GroundstationBase::Ethernet();
        GroundstationBase::RadioMain *radio_main =
            new GroundstationBase::RadioMain();
        GroundstationBase::RadioPayload *radio_payload =
            new GroundstationBase::RadioPayload();
        GroundstationBase::BoardStatus *board_status =
            new GroundstationBase::BoardStatus();

        bool ok = true;

        // Inserting modules

        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(buses);
        ok &= modules.insert(serial);
        ok &= modules.insert(ethernet);
        ok &= modules.insert(radio_main);
        ok &= modules.insert(radio_payload);
        ok &= modules.insert(board_status);

        // If insertion failed, stop right here
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to insert all modules!\n");
            errorLoop();
        }

        LOG_DEBUG(logger, "All modules inserted successfully!\n");

        // Ok now start them

        ok &= serial->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start serial!\n");
        }

        ok &= ethernet->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start ethernet!\n");
        }

        ok &= radio_main->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start main radio!\n");
        }

        ok &= radio_payload->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start payload radio!\n");
        }

        ok &= board_status->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start board status!\n");
        }

        LOG_DEBUG(logger, "All modules started successfully!\n");

        if (board_status->isMainRadioPresent())
        {
            LOG_ERR(logger, "Main radio detected!\n");
            led1On();  //< GREEN led on (CU)
        }

        if (board_status->isPayloadRadioPresent())
        {
            LOG_ERR(logger, "Payload radio detected!\n");
            led2On();  //< YELLOW led on (CU)
        }

        if (board_status->isEthernetPresent())
        {
            LOG_ERR(logger, "Ethernet detected!\n");
            led4On();  //< ORANGE led on (CU)
        }

        LOG_DEBUG(logger, "All boards detected!\n");

        if (!ok)
        {
            errorLoop();
        }

        led3On();  //< fix RED led (CU)
        idleLoop();
        return 0;
    }
}