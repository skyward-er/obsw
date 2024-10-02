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
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/PinHandler/PinHandler.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/LyraGS/Base/Hub.h>
#include <Groundstation/LyraGS/BoardStatus.h>
#include <Groundstation/LyraGS/Buses.h>
#include <Groundstation/LyraGS/Ports/Ethernet.h>
#include <Groundstation/LyraGS/Radio/Radio.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/DipSwitch/DipSwitch.h>
#include <events/EventBroker.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace miosix;
using namespace Groundstation;
using namespace LyraGS;

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
    uint32_t microSecClk = 100;
    GpioPin sh           = dipSwitch::sh::getPin();
    GpioPin clk          = dipSwitch::clk::getPin();
    GpioPin qh           = dipSwitch::qh::getPin();

    DipSwitch dip(sh, clk, qh, microSecClk);
    DipStatus dipRead = dip.read();

    // TODO: Pass to DependencyManager when rebased!
    ModuleManager &modules = ModuleManager::getInstance();
    PrintLogger logger     = Logging::getLogger("lyra_gs");

    // TODO: Board scheduler for the schedulers
    TaskScheduler *scheduler_low  = new TaskScheduler(0);
    TaskScheduler *scheduler_high = new TaskScheduler();
    Buses *buses                  = new Buses();
    Serial *serial                = new Serial();
    RadioMain *radio_main         = new LyraGS::RadioMain();
    BoardStatus *board_status     = new BoardStatus(dipRead.isARP);
    LyraGS::Ethernet *ethernet  = new LyraGS::Ethernet(false, dipRead.ipConfig);
    RadioPayload *radio_payload = new RadioPayload();
    Antennas::Actuators *actuators = nullptr;

    bool ok = true;

    ok &= modules.insert(buses);
    ok &= modules.insert(serial);
    ok &= modules.insert(radio_main);
    ok &= modules.insert(ethernet);
    ok &= modules.insert(radio_payload);
    ok &= modules.insert(board_status);

    // Inserting Modules

    // ARP modules insertion
    if (dipRead.isARP)
    {
        LOG_DEBUG(logger, "[debug] Starting as ARP Ground Station\n");
        Antennas::Leds *leds             = new Antennas::Leds(scheduler_low);
        HubBase *hub                     = new Antennas::Hub();
        actuators                        = new Antennas::Actuators();
        Antennas::Sensors *sensors       = new Antennas::Sensors();
        Antennas::SMA *sma               = new Antennas::SMA(scheduler_high);
        Antennas::PinHandler *pinHandler = new Antennas::PinHandler();

        ok &= modules.insert(sma);
        ok &= modules.insert<HubBase>(hub);
        ok &= modules.insert(actuators);
        ok &= modules.insert(sensors);
        ok &= modules.insert(leds);
        ok &= modules.insert(pinHandler);
    }
    // Ground station module insertion
    else
    {
        LOG_DEBUG(logger, "[debug] Starting as GS base Ground Station\n");
        HubBase *hub = new GroundstationBase::Hub();
        ok &= modules.insert<HubBase>(hub);
    }

    // If insertion failed, stop right here

    if (!ok)
    {
        LOG_ERR(logger, "[error] Failed to insert all modules!\n");
        errorLoop();
    }

    LOG_DEBUG(logger, "[debug] All modules inserted correctly!\n");

    // Start the modules

    // ARP start errors
    bool init_fatal = false;

#ifndef NO_SD_LOGGING
    if (!Logger::getInstance().start())
    {
        LOG_ERR(logger, "ERROR: Failed to start Logger\n");
        ok = false;
    }
#endif

    if (!scheduler_low->start())
    {
        LOG_ERR(logger, "[error] Failed to start scheduler_low!\n");
        ok = false;
    }

    if (!scheduler_high->start())
    {
        LOG_ERR(logger, "[error] Failed to start scheduler_high!\n");
        ok         = false;
        init_fatal = true;
    }

    if (!serial->start())
    {
        LOG_ERR(logger, "[error] Failed to start serial!\n");
        ok = false;
    }

    if (!radio_main->start())
    {
        LOG_ERR(logger, "[error] Failed to start radio_main!\n");
        ok         = false;
        init_fatal = true;
    }

    if (!radio_payload->start())
    {
        LOG_ERR(logger, "[error] Failed to start payload radio!\n");
    }

    if (!ethernet->start())
    {
        LOG_ERR(logger, "[error] Failed to start ethernet!\n");
        ok = false;
    }

    if (!board_status->start())
    {
        LOG_ERR(logger, "[error] Failed to start board_status!\n");
        ok = false;
    }

    // Starting ARP specific modules

    if (dipRead.isARP)
    {
        ok &= ModuleManager::getInstance().get<Antennas::Leds>()->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start leds!\n");
        }

        ok &= ModuleManager::getInstance().get<Antennas::Sensors>()->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start sensors!\n");
        }

        ok &= ModuleManager::getInstance().get<Antennas::SMA>()->start();
        if (!ok)
        {
            LOG_ERR(logger, "[error] Failed to start sma!\n");
        }

        if (actuators)
        {
            LOG_INFO(logger, "[info] Actuators started!\n");
        }

        ok &= ModuleManager::getInstance().get<Antennas::PinHandler>()->start();

        if (!modules.get<Antennas::PinHandler>()->start())
            if (!ok)
            {
                LOG_ERR(logger, "[error] Failed to start PinHandler!\n");
            }
    }

    if (!ok)
    {
        LOG_ERR(logger, "Could not start all modules successfully!\n");
        errorLoop();
    }

    LOG_DEBUG(logger, "All modules started successfully!\n");

    // Check presence of radio and ethernet

    if (board_status->isMainRadioPresent())
    {
        LOG_INFO(logger, "Main radio detected!\n");
        led1On();  //< GREEN led on (CU)
    }
    else
        LOG_ERR(logger, "Main NOT detected\n");

    if (board_status->isPayloadRadioPresent())
    {
        LOG_INFO(logger, "Payload radio detected!\n");
        led2On();  //< YELLOW led on (CU)
    }
    else
        LOG_ERR(logger, "Payload NOT detected\n");

    if (board_status->isEthernetPresent())
    {
        LOG_INFO(logger, "Ethernet detected!\n");
        led4On();  //< ORANGE led on (CU)
    }
    else
        LOG_ERR(logger, "Ethernet NOT detected\n");

    if (!ok)
    {
        errorLoop();
    }

    if (dipRead.isARP)
    {
        // If init fatal and sma not started, blink red endlessly
        if (init_fatal)
        {
            ModuleManager::getInstance().get<Antennas::Leds>()->endlessBlink(
                Antennas::LedColor::RED, LED_BLINK_FAST_PERIOD_MS);
        }  // If another module is in error
        else if (!ok)
        {
            EventBroker::getInstance().post(Common::ARP_INIT_ERROR,
                                            Common::TOPIC_ARP);
        }  // If all modules are ok
        else
        {
            LOG_INFO(logger, "Starting ARP");
            EventBroker::getInstance().post(Common::ARP_INIT_OK,
                                            Common::TOPIC_ARP);
        }
    }
    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}
