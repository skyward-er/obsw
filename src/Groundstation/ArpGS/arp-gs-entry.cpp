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
#include <Groundstation/Automated/ArpStatus.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/PinHandler/PinHandler.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/GsEntryCommon.h>
#include <Groundstation/GS/Base/Hub.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <interfaces-impl/bsp_impl.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <thread>

using namespace Boardcore;
using namespace miosix;
using namespace Groundstation;
using namespace Antennas;

/**
 * @brief ARP GS entrypoint.
 *
 * Builds on the same common base as gs-entry.cpp (see
 * GsEntryCommon.h), adding the ARP-specific modules.
 *
 * Initializes the shared + ARP-specific software modules
 *   - Green LED is turned on when main radio is detected
 *   - Yellow LED is turned on when payload radio is detected
 *   - Orange LED is turned on when ethernet is detected
 * Waits for the rocket to be powered on and acquire a GPS fix
 * Waits for the antenna to acquire a GPS fix
 * Initializes the follower and starts the follower task
 */
int main()
{
    ledOff();

    DipStatus dipRead = readDipStatus();
    dipRead.print(std::cout);

    DependencyManager manager;
    PrintLogger logger = Logging::getLogger("arp_gs");

    // Safety check: this binary should always be ARP. If the dip switch says ARP
    // mode is OFF, the wrong firmware is flashed on this board, so it falls back and 
    // flashes the normal gs instead
    bool runAsArp = checkDipMatchesBinary(dipRead, /*expectedArp=*/true, logger);
    if (!runAsArp)
    {
        std::cout << "[warning] DipSwitch and flashed firmware do not match: running GS-only, ARP modules will NOT be started." << std::endl;
    }

    CommonModules mods{};
    bool ok = insertCommonModules(manager, dipRead, runAsArp, mods);

    // ARP-only modules: stay nullptr when running GS-only
    Actuators* actuators             = nullptr;
    Sensors* sensors                 = nullptr;
    Antennas::SMA* sma               = nullptr;
    Antennas::PinHandler* pinHandler = nullptr;
    Antennas::Leds* leds             = nullptr;
    Antennas::ArpStatus* arpStatus   = nullptr;
    HubBase* hub                     = nullptr;
    if (runAsArp){
        LOG_DEBUG(logger, "[debug] Inserting ARP Ground Station modules\n");
        actuators  = new Antennas::Actuators();
        sensors    = new Antennas::Sensors();
        sma        = new Antennas::SMA();
        pinHandler = new Antennas::PinHandler();
        leds       = new Antennas::Leds();
        arpStatus  = new Antennas::ArpStatus();
        hub        = new Antennas::Hub();

        ok &= manager.insert(sma);
        ok &= manager.insert(actuators);
        ok &= manager.insert(sensors);
        ok &= manager.insert(leds);
        ok &= manager.insert(pinHandler);
        ok &= manager.insert(arpStatus);
    }
    else
        hub = new GroundstationBase::Hub();

    ok &= manager.insert<HubBase>(hub);

    if (!ok)
    {
        std::cout << "[error] Failed to insert all modules!" << std::endl;
        errorLoop();
    }

    LOG_DEBUG(logger, "[debug] All modules inserted correctly!\n");

    if (!manager.inject())
    {
        std::cout << "[error] Failed to inject the dependencies!" << std::endl;
        errorLoop();
    }

    ledOn();
    Thread::sleep(2000);
    ledOff();

    bool initFatal = false;
    bool ok2       = startCommonModules(mods, runAsArp, logger, initFatal);

        if (runAsArp){
        LOG_INFO(logger, "leds starting...\n");
        if (!leds->start())
        {  
            std::cout << "[error] Failed to start leds!" << std::endl;
            ok2 = false;
        }

        LOG_INFO(logger, "sensors starting...\n");
        if (!sensors->start())
        {
            std::cout << "[error] Failed to start sensors!" << std::endl;
            ok2 = false;
        }

        LOG_INFO(logger, "DEBUG: sma starting...\n");
        if (!sma->start())
        {
            std::cout << "[error] Failed to start sma!" << std::endl;
            ok2 = false;
        }

        LOG_INFO(logger, "DEBUG: actuators starting...\n");
        actuators->start();
        LOG_INFO(logger, "[info] Actuators started!\n");

        LOG_INFO(logger, "pin handler starting...\n");
        if (!pinHandler->start())
        {
            std::cout << "[error] Failed to start PinHandler!" << std::endl;
            ok2 = false;
        }

        LOG_INFO(logger, "arpStatus starting... \n");
        if (!arpStatus->start())
        {
            std::cout << "[ERROR] Failed to start Arp Status" << std::endl;
            ok2 = false;
        }

        LOG_INFO(logger, "All modules started successfully!\n");
    }

    reportCommonStatus(mods.boardStatus, logger);

    if (runAsArp)
    {
        // If init fatal (radio_main failed), blink red endlessly -- nothing
        // can work without it.
        if (initFatal)
        {
            std::cout << "ARP's modules initialization has failed. Init fatal "
                         "error. Cannot proceed, a restart and fix of the "
                         "boards/module is required."
                      << std::endl;
            sma->setFatal();
            // Still go to INIT_ERROR to still allow initialization
            EventBroker::getInstance().post(Common::ARP_INIT_ERROR,
                                            Common::TOPIC_ARP);
        }
        else if (!ok2)
        {
            std::cout << "ARP's modules initialization has failed. Init "
                         "error. It is still possible to proceed with "
                         "MAV_ARP_CMD_FORCE_INIT."
                      << std::endl;
            EventBroker::getInstance().post(Common::ARP_INIT_ERROR,
                                            Common::TOPIC_ARP);
        }
        else
        {
            LOG_INFO(logger, "Starting ARP");
            EventBroker::getInstance().post(Common::ARP_INIT_OK,
                                            Common::TOPIC_ARP);
        }
    }
    else if (!ok2)
    {
        // GS-only fallback: same rule as gs-entry
        errorLoop();
    }

    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}
