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

#include <Groundstation/ArpGS/Base/Hub.h>
#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Leds/Leds.h>
#include <Groundstation/Automated/PinHandler/PinHandler.h>
#include <Groundstation/Automated/SMA/SMA.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <Groundstation/Common/GsEntryCommon.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
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
 * Builds on the same common base as radio-gs-entry.cpp (see
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

    CommonModules mods{};
    bool ok = insertCommonModules(manager, dipRead, mods);

    LOG_DEBUG(logger, "[debug] Inserting ARP Ground Station modules\n");
    Actuators* actuators             = new Antennas::Actuators();
    Sensors* sensors                 = new Antennas::Sensors();
    Antennas::SMA* sma               = new Antennas::SMA();
    Antennas::PinHandler* pinHandler = new Antennas::PinHandler();
    Antennas::Leds* leds             = new Antennas::Leds();
    ok &= manager.insert(sma);
    ok &= manager.insert(actuators);
    ok &= manager.insert(sensors);
    ok &= manager.insert(leds);
    ok &= manager.insert(pinHandler);

    HubBase* hub = new Antennas::Hub();
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
    bool ok2       = startCommonModules(mods, dipRead, logger, initFatal);

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

    LOG_INFO(logger, "All modules started successfully!\n");

    reportCommonStatus(mods.boardStatus, logger);

    // Safety check: this binary is always ARP. If the dip switch says ARP
    // mode is OFF, the wrong firmware is flashed on this board.
    if (!checkDipMatchesBinary(dipRead, /*expectedArp=*/true, logger))
    {
        std::cout << "ARP initialization warning: dip switch / firmware "
                     "mismatch (see above). This board will run as ARP "
                     "regardless of the dip switch setting."
                  << std::endl;
    }

    // If init fatal (radio_main failed), blink red endlessly -- nothing can
    // work without it.
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
        std::cout << "ARP's modules initialization has failed. Init error. It "
                     "is still possible to proceed with MAV_ARP_CMD_FORCE_INIT."
                  << std::endl;
        EventBroker::getInstance().post(Common::ARP_INIT_ERROR,
                                        Common::TOPIC_ARP);
    }
    else
    {
        LOG_INFO(logger, "Starting ARP");
        EventBroker::getInstance().post(Common::ARP_INIT_OK, Common::TOPIC_ARP);
    }

    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}
