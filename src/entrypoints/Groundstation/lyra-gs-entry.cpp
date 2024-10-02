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
#include <utils/DependencyManager/DependencyManager.h>

#include <thread>

using namespace Boardcore;
using namespace miosix;
using namespace Groundstation;
using namespace LyraGS;
using namespace Antennas;

/**
 * @brief Dip switch status for the GS board
 */
struct DipStatusLyraGS
{
    bool isARP;
    bool mainHasBackup;
    bool payloadHasBackup;
    bool mainTXenable;
    bool payloadTXenable;
    uint8_t ipConfig;
};

DipStatusLyraGS getDipStatus(uint8_t read)
{
    DipStatusLyraGS dipRead;
    dipRead.isARP            = 1 & read;
    dipRead.mainHasBackup    = 1 & (read >> 1);
    dipRead.payloadHasBackup = 1 & (read >> 2);
    dipRead.mainTXenable     = 1 & (read >> 3);
    dipRead.payloadTXenable  = 1 & (read >> 4);
    dipRead.ipConfig         = 0 | (read >> 5);
    return dipRead;
}

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
 *   -> Yellow LED is turned on when payload radio on
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
    DipStatusLyraGS dipRead = getDipStatus(dip.read());

    DependencyManager manager;
    PrintLogger logger = Logging::getLogger("lyra_gs");

    // TODO: Board scheduler for the schedulers
    TaskScheduler *scheduler_low  = new TaskScheduler(0);
    TaskScheduler *scheduler_high = new TaskScheduler();
    Buses *buses                  = new Buses();
    Serial *serial                = new Serial();
    LyraGS::RadioMain *radio_main =
        new LyraGS::RadioMain(dipRead.mainHasBackup, dipRead.mainTXenable);
    LyraGS::BoardStatus *board_status = new LyraGS::BoardStatus(dipRead.isARP);
    LyraGS::EthernetGS *ethernet =
        new LyraGS::EthernetGS(false, dipRead.ipConfig);
    LyraGS::RadioPayload *radio_payload = new LyraGS::RadioPayload(
        dipRead.payloadHasBackup, dipRead.payloadTXenable);

    HubBase *hub = nullptr;

    // ARP-related things
    Antennas::Actuators *actuators   = nullptr;
    Antennas::Leds *leds             = nullptr;
    Antennas::Sensors *sensors       = nullptr;
    Antennas::SMA *sma               = nullptr;
    Antennas::PinHandler *pinHandler = nullptr;

    bool ok = true;

    ok &= manager.insert(buses);
    ok &= manager.insert(serial);
    ok &= manager.insert<LyraGS::RadioMain>(radio_main);
    ok &= manager.insert<LyraGS::EthernetGS>(ethernet);
    ok &= manager.insert<LyraGS::RadioPayload>(radio_payload);
    ok &= manager.insert(board_status);

    // Inserting Modules

    // ARP modules insertion
    LOG_DEBUG(logger, "[debug] Inserting ARP Ground Station modules\n");
    actuators  = new Antennas::Actuators();
    sensors    = new Antennas::Sensors();
    sma        = new Antennas::SMA(scheduler_high);
    pinHandler = new Antennas::PinHandler();
    leds       = new Antennas::Leds(scheduler_low);
    ok &= manager.insert(sma);
    ok &= manager.insert(actuators);
    ok &= manager.insert(sensors);
    ok &= manager.insert(leds);
    ok &= manager.insert(pinHandler);

    if (dipRead.isARP)
    {
        hub = new Antennas::Hub();
        ok &= manager.insert<HubBase>(hub);
    }
    // Ground station module insertion
    else
    {
        LOG_DEBUG(logger, "[debug] Starting as GS base Ground Station\n");
        hub = new GroundstationBase::Hub();
        ok &= manager.insert<HubBase>(hub);
    }

    // If insertion failed, stop right here
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

    // Print out the graph of dependencies
    manager.graphviz(std::cout);

    // Start the modules

    ledOn();
    Thread::sleep(2000);
    ledOff();

    // ARP start errors
    bool init_fatal = false;

#ifndef NO_SD_LOGGING
    if (!Logger::getInstance().start() && dipRead.isARP)
    {
        std::cout << "ERROR: Failed to start Logger" << std::endl;
        ok = false;
    }
#endif

    LOG_DEBUG(logger, "DEBUG: scheduler_low starting...\n");

    if (!scheduler_low->start())
    {
        std::cout << "[error] Failed to start scheduler_low!" << std::endl;
        ok = false;
    }

    LOG_DEBUG(logger, "DEBUG: scheduler_high starting...\n");

    if (!scheduler_high->start())
    {
        std::cout << "[error] Failed to start scheduler_high!" << std::endl;
        ok         = false;
        init_fatal = true;
    }

    LOG_DEBUG(logger, "DEBUG: serial starting...\n");

    if (!serial->start())
    {
        std::cout << "[error] Failed to start serial!" << std::endl;
        ok = false;
    }

    LOG_DEBUG(logger, "DEBUG: radio_main starting...\n");

    if (!radio_main->start())
    {
        std::cout << "[error] Failed to start radio_main!" << std::endl;
        ok         = false;
        init_fatal = true;
    }

    LOG_DEBUG(logger, "DEBUG: radio_payload starting...\n");

    if (!radio_payload->start())
    {
        std::cout << "[error] Failed to start payload radio!" << std::endl;
        // Payload module is needed just for GS, not for ARP
        ok &= dipRead.isARP;
    }

    LOG_DEBUG(logger, "DEBUG: ethernet starting...\n");

    if (!ethernet->start())
    {
        std::cout << "[error] Failed to start ethernet!" << std::endl;
        ok = false;
    }

    LOG_DEBUG(logger, "DEBUG: board_status starting...\n");

    if (!board_status->start())
    {
        std::cout << "[error] Failed to start board_status!" << std::endl;
        ok = false;
    }

    // Starting ARP specific modules

    if (dipRead.isARP)
    {
        LOG_DEBUG(logger, "DEBUG: leds starting...\n");

        if (leds && !(leds->start()))
        {
            std::cout << "[error] Failed to start leds!" << std::endl;
            ok = false;
        }

        LOG_DEBUG(logger, "DEBUG: sensors starting...\n");

        if (sensors && !(sensors->start()))
        {
            std::cout << "[error] Failed to start sensors!" << std::endl;
            ok = false;
        }

        LOG_DEBUG(logger, "DEBUG: sma starting...\n");

        if (sma && !(sma->start()))
        {
            std::cout << "[error] Failed to start sma!" << std::endl;
            ok = false;
        }

        LOG_DEBUG(logger, "DEBUG: actuators starting...\n");

        if (actuators)
        {
            actuators->start();
            LOG_INFO(logger, "[info] Actuators started!\n");
        }
        else
            std::cout << "[error] Failed to start actuators!" << std::endl;

        LOG_DEBUG(logger, "DEBUG: pin handler starting...\n");

        if (pinHandler && !pinHandler->start())
        {
            std::cout << "[error] Failed to start PinHandler!" << std::endl;
            ok = false;
        }
    }

    if (!dipRead.isARP && !ok)
    {
        std::cout << "GS: could not start all modules successfully!"
                  << std::endl;
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
        std::cout << "Main NOT detected" << std::endl;

    if (board_status->isPayloadRadioPresent())
    {
        LOG_INFO(logger, "Payload radio detected!\n");
        led2On();  //< YELLOW led on (CU)
    }
    else
        std::cout << "Payload NOT detected" << std::endl;

    if (board_status->isEthernetPresent())
    {
        LOG_INFO(logger, "Ethernet detected!\n");
        led4On();  //< ORANGE led on (CU)
    }
    else
        std::cout << "Ethernet NOT detected" << std::endl;

    if (!dipRead.isARP && !ok)
    {
        std::cout << "GS initialization has failed. Not all modules started "
                     "correctly!"
                  << std::endl;
        errorLoop();
    }

    if (dipRead.isARP)
    {
        // If init fatal and sma not started, blink red endlessly
        if (init_fatal)
        {
            std::cout << "ARP's modules initialization has failed. Init fatal "
                         "error. Cannot proceed, a restart and fix of the "
                         "boards/module is required."
                      << std::endl;
            if (sma)
                sma->setFatal();
            // Still go to INIT_ERROR to still allow initialization
            EventBroker::getInstance().post(Common::ARP_INIT_ERROR,
                                            Common::TOPIC_ARP);

        }  // If another module is in error
        else if (!ok)
        {
            std::cout
                << "ARP's modules initialization has failed. Init error. It "
                   "is still possible to proceed with MAV_ARP_CMD_FORCE_INIT."
                << std::endl;
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
