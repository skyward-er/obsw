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

#include <Groundstation/Arp/Automated/Actuators/Actuators.h>
#include <Groundstation/Arp/Automated/ArpStatus.h>
#include <Groundstation/Arp/Automated/BoardScheduler.h>
#include <Groundstation/Arp/Automated/Leds/Leds.h>
#include <Groundstation/Arp/Automated/PinHandler/PinHandler.h>
#include <Groundstation/Arp/Automated/SMA/SMA.h>
#include <Groundstation/Arp/Automated/Sensors/Sensors.h>
#include <Groundstation/Arp/BoardStatus.h>
#include <Groundstation/Arp/Buses.h>
#include <Groundstation/Arp/Hub.h>
#include <Groundstation/Arp/Ports/Ethernet.h>
#include <Groundstation/Arp/Ports/EthernetSniffer.h>
#include <Groundstation/Arp/Ports/Serial.h>
#include <Groundstation/Arp/Radio/Radio.h>
#include <Groundstation/Common/DipStatus.h>
#include <common/Events.h>
#include <diagnostic/PrintLogger.h>
#include <events/EventBroker.h>
#include <interfaces-impl/bsp_impl.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iostream>
#include <thread>

using namespace Boardcore;
using namespace miosix;

/**
 * @brief Entry point of the ARP groundstation
 *
 * Build/flash: ./sbs build arp-gs-entry
 *
 * Modules (Arp/):
 *  - BoardScheduler: TaskScheduler instances (SMA, sensors, others) shared by
 *                    the ARP modules
 *  - Buses:          SPI/serial peripheral buses
 *  - Serial:         serial link
 *  - RadioMain       the radio modules((also RadioPayload))
 *  - EthernetGS:     Ethernet link
 *  - EthernetSniffer: second Ethernet port that sniffs traffic
 *  - Hub:            routes packets between radios, Ethernet and serial
 *  - BoardStatus:    periodic board telemetry (radio/Ethernet state, bitrates)
 *  - Actuators:      the two stepper motors (horizontal/vertical) of arp
 *  - Sensors:        VN300 IMU/GPS, read through SensorManager
 *  - SMA:            "State Machine (for) Arp"
 *  - PinHandler:     reads the ARM and ACTIVE switches
 *  - Leds:           leds :)
 *  - ArpStatus:      periodic ARP-only telemetry (mavlink_arp_tm_t)

 * Shared code (DIP, HubBase, configs, base classes) is in Common/.
    *!* Arp/ and Gs/ are independent: never include each other  *!*
 */

namespace
{

constexpr bool RANDOM_IP         = false;
constexpr bool ETHERNET_SNIFFING = true;

/// Used once init succeeds
void idleLoop()
{
    while (1)
        Thread::wait();
}

/// Blinks the red LED at 5Hz forever
void errorLoop()
{
    while (1)
    {
        led3On();
        Thread::sleep(100);
        led3Off();
        Thread::sleep(100);
    }
}

/// Logs which radios/ethernet are actually present and turns on their LEDs.
void reportStatus(Arp::BoardStatus* boardStatus, PrintLogger& logger)
{
    if (boardStatus->isMainRadioPresent())
    {
        LOG_INFO(logger, "Main radio detected!\n");
        led1On();  //< GREEN led on (CU)
    }
    else
        std::cout << "Main NOT detected" << std::endl;

    if (boardStatus->isPayloadRadioPresent())
    {
        LOG_INFO(logger, "Payload radio detected!\n");
        led2On();  //< YELLOW led on (CU)
    }
    else
        std::cout << "Payload NOT detected" << std::endl;

    if (boardStatus->isEthernetPresent())
    {
        LOG_INFO(logger, "Ethernet detected!\n");
        led4On();  //< ORANGE led on (CU)
    }
    else
        std::cout << "Ethernet NOT detected" << std::endl;
}

}  // namespace

/**
 * @brief ARP GS entrypoint.
 *
 * For ARP station with the relative hardware attached
 * (motors, steppers, etc.). The gs-only variant lives in
 * Groundstation/Gs, the two only share Groundstation/Common.
 *
 * Initializes the software modules (radios, ethernet, serial) and the
 * ARP-specific ones (actuators, sensors, SMA, leds, pin handler, status)
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

    Groundstation::DipStatus dipRead = Groundstation::readDipStatus();
    dipRead.print(std::cout);

    PrintLogger logger = Logging::getLogger("arp_gs");

    // This binary is the ARP one: if the dip switch says ARP mode is OFF the
    // board is flashed with the wrong firmware. Do not start anything (the
    // steppers included), just signal the error.
    if (!dipRead.isARP)
    {
        std::cout << "[error] Dip switch / firmware mismatch! This board is "
                     "flashed with the ARP firmware (arp-gs-entry), but the "
                     "dip switch ARP bit is OFF. Turn ON the ARP bit (dip "
                     "switch 1) or flash the radio-only gs-entry firmware."
                  << std::endl;
        errorLoop();
    }

    DependencyManager manager;

    auto* scheduler = new Arp::BoardScheduler();
    auto* buses     = new Arp::Buses();
    auto* serial    = new Arp::Serial();
    auto* radioMain =
        new Arp::RadioMain(dipRead.mainHasBackup, dipRead.mainTXenable);
    auto* boardStatus = new Arp::BoardStatus();
    auto* ethernet =
        new Arp::EthernetGS(RANDOM_IP, dipRead.ipConfig, ETHERNET_SNIFFING);
    auto* sniffer      = new Arp::EthernetSniffer();
    auto* radioPayload = new Arp::RadioPayload(dipRead.payloadHasBackup,
                                               dipRead.payloadTXenable);

    auto* actuators             = new Arp::Actuators();
    auto* sensors               = new Arp::Sensors();
    auto* sma                   = new Arp::SMA();
    auto* pinHandler            = new Arp::PinHandler();
    auto* leds                  = new Arp::Leds();
    auto* arpStatus             = new Arp::ArpStatus();
    Groundstation::HubBase* hub = new Arp::Hub();

    bool ok = true;
    ok &= manager.insert(buses);
    ok &= manager.insert(serial);
    ok &= manager.insert<Arp::RadioMain>(radioMain);
    ok &= manager.insert<Arp::EthernetGS>(ethernet);
    ok &= manager.insert<Arp::EthernetSniffer>(sniffer);
    ok &= manager.insert<Arp::RadioPayload>(radioPayload);
    ok &= manager.insert(boardStatus);
    ok &= manager.insert<Arp::BoardScheduler>(scheduler);
    ok &= manager.insert(sma);
    ok &= manager.insert(actuators);
    ok &= manager.insert(sensors);
    ok &= manager.insert(leds);
    ok &= manager.insert(pinHandler);
    ok &= manager.insert(arpStatus);
    ok &= manager.insert<Groundstation::HubBase>(hub);

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

    bool ok2       = true;
    bool initFatal = false;  //< set if the main radio failed to start

#ifndef NO_SD_LOGGING
    if (!Logger::getInstance().start())
    {
        std::cout << "ERROR: Failed to start Logger" << std::endl;
        ok2 = false;
    }
#endif

    LOG_INFO(logger, "DEBUG: serial starting...\n");
    if (!serial->start())
    {
        std::cout << "[error] Failed to start serial!" << std::endl;
        ok2 = false;
    }

    LOG_INFO(logger, "radio_main starting...\n");
    if (!radioMain->start())
    {
        std::cout << "[error] Failed to start radio_main!" << std::endl;
        ok2       = false;
        initFatal = true;
    }

    LOG_INFO(logger, "radio_payload starting...\n");
    if (!radioPayload->start())
    {
        // The payload radio is optional for ARP
        std::cout << "[error] Failed to start payload radio!" << std::endl;
    }

    LOG_INFO(logger, "DEBUG: ethernet starting...\n");
    if (!ethernet->start())
    {
        std::cout << "[error] Failed to start ethernet!" << std::endl;
        ok2 = false;
    }
    else
    {
        ethernet->printIpConfig(std::cout);
    }

    LOG_INFO(logger, "Board scheduler starting...\n");
    if (!scheduler->start())
    {
        std::cout << "[error] Failed to start BoardScheduler!" << std::endl;
        ok2 = false;
    }

    LOG_INFO(logger, "board_status starting...\n");
    if (!boardStatus->start())
    {
        std::cout << "[error] Failed to start board_status!" << std::endl;
        ok2 = false;
    }

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

    if (ok2)
        LOG_INFO(logger, "All modules started successfully!\n");

    reportStatus(boardStatus, logger);

    // If init fatal (radio_main failed) nothing can work without it.
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
        EventBroker::getInstance().post(Common::ARP_INIT_OK, Common::TOPIC_ARP);
    }

    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}
