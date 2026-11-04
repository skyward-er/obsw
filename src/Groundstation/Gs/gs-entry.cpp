/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Tommaso Domenicali
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

#include <Groundstation/Common/DipStatus.h>
#include <Groundstation/Gs/BoardStatus.h>
#include <Groundstation/Gs/Buses.h>
#include <Groundstation/Gs/Hub.h>
#include <Groundstation/Gs/Ports/Ethernet.h>
#include <Groundstation/Gs/Ports/Serial.h>
#include <Groundstation/Gs/Radio/Radio.h>
#include <diagnostic/PrintLogger.h>
#include <interfaces-impl/bsp_impl.h>
#include <logger/Logger.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iostream>

using namespace Boardcore;
using namespace miosix;

/**
 * @brief Entry point of the plain groundstations
 *
 * Build/flash: ./sbs build gs-entry
 *
 * Modules (Gs/):
 *  - Buses:          SPI peripheral buses
 *  - Serial:         serial link to the PC
 *  - RadioMain:      the radio modules (also RadioPayload)
 *  - EthernetGS:     Ethernet link
 *  - Hub:            routes packets between radios, Ethernet and serial
 *  - BoardStatus:    periodic board telemetry (radio/Ethernet state, bitrates)
 *
 * Shared code (DIP, HubBase, configs, base classes) is in Common/.
 *!* Arp/ and Gs/ are independent: never include each other  *!*
 */

namespace
{

constexpr bool RANDOM_IP = false;

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
void reportStatus(Gs::BoardStatus* boardStatus, PrintLogger& logger)
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
 * @brief Radio-only GS entrypoint.
 *
 * For ground stations acting purely as a radio, so no ARP hardware attached
 * (no motors, no steppers, etc.). The ARP-capable variant lives in
 * Groundstation/Arp, the two only share Groundstation/Common.
 *
 * Initializes the software modules (radios, ethernet, serial)
 *  - Green LED is turned on when main radio is detected
 *  - Yellow LED is turned on when payload radio is detected
 *  - Orange LED is turned on when ethernet is detected
 * When done the red LED is fixed. Blinks if a module failed to init.
 */
int main()
{
    ledOff();

    Groundstation::DipStatus dipRead = Groundstation::readDipStatus();
    dipRead.print(std::cout);

    PrintLogger logger = Logging::getLogger("radio_gs");

    // This binary is the radio-only one: if the dip switch says ARP mode is ON
    // the board is probably flashed with the wrong firmware. Only a warning
    if (dipRead.isARP)
    {
        std::cout << "[warning] Dip switch ARP bit is ON, but this board is "
                     "flashed with the radio-only GS firmware (gs-entry)! "
                     "ARP features are NOT available. If this board is meant "
                     "to be an ARP flash arp-gs-entry, otherwise turn OFF the "
                     "ARP bit (dip switch 1)."
                  << std::endl;
    }

    DependencyManager manager;

    auto* buses  = new Gs::Buses();
    auto* serial = new Gs::Serial();
    auto* radioMain =
        new Gs::RadioMain(dipRead.mainHasBackup, dipRead.mainTXenable);
    auto* radioPayload =
        new Gs::RadioPayload(dipRead.payloadHasBackup, dipRead.payloadTXenable);
    auto* ethernet    = new Gs::EthernetGS(RANDOM_IP, dipRead.ipConfig);
    auto* boardStatus = new Gs::BoardStatus();
    Groundstation::HubBase* hub = new Gs::Hub();

    bool ok = true;
    ok &= manager.insert(buses);
    ok &= manager.insert(serial);
    ok &= manager.insert<Gs::RadioMain>(radioMain);
    ok &= manager.insert<Gs::EthernetGS>(ethernet);
    ok &= manager.insert<Gs::RadioPayload>(radioPayload);
    ok &= manager.insert(boardStatus);
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

    bool started = true;

#ifndef NO_SD_LOGGING
    // On the radio-only GS the SD logger is optional, ignore a failure.
    Logger::getInstance().start();
#endif

    LOG_INFO(logger, "DEBUG: serial starting...\n");
    if (!serial->start())
    {
        std::cout << "[error] Failed to start serial!" << std::endl;
        started = false;
    }

    LOG_INFO(logger, "radio_main starting...\n");
    if (!radioMain->start())
    {
        std::cout << "[error] Failed to start radio_main!" << std::endl;
        started = false;
    }

    LOG_INFO(logger, "radio_payload starting...\n");
    if (!radioPayload->start())
    {
        std::cout << "[error] Failed to start payload radio!" << std::endl;
        started = false;
    }

    LOG_INFO(logger, "DEBUG: ethernet starting...\n");
    if (!ethernet->start())
    {
        std::cout << "[error] Failed to start ethernet!" << std::endl;
        started = false;
    }
    else
    {
        ethernet->printIpConfig(std::cout);
    }

    LOG_INFO(logger, "board_status starting...\n");
    if (!boardStatus->start())
    {
        std::cout << "[error] Failed to start board_status!" << std::endl;
        started = false;
    }

    if (!started)
    {
        std::cout << "GS: could not start all modules successfully!"
                  << std::endl;
        errorLoop();
    }

    LOG_INFO(logger, "All modules started successfully!\n");

    reportStatus(boardStatus, logger);

    led3On();  //< fix RED led (CU)
    idleLoop();
    return 0;
}
