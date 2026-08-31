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

#include "GsEntryCommon.h"

#include <drivers/DipSwitch/DipSwitch.h>
#include <logger/Logger.h>
#include <miosix.h>

using namespace Boardcore;
using namespace miosix;

namespace Groundstation
{

static bool constexpr randomIp         = false;
static bool constexpr ethernetSniffing = true;

DipStatus readDipStatus()
{
    uint32_t microSecClk = 100;
    GpioPin sh           = dipSwitch::sh::getPin();
    GpioPin clk          = dipSwitch::clk::getPin();
    GpioPin qh           = dipSwitch::qh::getPin();

    DipSwitch dip(sh, clk, qh, std::chrono::microseconds(microSecClk));
    uint8_t read = dip.read();

    DipStatus dipRead;
    dipRead.isARP            = 1 & read;
    dipRead.mainHasBackup    = 1 & (read >> 1);
    dipRead.payloadHasBackup = 1 & (read >> 2);
    dipRead.mainTXenable     = 1 & (read >> 3);
    dipRead.payloadTXenable  = 1 & (read >> 4);
    dipRead.ipConfig         = read >> 5;
    return dipRead;
}

bool insertCommonModules(DependencyManager& manager, const DipStatus& dipRead,
                         CommonModules& outMods)
{
    outMods.scheduler = new Antennas::BoardScheduler();
    outMods.buses     = new ArpGS::Buses();
    outMods.serial    = new ArpGS::SerialArpGS();
    outMods.radioMain =
        new ArpGS::RadioMain(dipRead.mainHasBackup, dipRead.mainTXenable);
    outMods.boardStatus = new ArpGS::BoardStatus(dipRead.isARP);
    outMods.ethernet = new ArpGS::EthernetGS(randomIp, dipRead.ipConfig,
                                             dipRead.isARP & ethernetSniffing);
    outMods.ethernetSniffer = new EthernetSniffer();
    outMods.radioPayload    = new ArpGS::RadioPayload(dipRead.payloadHasBackup,
                                                      dipRead.payloadTXenable);

    bool ok = true;
    ok &= manager.insert(outMods.buses);
    ok &= manager.insert(outMods.serial);
    ok &= manager.insert<ArpGS::RadioMain>(outMods.radioMain);
    ok &= manager.insert<ArpGS::EthernetGS>(outMods.ethernet);
    ok &= manager.insert<EthernetSniffer>(outMods.ethernetSniffer);
    ok &= manager.insert<ArpGS::RadioPayload>(outMods.radioPayload);
    ok &= manager.insert(outMods.boardStatus);
    ok &= manager.insert<Antennas::BoardScheduler>(outMods.scheduler);
    return ok;
}

bool startCommonModules(CommonModules& mods, const DipStatus& dipRead,
                        PrintLogger& logger, bool& initFatal)
{
    bool ok   = true;
    initFatal = false;

#ifndef NO_SD_LOGGING
    if (!Logger::getInstance().start() && dipRead.isARP)
    {
        std::cout << "ERROR: Failed to start Logger" << std::endl;
        ok = false;
    }
#endif

    LOG_INFO(logger, "DEBUG: serial starting...\n");
    if (!mods.serial->start())
    {
        std::cout << "[error] Failed to start serial!" << std::endl;
        ok = false;
    }

    LOG_INFO(logger, "radio_main starting...\n");
    if (!mods.radioMain->start())
    {
        std::cout << "[error] Failed to start radio_main!" << std::endl;
        ok        = false;
        initFatal = true;
    }

    LOG_INFO(logger, "radio_payload starting...\n");
    if (!mods.radioPayload->start())
    {
        std::cout << "[error] Failed to start payload radio!" << std::endl;
        // Payload module is needed just for GS, not for ARP
        ok &= dipRead.isARP;
    }

    LOG_INFO(logger, "DEBUG: ethernet starting...\n");
    if (!mods.ethernet->start())
    {
        std::cout << "[error] Failed to start ethernet!" << std::endl;
        ok = false;
    }
    else
    {
        mods.ethernet->printIpConfig(std::cout);
    }

    LOG_INFO(logger, "Board scheduler starting...\n");
    if (!mods.scheduler->start())
    {
        std::cout << "[error] Failed to start BoardScheduler!" << std::endl;
        ok = false;
    }

    LOG_INFO(logger, "board_status starting...\n");
    if (!mods.boardStatus->start())
    {
        std::cout << "[error] Failed to start board_status!" << std::endl;
        ok = false;
    }

    return ok;
}

void reportCommonStatus(ArpGS::BoardStatus* boardStatus, PrintLogger& logger)
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

bool checkDipMatchesBinary(const DipStatus& dipRead, bool expectedArp,
                           PrintLogger& logger)
{
    if (dipRead.isARP != expectedArp)
    {
        std::cout << "[error] Dip switch / firmware mismatch! This board is "
                     "flashed with "
                  << (expectedArp ? "arp-gs-entry" : "radio-gs-entry")
                  << " firmware, but the dip switch ARP bit reads "
                  << (dipRead.isARP ? "ON" : "OFF")
                  << ". Check that the correct firmware is flashed on this "
                     "board :I or just cry"
                  << std::endl;
        return false;
    }
    return true;
}

void idleLoop()
{
    while (1)
        Thread::wait();
}

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

}  // namespace Groundstation
