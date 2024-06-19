/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include "Radio.h"

#include <Groundstation/Base/BoardStatus.h>
#include <Groundstation/Base/Buses.h>
#include <Groundstation/Base/Hub.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Groundstation;
using namespace GroundstationBase;
using namespace Boardcore;
using namespace miosix;

void __attribute__((used)) MIOSIX_RADIO1_DIO0_IRQ()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO1_DIO1_IRQ()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO1_DIO3_IRQ()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO0_IRQ()
{
    ModuleManager::getInstance().get<RadioPayload>()->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO1_IRQ()
{
    ModuleManager::getInstance().get<RadioPayload>()->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO3_IRQ()
{
    ModuleManager::getInstance().get<RadioPayload>()->handleDioIRQ();
}

bool RadioMain::start()
{

    std::unique_ptr<SX1278::ISX1278Frontend> frontend;

    if (hasBackup)
        frontend = std::make_unique<EbyteFrontend>(radio1::txen::getPin(),
                                                   radio1::rxen::getPin());
    else
        frontend = std::make_unique<Skyward433Frontend>();

    std::unique_ptr<Boardcore::SX1278Fsk> sx1278 =
        std::make_unique<Boardcore::SX1278Fsk>(
            ModuleManager::getInstance().get<Buses>()->radio1_bus,
            radio1::cs::getPin(), radio1::dio0::getPin(),
            radio1::dio1::getPin(), radio1::dio3::getPin(),
            SPI::ClockDivider::DIV_64, std::move(frontend));

    // First check if the device is even connected
    bool present = sx1278->checkVersion();

    ModuleManager::getInstance().get<BoardStatus>()->setMainRadioPresent(
        present);

    if (present)
    {
        // Configure the radio
        if (sx1278->configure(Common::MAIN_RADIO_CONFIG) !=
            SX1278Fsk::Error::NONE)
        {
            return false;
        }

        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278)))
        {
            return false;
        }
    }

    return true;
}

bool RadioPayload::start()
{
    std::unique_ptr<SX1278::ISX1278Frontend> frontend;
    std::unique_ptr<Boardcore::SX1278Fsk> sx1278;
    if (hasBackup)
        frontend = std::make_unique<EbyteFrontend>(radio2::txen::getPin(),
                                                   radio2::rxen::getPin());
    else
    {
        frontend = std::make_unique<Skyward433Frontend>();

        sx1278 = std::make_unique<Boardcore::SX1278Fsk>(
            ModuleManager::getInstance().get<Buses>()->radio2_bus,
            radio2::cs::getPin(), radio2::dio0::getPin(),
            radio2::dio1::getPin(), radio2::dio3::getPin(),
            SPI::ClockDivider::DIV_64, std::move(frontend));
    }

    // First check if the device is even connected
    bool present = sx1278->checkVersion();

    ModuleManager::getInstance().get<BoardStatus>()->setPayloadRadioPresent(
        present);

    if (present)
    {
        // Configure the radio
        if (sx1278->configure(Common::PAYLOAD_RADIO_CONFIG) !=
            SX1278Fsk::Error::NONE)
        {
            return false;
        }

        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278)))
        {
            return false;
        }
    }

    return true;
}