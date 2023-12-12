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

#include <Groundstation/Automated/BoardStatus.h>
#include <Groundstation/Automated/Buses.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Common/Ports/Serial.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Antennas;
using namespace Boardcore;
using namespace miosix;

void __attribute__((used)) EXTI6_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

void __attribute__((used)) EXTI4_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

void __attribute__((used)) EXTI5_IRQHandlerImpl()
{
    ModuleManager::getInstance().get<RadioMain>()->handleDioIRQ();
}

namespace Antennas
{

bool RadioMain::start()
{
#ifdef SKYWARD_GS_MAIN_USE_BACKUP_RF
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio::txen::getPin(),
                                        radio::rxen::getPin());
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();
#endif

    std::unique_ptr<Boardcore::SX1278Fsk> sx1278 =
        std::make_unique<Boardcore::SX1278Fsk>(
            ModuleManager::getInstance().get<Antennas::Buses>()->radio_bus,
            radio::cs::getPin(), radio::dio0::getPin(), radio::dio1::getPin(),
            radio::dio3::getPin(), SPI::ClockDivider::DIV_64,
            std::move(frontend));

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

}  // namespace Antennas