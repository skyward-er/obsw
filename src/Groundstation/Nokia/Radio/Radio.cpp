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

#include <Groundstation/Common/Ports/Serial.h>
#include <Groundstation/Nokia/Buses.h>
#include <radio/SX1278/SX1278Frontends.h>

#include "interfaces-impl/hwmapping.h"

using namespace Groundstation;
using namespace GroundstationNokia;
using namespace Boardcore;
using namespace miosix;

#define SX1278_DIO0_IRQ EXTI6_IRQHandlerImpl
#define SX1278_DIO1_IRQ EXTI4_IRQHandlerImpl
#define SX1278_DIO3_IRQ EXTI11_IRQHandlerImpl

Radio* radioGlobal = nullptr;

void __attribute__((used)) SX1278_DIO0_IRQ()
{
    if (radioGlobal)
        radioGlobal->handleDioIRQ();
}

void __attribute__((used)) SX1278_DIO1_IRQ()
{
    if (radioGlobal)
        radioGlobal->handleDioIRQ();
}

void __attribute__((used)) SX1278_DIO3_IRQ()
{
    if (radioGlobal)
        radioGlobal->handleDioIRQ();
}

bool Radio::start()
{
#ifdef SKYWARD_GS_MAIN_USE_BACKUP_RF
#error "Backup RF not supported on nokia"
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();
#endif

    std::unique_ptr<Boardcore::SX1278Fsk> sx1278 =
        std::make_unique<Boardcore::SX1278Fsk>(
            getModule<GroundstationNokia::Buses>()->radio_bus,
            peripherals::ra01::pc13::cs::getPin(),
            peripherals::ra01::pc13::dio0::getPin(),
            peripherals::ra01::pc13::dio1::getPin(),
            peripherals::ra01::pc13::dio3::getPin(), SPI::ClockDivider::DIV_64,
            std::move(frontend));

    // First check if the device is even connected
    if (!sx1278->checkVersion())
        return false;

    // Configure the radio
    if (sx1278->configure(Common::MAIN_RADIO_CONFIG) != SX1278Fsk::Error::NONE)
        return false;

    // Initialize if only if present
    if (!RadioBase::start(std::move(sx1278)))
        return false;

    return true;
}
