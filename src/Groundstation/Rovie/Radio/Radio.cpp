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
#include <Groundstation/Rovie/Buses.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace Groundstation;
using namespace GroundstationRovie;
using namespace Boardcore;
using namespace miosix;

SX1278Lora* gRadio{nullptr};

void handleDioIRQ()
{
    SX1278Lora* instance = gRadio;
    if (instance)
        instance->handleDioIRQ();
}

void setIRQRadio(SX1278Lora* radio)
{
    FastInterruptDisableLock dl;
    gRadio = radio;
}

void __attribute__((used)) MIOSIX_RADIO_DIO0_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO1_IRQ() { handleDioIRQ(); }
void __attribute__((used)) MIOSIX_RADIO_DIO3_IRQ() { handleDioIRQ(); }

bool RadioRig::start()
{
#ifdef SKYWARD_GS_MAIN_USE_BACKUP_RF
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio1::txen::getPin(),
                                        radio1::rxen::getPin());
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();
#endif

    sx1278 = std::make_unique<SX1278Lora>(
        getModule<Buses>()->radio1, radio1::cs::getPin(),
        radio1::dio0::getPin(), radio1::dio1::getPin(), radio1::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Store the global radio instance
    setIRQRadio(sx1278.get());

    // Configure the radio
    if (sx1278->configure(Common::RIG_RADIO_CONFIG) != SX1278Lora::Error::NONE)
        return false;

    auto mavHandler = [this](RadioMavDriver* channel,
                             const mavlink_message_t& msg) { handleMsg(msg); };

    mavDriver = std::make_unique<RadioMavDriver>(
        sx1278.get(), mavHandler, Groundstation::MAV_SLEEP_AFTER_SEND,
        Groundstation::MAV_OUT_BUFFER_MAX_AGE);

    if (!mavDriver->start())
        return false;

    started = true;
    return true;
}

void RadioRig::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    getModule<HubBase>()->dispatchIncomingMsg(msg);
}
