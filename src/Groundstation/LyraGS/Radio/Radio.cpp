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

using namespace LyraGS;
using namespace Boardcore;
using namespace miosix;

SX1278Fsk* radioMainGlobal{nullptr};
SX1278Fsk* radioPayloadGlobal{nullptr};

void __attribute__((used)) MIOSIX_RADIO1_DIO0_IRQ()
{
    SX1278Fsk* instance = radioMainGlobal;
    if (instance)
        instance->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO1_DIO1_IRQ()
{
    SX1278Fsk* instance = radioMainGlobal;
    if (instance)
        instance->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO1_DIO3_IRQ()
{
    SX1278Fsk* instance = radioMainGlobal;
    if (instance)
        instance->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO0_IRQ()
{
    if (radioPayloadGlobal)
        radioPayloadGlobal->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO1_IRQ()
{
    if (radioPayloadGlobal)
        radioPayloadGlobal->handleDioIRQ();
}

void __attribute__((used)) MIOSIX_RADIO2_DIO3_IRQ()
{
    if (radioPayloadGlobal)
        radioPayloadGlobal->handleDioIRQ();
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
            getModule<Buses>()->radio1_bus, radio1::cs::getPin(),
            radio1::dio0::getPin(), radio1::dio1::getPin(),
            radio1::dio3::getPin(), SPI::ClockDivider::DIV_64,
            std::move(frontend));

    // Set the global radio for IRQ
    radioMainGlobal = sx1278.get();

    // First check if the device is even connected
    bool present = sx1278->checkVersion();

    if (present)
    {
        getModule<LyraGS::BoardStatus>()->setRadio433Present(hasBackup);

        // Configure the radio
        if (sx1278->configure(Common::MAIN_RADIO_CONFIG) !=
            SX1278Fsk::Error::NONE)
            return false;

        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278)))

            return false;
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
        frontend = std::make_unique<Skyward433Frontend>();

    sx1278 = std::make_unique<Boardcore::SX1278Fsk>(
        getModule<Buses>()->radio2_bus, radio2::cs::getPin(),
        radio2::dio0::getPin(), radio2::dio1::getPin(), radio2::dio3::getPin(),
        SPI::ClockDivider::DIV_64, std::move(frontend));

    // Set the global radio for IRQ
    radioPayloadGlobal = sx1278.get();

    // First check if the device is even connected
    bool present = sx1278->checkVersion();

    if (present)
    {
        getModule<LyraGS::BoardStatus>()->setRadio868Present(hasBackup);

        // Configure the radio
        if (sx1278->configure(Common::PAYLOAD_RADIO_CONFIG) !=
            SX1278Fsk::Error::NONE)
        {
            return false;
        }

        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278)))
            return false;
    }

    return true;
}

bool RadioMain::sendMsg(const mavlink_message_t& msg)
{
    if (txEnable)
        return RadioBase::sendMsg(msg);
    return false;
};

/**
 * @brief Send a mavlink message through this radio if it has been enabled
 * by dipSwitch
 *
 * @returns false when the queue is full.
 */
bool RadioPayload::sendMsg(const mavlink_message_t& msg)
{
    if (txEnable)
        return RadioBase::sendMsg(msg);
    return false;
};
