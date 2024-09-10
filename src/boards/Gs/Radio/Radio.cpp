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

#include <Gs/Buses.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace miosix;
using namespace Gs;
using namespace Boardcore;

SX1278Fsk* radio1_ptr = nullptr;
SX1278Fsk* radio2_ptr = nullptr;

void __attribute__((used)) MIOSIX_RADIO1_DIO0_IRQ()
{
    if (radio1_ptr != nullptr)
    {
        radio1_ptr->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO1_DIO1_IRQ()
{
    if (radio1_ptr != nullptr)
    {
        radio1_ptr->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO1_DIO3_IRQ()
{
    if (radio1_ptr != nullptr)
    {
        radio1_ptr->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO2_DIO0_IRQ()
{
    if (radio2_ptr != nullptr)
    {
        radio2_ptr->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO2_DIO1_IRQ()
{
    if (radio2_ptr != nullptr)
    {
        radio2_ptr->handleDioIRQ();
    }
}

void __attribute__((used)) MIOSIX_RADIO2_DIO3_IRQ()
{
    if (radio2_ptr != nullptr)
    {
        radio2_ptr->handleDioIRQ();
    }
}

void RadioBase::sendMsg(const mavlink_message_t& msg)
{
    Lock<FastMutex> l(mutex);
    pending_msgs.put(msg);
}

bool RadioBase::start(std::unique_ptr<SX1278Fsk> sx1278,
                      const SX1278Fsk::Config& config)
{
    this->sx1278 = std::move(sx1278);

    // Configure the radio
    if (this->sx1278->configure(config) != SX1278Fsk::Error::NONE)
        return false;

    auto mav_handler = [this](MavDriver* channel, const mavlink_message_t& msg)
    { handleMsg(msg); };

    mav_driver = std::make_unique<MavDriver>(this->sx1278.get(), mav_handler,
                                             Gs::MAV_SLEEP_AFTER_SEND,
                                             Gs::MAV_OUT_BUFFER_MAX_AGE);

    if (!mav_driver->start())
        return false;

    return true;
}

bool RadioMain::start()
{
#ifdef SKYWARD_GS_MAIN_USE_BACKUP_RF
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio1::txen::getPin(),
                                        radio1::rxen::getPin());
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();
#endif

    std::unique_ptr<Boardcore::SX1278Fsk> sx1278 =
        std::make_unique<Boardcore::SX1278Fsk>(
            ModuleManager::getInstance().get<Gs::Buses>()->radio1_bus,
            radio1::cs::getPin(), radio1::dio0::getPin(),
            radio1::dio1::getPin(), radio1::dio3::getPin(),
            SPI::ClockDivider::DIV_64, std::move(frontend));

    // This is valid, as the module will never be deleted
    radio1_ptr = sx1278.get();

    return RadioBase::start(std::move(sx1278), Common::MAIN_RADIO_CONFIG);
}

bool RadioPayload::start()
{
#ifdef SKYWARD_GS_MAIN_USE_BACKUP_RF
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<EbyteFrontend>(radio2::txen::getPin(),
                                        radio2::rxen::getPin());
#else
    std::unique_ptr<SX1278::ISX1278Frontend> frontend =
        std::make_unique<Skyward433Frontend>();
#endif

    std::unique_ptr<Boardcore::SX1278Fsk> sx1278 =
        std::make_unique<Boardcore::SX1278Fsk>(
            ModuleManager::getInstance().get<Gs::Buses>()->radio2_bus,
            radio2::cs::getPin(), radio2::dio0::getPin(),
            radio2::dio1::getPin(), radio2::dio3::getPin(),
            SPI::ClockDivider::DIV_64, std::move(frontend));

    // This is valid, as the module will never be deleted
    radio2_ptr = sx1278.get();

    return RadioBase::start(std::move(sx1278), Common::PAYLOAD_RADIO_CONFIG);
}

void RadioBase::handleMsg(const mavlink_message_t& msg)
{
    // TODO

    /*
    if(msg.msgid == ) {
        flush();
    }
    */
}

void RadioBase::flush()
{
    // Why is this here?
    //
    // Basically even SyncPacketQueue is not 100% thread safe (due to a bug). So
    // we will use the good old "put a massive fucking mutex over everything"
    // trick to fix this until we find a better solution.
    Lock<FastMutex> l(mutex);
    while (!pending_msgs.isEmpty())
    {
        mav_driver->enqueueMsg(pending_msgs.pop());
    }
}