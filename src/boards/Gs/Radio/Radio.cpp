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
#include <Gs/Hub.h>
#include <Gs/Ports/Serial.h>
#include <Gs/Radio/RadioStatus.h>
#include <radio/SX1278/SX1278Frontends.h>

using namespace miosix;
using namespace Gs;
using namespace Boardcore;

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

void RadioBase::sendMsg(const mavlink_message_t& msg)
{
    Lock<FastMutex> l(mutex);
    pending_msgs.put(msg);
}

void RadioBase::handleDioIRQ()
{
    if (sx1278)
    {
        sx1278->handleDioIRQ();
    }
}

bool RadioBase::start(std::unique_ptr<SX1278Fsk> sx1278,
                      const SX1278Fsk::Config& config)
{
    this->sx1278 = std::move(sx1278);

    // Configure the radio
    if (this->sx1278->configure(config) != SX1278Fsk::Error::NONE)
    {
        return false;
    }

    auto mav_handler = [this](MavDriver* channel, const mavlink_message_t& msg)
    { handleMsg(msg); };

    mav_driver = std::make_unique<MavDriver>(this->sx1278.get(), mav_handler,
                                             Gs::MAV_SLEEP_AFTER_SEND,
                                             Gs::MAV_OUT_BUFFER_MAX_AGE);

    if (!mav_driver->start())
    {
        return false;
    }

    if (!ActiveObject::start())
    {
        return false;
    }

    return true;
}

void RadioBase::run()
{

    while (!shouldStop())
    {
        miosix::Thread::sleep(AUTOMATIC_FLUSH_PERIOD);

        // If enough time has passed, automatically flush.
        if (miosix::getTick() > last_eot_packet_ts + AUTOMATIC_FLUSH_DELAY)
        {
            flush();
        }
    }
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

    // First check if the device is even connected
    RadioStatus* status = ModuleManager::getInstance().get<RadioStatus>();
    // Set if the device is present
    status->setMainRadioPresent(sx1278->checkVersion());

    if (status->isMainRadioPresent())
    {
        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278), Common::MAIN_RADIO_CONFIG))
        {
            return false;
        }
    }

    return true;
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

    // First check if the device is even connected
    RadioStatus* status = ModuleManager::getInstance().get<RadioStatus>();
    // Set if the device is present
    status->setPayloadRadioPresent(sx1278->checkVersion());

    if (status->isPayloadRadioPresent())
    {
        // Initialize if only if present
        if (!RadioBase::start(std::move(sx1278), Common::PAYLOAD_RADIO_CONFIG))
        {
            return false;
        }
    }

    return true;
}

void RadioBase::handleMsg(const mavlink_message_t& msg)
{
    // Dispatch the message through the hub.
    ModuleManager::getInstance().get<Hub>()->dispatchIncomingMsg(msg);

    if (isEndOfTransmissionPacket(msg))
    {
        last_eot_packet_ts = miosix::getTick();
        flush();
    }
}

void RadioBase::flush()
{
    Lock<FastMutex> l(mutex);
    while (!pending_msgs.isEmpty())
    {
        mav_driver->enqueueMsg(pending_msgs.pop());
    }
}

bool RadioBase::isEndOfTransmissionPacket(const mavlink_message_t& msg)
{
    return msg.msgid == MAVLINK_MSG_ID_ROCKET_FLIGHT_TM ||
           msg.msgid == MAVLINK_MSG_ID_PAYLOAD_FLIGHT_TM;
}