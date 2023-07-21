/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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
#include <Main/Buses.h>
#include <Main/Radio/Radio.h>
#include <Main/TMRepository/TMRepository.h>
#include <drivers/interrupt/external_interrupts.h>

using namespace Boardcore;

#define SX1278_IRQ_DIO0 EXTI13_IRQHandlerImpl
#define SX1278_IRQ_DIO1 EXTI4_IRQHandlerImpl
#define SX1278_IRQ_DIO3 EXTI5_IRQHandlerImpl

void __attribute__((used)) SX1278_IRQ_DIO0()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ(
            EbyteFsk::Dio::DIO0);
    }
}

void __attribute__((used)) SX1278_IRQ_DIO1()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ(
            EbyteFsk::Dio::DIO1);
    }
}

void __attribute__((used)) SX1278_IRQ_DIO3()
{
    ModuleManager& modules = ModuleManager::getInstance();
    if (modules.get<Main::Radio>()->transceiver)
    {
        modules.get<Main::Radio>()->transceiver->handleDioIRQ(
            EbyteFsk::Dio::DIO3);
    }
}
namespace Main
{

Radio::Radio(TaskScheduler* sched) : scheduler(sched) {}

bool Radio::start()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Config the transceiver
    EbyteFsk::Config config;
    config.power    = 12;
    config.ocp      = 0;
    config.pa_boost = false;

    // Config the SPI
    SPIBusConfig spiConfig;
    spiConfig.clockDivider = SPI::ClockDivider::DIV_128;
    spiConfig.mode         = SPI::Mode::MODE_0;
    spiConfig.bitOrder     = SPI::Order::MSB_FIRST;
    spiConfig.writeBit     = SPI::WriteBit::INVERTED;

    transceiver = new EbyteFsk(SPISlave(modules.get<Buses>()->spi6,
                                        miosix::radio::cs::getPin(), spiConfig),
                               miosix::radio::tx_enable::getPin(),
                               miosix::radio::rx_enable::getPin());

    // Enable the interrupts
    enableExternalInterrupt(miosix::radio::dio0::getPin().getPort(),
                            miosix::radio::dio0::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(miosix::radio::dio1::getPin().getPort(),
                            miosix::radio::dio1::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);
    enableExternalInterrupt(miosix::radio::dio3::getPin().getPort(),
                            miosix::radio::dio3::getPin().getNumber(),
                            InterruptTrigger::RISING_EDGE);

    // Config the radio
    EbyteFsk::Error error = transceiver->init(config);

    // Add periodic telemetry send task
    uint8_t result =
        scheduler->addTask([=]() { this->sendPeriodicMessage(); },
                           RadioConfig::RADIO_PERIODIC_TELEMETRY_PERIOD,
                           TaskScheduler::Policy::RECOVER);

    // Config mavDriver
    // TODO change 0 and 1 into constants
    mavDriver = new MavDriver(
        transceiver,
        [=](MavDriver*, const mavlink_message_t& msg)
        { this->handleMavlinkMessage(msg); },
        0, 1);

    // Check radio failure
    if (error != EbyteFsk::Error::NONE)
    {
        return false;
    }

    // Start the mavlink driver thread
    return mavDriver->start() && result != 0;
}

void Radio::sendAck(const mavlink_message_t& msg) {}

void Radio::sendNack(const mavlink_message_t& msg) {}

void Radio::logStatus() {}

bool Radio::isStarted()
{
    return mavDriver->isStarted() && scheduler->isRunning();
}

void Radio::handleMavlinkMessage(const mavlink_message_t& msg) {}

void Radio::handleCommand(const mavlink_message_t& msg) {}

void Radio::sendPeriodicMessage()
{
    ModuleManager& modules = ModuleManager::getInstance();

    // Send all the queue messages
    {
        Lock<FastMutex> lock(queueMutex);

        for (uint8_t i = 0; i < messageQueueIndex; i++)
        {
            mavDriver->enqueueMsg(messageQueue[i]);
        }

        // Reset the index
        messageQueueIndex = 0;
    }

    mavDriver->enqueueMsg(
        modules.get<TMRepository>()->packSystemTm(MAV_FLIGHT_ID, 0, 0));
}

void Radio::enqueueMsg(const mavlink_message_t& msg)
{
    {
        Lock<FastMutex> lock(queueMutex);

        // Insert the message inside the queue only if there is enough space
        if (messageQueueIndex < RadioConfig::MAVLINK_QUEUE_SIZE)
        {
            messageQueue[messageQueueIndex] = msg;
            messageQueueIndex++;
        }
    }
}
}  // namespace Main