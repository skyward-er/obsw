/* Copyright (c) 2022 Skyward Experimental Rocketry
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
#include <Payload/Main/Radio.h>
#include <Payload/Payload.h>
#include <drivers/interrupt/external_interrupts.h>
#include <radio/Xbee/ATCommands.h>

using namespace Boardcore;
using namespace std::placeholders;

/**
 * @brief We must define the interrupt handler. This calls
 * the message handler which is: handleMavlinkMessage
 */
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    if (Payload::Payload::getInstance().radio->xbee != nullptr)
    {
        Payload::Payload::getInstance().radio->xbee->handleATTNInterrupt();
    }
}

namespace Payload
{
Radio::Radio(SPIBusInterface& xbeeBus, TaskScheduler* scheduler)
    : xbeeBus(xbeeBus)
{
    // Create the SPI bus configuration
    SPIBusConfig config{};
    config.clockDivider = SPI::ClockDivider::DIV_16;

    // Set the internal scheduler
    this->scheduler = scheduler;

    // Create the xbee object
    xbee = new Xbee::Xbee(xbeeBus, config, miosix::xbee::cs::getPin(),
                          miosix::xbee::attn::getPin(),
                          miosix::xbee::reset::getPin());

    // Create the mavlink driver with the radio
    mavDriver =
        new MavDriver(xbee, bind(&Radio::handleMavlinkMessage, this, _1, _2),
                      SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
}

Radio::~Radio()
{
    // Destruct the mavlink driver and the radio
    delete mavDriver;
    delete xbee;
}

void Radio::handleMavlinkMessage(MavDriver* driver,
                                 const mavlink_message_t& msg)
{
}

bool Radio::sendTelemetry(const uint8_t tmId) { return false; }

void Radio::sendAck(const mavlink_message_t& msg)
{
    mavlink_message_t ackMsg;

    // Pack the ack message based on the received one
    mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid,
                            msg.seq);

    // Send the message
    mavDriver->enqueueMsg(ackMsg);
}

bool Radio::start()
{
    // Init the radio module
    init();

    // Start the mavlink driver and the scheduler
    return mavDriver->start() && scheduler->start();
}

void Radio::logStatus()
{
    // TODO log the internal radio status
}

void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
{
    // TODO log the received frame
}

void Radio::init()
{
    // Enable the external interrupt
    enableExternalInterrupt(miosix::xbee::attn::getPin().getPort(),
                            miosix::xbee::attn::getPin().getNumber(),
                            InterruptTrigger::FALLING_EDGE);

    // Set the callback
    xbee->setOnFrameReceivedListener(
        bind(&Radio::onXbeeFrameReceived, this, _1));

    // Set the data rate
    Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);
}
}  // namespace Payload