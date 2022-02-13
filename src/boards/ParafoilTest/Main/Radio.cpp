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

#include <Main/Radio.h>
#include <TelemetriesTelecommands/TMRepository.h>
#include <Configs/XbeeConfig.h>
#include <drivers/spi/SPIDriver.h>
#include <drivers/Xbee/ATCommands.h>
#include <drivers/interrupt/external_interrupts.h>
#include <ParafoilTest.h>

using std::bind;
using namespace std::placeholders;
using namespace Boardcore;

// Xbee ATTN interrupt
/**
 * @brief We must define the interrupt handler. This calls
 * the message handler which is: handleMavlinkMessage
 */
void __attribute__((used)) EXTI10_IRQHandlerImpl()
{
    using namespace ParafoilTestDev;

    if (ParafoilTest::getInstance().radio->xbee != nullptr)
    {
        ParafoilTest::getInstance().radio->xbee->handleATTNInterrupt();
    }
}

namespace ParafoilTestDev
{
    /**
    * PUBLIC METHODS
    */
    Radio::Radio(SPIBusInterface& xbee_bus, TaskScheduler* scheduler) : xbee_bus(xbee_bus)
    {
        //Create a SPI configuration object
        SPIBusConfig config{};

        //Set the internal scheduler
        this -> scheduler = scheduler;

        //Add a clock divider config
        config.clockDivider = SPI::ClockDivider::DIV_16;

        //Instantiate the xbee object
        xbee = new Xbee::Xbee(xbee_bus, config,
                              XBEE_CS,
                              XBEE_ATTN,
                              XBEE_RESET);
        
        //Set the frame receive callback
        xbee -> setOnFrameReceivedListener(
                bind(&Radio::onXbeeFrameReceived, this, _1));

        //Set the data rate
        Xbee::setDataRate(*xbee, XBEE_80KBPS_DATA_RATE, XBEE_TIMEOUT);

        //Create the mavlink driver
        mav_driver = new MavDriver(xbee, 
                                   bind(&Radio::handleMavlinkMessage, this, _1, _2),
                                   SLEEP_AFTER_SEND, MAV_OUT_BUFFER_MAX_AGE);
        //Enable external interrupt on F10 pin
        enableExternalInterrupt(GPIOF_BASE, 10, InterruptTrigger::FALLING_EDGE);
    }

    Radio::~Radio()
    {

    }

    void Radio::handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg)
    {
        prova = true;
    }

    bool Radio::sendTelemetry(const uint8_t tm_id)
    {
        //Enqueue the message
        bool result = mav_driver -> enqueueMsg(TMRepository::getInstance().packTM(tm_id));
        //TODO log the operation
        return result;
    }

    void Radio::sendHRTelemetry()
    {
        mavlink_message_t msg;
        msg.msgid = 10;
        mav_driver -> enqueueMsg(msg);
    }

    void Radio::sendLRTelemetry()
    {

    }

    void Radio::sendAck(const mavlink_message_t& msg)
    {
        mavlink_message_t ackMsg;
        //Pack the ack message based on the received message
        mavlink_msg_ack_tm_pack(TMTC_MAV_SYSID, TMTC_MAV_COMPID, &ackMsg, msg.msgid, msg.seq);

        //Put the message in the queue
        mav_driver -> enqueueMsg(ackMsg);
        //TODO log the thing
    }

    bool Radio::start()
    {
        bool result = mav_driver -> start();
        //TODO start the scheduler
        return result;
    }

    void Radio::logStatus()
    {

    }

    /**
     * PRIVATE METHODS
     */
    void Radio::onXbeeFrameReceived(Xbee::APIFrame& frame)
    {

    }
}