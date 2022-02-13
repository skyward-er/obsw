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
#pragma once

#include <TelemetriesTelecommands/Mavlink.h>
#include <drivers/Xbee/Xbee.h>
#include <scheduler/TaskScheduler.h>

/**
 * @brief This class represents the radio module. It allows the communications
 * between the xbee radio module and the on board software. The main tasks are about
 * sending automatic updates to ground based on the current state (High rate telemetry
 * or Low rate telemetry) and handle the messages from the ground station.
 * To handle the various messages there is an apposit callback method and,
 * to pack the optional reply data we use the TMRepository singleton. 
 */

namespace ParafoilTestDev
{
    class Radio
    {
    public:
        /**
         * @brief The xbee module driver
         */
        Xbee::Xbee* xbee;

        bool prova = false;

        /**
         * @brief Construct a new Radio object
         * 
         * @param xbee_bus The Xbee SPI bus
         */
        Radio(SPIBusInterface& xbee_bus, TaskScheduler* scheduler);

        /**
         * @brief Destroy the Radio object
         */
        ~Radio();

        /**
         * @brief Method called when a mavlink message is received
         * 
         * @param msg The parsed message
         */
        void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

        /**
         * @brief This method is used to add in send queue
         * the requested message. It is necessary to call the 
         * TMrepository to process the actual packet.
         * 
         * @param tm_id The requested message id
         * @return boolean that indicates the operation's result
         */
        bool sendTelemetry(const uint8_t tm_id);

        /**
         * @brief Method automatically called by the task
         * scheduler that sends the high rate telemetry
         */
        void sendHRTelemetry();

        /**
         * @brief Method automatically called by the task
         * scheduler that sends the high rate telemetry
         */
        void sendLRTelemetry();

        /**
         * @brief Every time a message is received we send
         * an ack message to tell the ground station that we
         * received the message
         * 
         * @param msg The message received that we need to ack
         */
        void sendAck(const mavlink_message_t& msg);

        bool start();

        void logStatus();

    private:

        /**
         * @brief The mavlink driver
         */
        MavDriver* mav_driver;

        /**
         * @brief SPI bus
         */
        SPIBusInterface& xbee_bus;

        /**
         * @brief Main task scheduler
         */
        TaskScheduler* scheduler;

        /**
         * @brief Logger
         */
        PrintLogger logger = Logging::getLogger("Radio");

        /**
         * @brief Radio frame received callback method.
         * It's used for logging purposes
         */
        void onXbeeFrameReceived(Xbee::APIFrame& frame);
    };
}