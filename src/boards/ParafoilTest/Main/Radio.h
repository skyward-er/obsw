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
#include <radio/Xbee/Xbee.h>
#include <scheduler/TaskScheduler.h>
#include <common/events/Events.h>

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

        /**
         * @brief Method to change the telemetry rate
         */
        void toggleHRTelemetry();

        bool start();

        void logStatus();

    private:

        //1 to 1 corrispondence of every message to its event
        const std::map<uint8_t, uint8_t> tcMap = {
        {MAV_CMD_ARM, EV_TC_ARM},
        {MAV_CMD_DISARM, EV_TC_DISARM},

        {MAV_CMD_FORCE_INIT, EV_TC_FORCE_INIT},
        {MAV_CMD_FORCE_LAUNCH, EV_TC_LAUNCH},

        {MAV_CMD_NOSECONE_OPEN, EV_TC_NC_OPEN},
        {MAV_CMD_DPL_RESET_SERVO, EV_TC_DPL_RESET_SERVO},
        {MAV_CMD_DPL_WIGGLE_SERVO, EV_TC_DPL_WIGGLE_SERVO},
        {MAV_CMD_CUT_DROGUE, EV_TC_CUT_DROGUE},

        {MAV_CMD_ARB_RESET_SERVO, EV_TC_ABK_RESET_SERVO},
        {MAV_CMD_ARB_WIGGLE_SERVO, EV_TC_ABK_WIGGLE_SERVO},
        {MAV_CMD_DISABLE_AEROBRAKES, EV_TC_ABK_DISABLE},
        {MAV_CMD_TEST_AEROBRAKES, EV_TC_TEST_ABK},

        {MAV_CMD_CALIBRATE_ALGOS, EV_TC_CALIBRATE_ALGOS},
        {MAV_CMD_CALIBRATE_SENSORS, EV_TC_CALIBRATE_SENSORS},

        {MAV_CMD_SERIAL_TM, EV_TC_SERIAL_TM},
        {MAV_CMD_RADIO_TM, EV_TC_RADIO_TM},

        {MAV_CMD_START_LOGGING, EV_TC_START_LOG},
        {MAV_CMD_CLOSE_LOG, EV_TC_CLOSE_LOG},

        {MAV_CMD_TEST_MODE, EV_TC_TEST_MODE},
        {MAV_CMD_BOARD_RESET, EV_TC_RESET_BOARD},
        {MAV_CMD_END_MISSION, EV_TC_END_MISSION}};

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
         * @brief high rate telemetry status.
         * false -> LR telemetry
         * true  -> HR telemetry
         */
        bool HRtelemetry;

        /**
         * @brief Logger
         */
        PrintLogger logger = Logging::getLogger("Radio");

        /**
         * @brief SD logger (pre started because of the ParafoilTest.h main class)
         */
        Logger* SDlogger = &Logger::getInstance();

        /**
         * @brief Radio frame received callback method.
         * It's used for logging purposes
         */
        void onXbeeFrameReceived(Xbee::APIFrame& frame);

        /**
         * @brief Method to initialize all the variables and objects
         * and to register the HR and LR tasks into the task scheduler
         */
        void init();
    };
}