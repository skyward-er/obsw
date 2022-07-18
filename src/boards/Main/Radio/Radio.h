/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Main/Configs/RadioConfig.h>
#include <common/Mavlink.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SerialTransceiver/SerialTransceiver.h>
#include <scheduler/TaskScheduler.h>

namespace Main
{

using MavDriver = Boardcore::MavlinkDriver<RadioConfig::RADIO_PKT_LENGTH,
                                           RadioConfig::RADIO_OUT_QUEUE_SIZE,
                                           RadioConfig::RADIO_MAV_MSG_LENGTH>;

class Radio : public Boardcore::Singleton<Radio>
{
    friend class Boardcore::Singleton<Radio>;

public:
    Boardcore::SerialTransceiver* transceiver;
    MavDriver* mavDriver;

    /**
     * @brief Called by the MavlinkDriver when a message is received.
     */
    void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

    /**
     * @brief Called by handleMavlinkMessage to handle a command message.
     */
    void handleCommand(const mavlink_message_t& msg);

    /**
     * @brief Prepares and send an ack message for the given message.
     *
     * @param msg The message received that we need to acknowledge.
     */
    void sendAck(const mavlink_message_t& msg);

    /**
     * @brief Prepares and send a nack message for the given message.
     *
     * @param msg The message received that we need to not acknowledge.
     */
    void sendNack(const mavlink_message_t& msg);

    /**
     * @brief Starts the MavlinkDriver.
     */
    bool start();

    /**
     * @brief Tells whether the radio was started.
     */
    bool isStarted();

    Boardcore::MavlinkStatus getMavlinkStatus();

    /**
     * @brief Saves the MavlinkDriver and transceiver status.
     */
    void logStatus();

    /**
     * @brief Used to send the specified system telemetry message.
     */
    bool sendSystemTm(const SystemTMList tmId);

    /**
     * @brief Used to send the specified sensors telemetry message.
     */
    bool sendSensorsTm(const SensorsTMList tmId);

private:
    Radio();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace Main
