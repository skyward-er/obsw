/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <RIG/Configs/RadioConfig.h>
#include <common/Mavlink.h>
#include <radio/MavlinkDriver/MavlinkDriverPigna.h>
#include <radio/SX1278/SX1278Lora.h>
#include <utils/collections/CircularBuffer.h>

#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
using MavDriver = Boardcore::MavlinkDriverPignaSlave<Boardcore::SX1278Lora::MTU,
                                           Config::Radio::RADIO_OUT_QUEUE_SIZE,
                                           Config::Radio::RADIO_MAV_MSG_LENGTH>;
class Radio : public Boardcore::Module
{
public:
    Radio();

    ~Radio();

    /**
     * @brief Starts the MavlinkDriver.
     */
    [[nodiscard]] bool start();

    /**
     * @brief Prepares and adds to the buffer an ack message for the given
     * message.
     *
     * @param msg The message received that we need to acknowledge.
     */
    void sendAck(const mavlink_message_t& msg);

    /**
     * @brief Prepares and adds to the buffer a nack message for the given
     * message.
     *
     * @param msg The message received that we need to not acknowledge.
     */
    void sendNack(const mavlink_message_t& msg);

    /**
     * @brief Saves the MavlinkDriver and transceiver status.
     */
    void logStatus();

    /**
     * @brief Returns if the radio module is correctly started
     */
    bool isStarted();

    Boardcore::SX1278Lora* transceiver = nullptr;
    MavDriver* mavDriver               = nullptr;

private:
    /**
     * @brief Called by the MavlinkDriver when a message is received.
     */
    void handleMavlinkMessage(const mavlink_message_t& msg);

    /**
     * @brief Called by handleMavlinkMessage to handle a command message.
     */
    void handleCommand(const mavlink_message_t& msg);

    mavlink_conrig_state_tc_t previousState;
    std::thread radioBackupDIO;

    // Specifies the last tick [ms] in which a command is executed
    long long int lastManualCommand = 0;  

    Boardcore::Logger& SDlogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Radio");
};
}  // namespace RIG