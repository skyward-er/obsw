/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Federico Mandelli
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

#include <Payload/Configs/RadioConfig.h>
#include <common/MavlinkGemini.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Payload
{
using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Fsk::MTU,
                                           RadioConfig::RADIO_OUT_QUEUE_SIZE,
                                           RadioConfig::RADIO_MAV_MSG_LENGTH>;

class Radio : public Boardcore::Module
{
public:
    Radio(Boardcore::TaskScheduler& sched);

    /**
     * @brief Starts the MavlinkDriver
     */
    [[nodiscard]] bool start();

    /**
     * @brief Sends via radio an acknowledge message about the parameter passed
     * message
     */
    void sendAck(const mavlink_message_t& msg);

    /**
     * @brief Sends via radio an non-acknowledge message about the parameter
     * passed message
     */
    void sendNack(const mavlink_message_t& msg);

    /**
     * @brief Saves the MavlinkDriver and transceiver status
     */
    void logStatus();

    /**
     * @brief Returns if the radio module is correctly started
     */
    bool isStarted();

    Boardcore::SX1278Fsk* transceiver = nullptr;
    MavDriver* mavDriver              = nullptr;

private:
    /**
     * @brief Called by the MavlinkDriver when a message is received
     */
    void handleMavlinkMessage(const mavlink_message_t& msg);

    /**
     * @brief Called by the handleMavlinkMessage to handle a command message
     */
    void handleCommand(const mavlink_message_t& msg);

    /**
     * @brief Sends the periodic telemetry
     */
    void sendPeriodicMessage();

    /**
     * @brief Inserts the mavlink message into the queue
     */
    void enqueueMsg(const mavlink_message_t& msg);

    // Messages queue
    mavlink_message_t messageQueue[RadioConfig::MAVLINK_QUEUE_SIZE];
    uint32_t messageQueueIndex = 0;
    miosix::FastMutex queueMutex;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Radio");
    Boardcore::TaskScheduler& scheduler;
};

}  // namespace Payload
