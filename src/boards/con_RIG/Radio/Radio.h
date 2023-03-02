/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Giacomo Caironi
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

#include <common/Mavlink.h>
#include <con_RIG/Configs/RadioConfig.h>
#include <diagnostic/PrintLogger.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/Ebyte.h>

#include <cstdint>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace con_RIG
{

using MavDriver = Boardcore::MavlinkDriver<RadioConfig::RADIO_PKT_LENGTH,
                                           RadioConfig::RADIO_OUT_QUEUE_SIZE,
                                           RadioConfig::RADIO_MAV_MSG_LENGTH>;

class Radio : public Boardcore::Module
{
public:
    Boardcore::SX1278Lora* transceiver;

    MavDriver* mavDriver;

    Radio();

    bool start();

    bool isStarted();

    Boardcore::MavlinkStatus getMavlinkStatus();

    void sendMessages();

    void loopReadFromUsart();

    void logStatus();

private:
    void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

    void mavlinkWriteToUsart(const mavlink_message_t& msg);

    void sendAck(const mavlink_message_t& msg);

    void sendNack(const mavlink_message_t& msg);

    mavlink_message_t message_queue[RadioConfig::MAVLINK_QUEUE_SIZE];
    uint8_t message_queue_index;
    miosix::FastMutex mutex;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace con_RIG
