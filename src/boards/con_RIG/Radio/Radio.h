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
#include <radio/SX1278/SX1278Lora.h>
#include <scheduler/TaskScheduler.h>

#include <cstdint>
#include <thread>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace con_RIG
{

using MavDriver = Boardcore::MavlinkDriver<Config::Radio::RADIO_PKT_LENGTH,
                                           Config::Radio::RADIO_OUT_QUEUE_SIZE,
                                           Config::Radio::RADIO_MAV_MSG_LENGTH>;

class Radio : public Boardcore::Module
{
public:
    explicit Radio(Boardcore::TaskScheduler* sched);

    bool start();

    bool isStarted();

    Boardcore::MavlinkStatus getMavlinkStatus();

    void sendMessages();

    void loopReadFromUsart();

    void setInternalState(mavlink_conrig_state_tc_t state);

    Boardcore::SX1278Lora* transceiver = nullptr;
    MavDriver* mavDriver               = nullptr;

private:
    void handleMavlinkMessage(MavDriver* driver, const mavlink_message_t& msg);

    void mavlinkWriteToUsart(const mavlink_message_t& msg);

    mavlink_message_t message_queue[Config::Radio::MAVLINK_QUEUE_SIZE];
    uint8_t message_queue_index = 0;
    miosix::FastMutex mutex;
    miosix::FastMutex internalStateMutex;

    // Button internal state
    mavlink_conrig_state_tc_t buttonState;

    std::thread receiverLooper;
    std::thread beeperLooper;
    std::atomic<uint8_t> messageReceived{0};
    Boardcore::TaskScheduler* scheduler = nullptr;
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace con_RIG
