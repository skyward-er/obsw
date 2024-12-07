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

#include <ConRIG/BoardScheduler.h>
#include <ConRIG/Buses.h>
#include <ConRIG/Buttons/Buttons.h>
#include <ConRIG/Configs/RadioConfig.h>
#include <ConRIG/Serial/Serial.h>
#include <common/MavlinkOrion.h>
#include <diagnostic/PrintLogger.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Lora.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/collections/CircularBuffer.h>

#include <cstdint>
#include <thread>

namespace ConRIG
{

using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Lora::MTU,
                                           Config::Radio::MAV_OUT_QUEUE_SIZE,
                                           Config::Radio::MAV_MAX_LENGTH>;

class Radio : public Boardcore::InjectableWithDeps<Buses, BoardScheduler,
                                                   Buttons, Serial>
{
public:
    Radio();

    [[nodiscard]] bool start();

    Boardcore::MavlinkStatus getMavlinkStatus();

    void setButtonsState(mavlink_conrig_state_tc_t state);

    bool enqueueMessage(const mavlink_message_t& msg);

private:
    void sendPeriodicPing();
    void loopBuzzer();
    void handleMessage(const mavlink_message_t& msg);

    std::unique_ptr<Boardcore::SX1278Lora> radio;
    std::unique_ptr<MavDriver> mavDriver;

    Boardcore::CircularBuffer<mavlink_message_t,
                              Config::Radio::CIRCULAR_BUFFER_SIZE>
        messageQueue;

    miosix::FastMutex queueMutex;
    miosix::FastMutex buttonsMutex;

    // Button internal state
    mavlink_conrig_state_tc_t buttonState;

    std::thread buzzerLooper;

    std::atomic<uint8_t> messagesReceived{0};
    std::atomic<bool> isArmed{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace ConRIG
