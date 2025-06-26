/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

#include <ConRIGv2/BoardScheduler.h>
#include <ConRIGv2/Buses.h>
#include <ConRIGv2/Buttons/Buttons.h>
#include <ConRIGv2/Configs/RadioConfig.h>
#include <ConRIGv2/Hub/Hub.h>
#include <common/MavlinkOrion.h>
#include <diagnostic/PrintLogger.h>
#include <drivers/timer/PWM.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Lora.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/collections/CircularBuffer.h>

#include <cstdint>
#include <thread>

namespace ConRIGv2
{

using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Lora::MTU,
                                           Config::Radio::MAV_OUT_QUEUE_SIZE,
                                           Config::Radio::MAV_MAX_LENGTH>;

class Radio
    : public Boardcore::InjectableWithDeps<Buses, BoardScheduler, Buttons, Hub>
{
public:
    Radio();

    [[nodiscard]] bool start();

    Boardcore::MavlinkStatus getMavlinkStatus();

    void updateButtonState(const mavlink_conrig_state_tc_t& state);

    bool enqueueMessage(const mavlink_message_t& msg);

private:
    /**
     * @brief Send ConRIG state to the RIG and all pending messages.
     *
     * @note Button state is reset after sending the state.
     */
    void sendPeriodicPing();

    void buzzerTask();
    void handleMessage(const mavlink_message_t& msg);

    void resetButtonState(const Lock<FastMutex>& /*lock*/);

    std::unique_ptr<Boardcore::SX1278Lora> radio;
    std::unique_ptr<MavDriver> mavDriver;

    void buzzerOn();
    void buzzerOff();

    Boardcore::PWM buzzer;

    std::atomic<uint32_t> buzzerCounter{0};

    Boardcore::CircularBuffer<mavlink_message_t,
                              Config::Radio::CIRCULAR_BUFFER_SIZE>
        messageQueue;

    miosix::FastMutex queueMutex;

    // Button internal state
    miosix::FastMutex buttonsMutex;
    mavlink_conrig_state_tc_t buttonState{};

    // Thread for periodic pings
    miosix::Thread* pingThread = nullptr;

    std::atomic<uint8_t> messagesReceived{
        Config::Radio::AUDIO_FEEDBACK_RESET_VALUE};
    std::atomic<bool> isArmed{false};

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");
};

}  // namespace ConRIGv2
