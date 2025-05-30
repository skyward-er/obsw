/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Buses.h>
#include <RIGv2/CanHandler/CanHandler.h>
#include <RIGv2/Configs/RadioConfig.h>
#include <RIGv2/Registry/Registry.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/GroundModeManager/GroundModeManager.h>
#include <RIGv2/StateMachines/TARS1/TARS1.h>
#include <RIGv2/StateMachines/TARS3/TARS3.h>
#include <common/MavlinkOrion.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Lora.h>
#include <utils/DependencyManager/DependencyManager.h>
#include <utils/collections/CircularBuffer.h>

namespace RIGv2
{
using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Lora::MTU,
                                           Config::Radio::MAV_OUT_QUEUE_SIZE,
                                           Config::Radio::MAV_MAX_LENGTH>;

class Radio
    : public Boardcore::InjectableWithDeps<Buses, BoardScheduler, Registry,
                                           Actuators, Sensors, CanHandler,
                                           GroundModeManager, TARS1, TARS3>
{
public:
    Radio() {}

    [[nodiscard]] bool start();

    bool isStarted();

    Boardcore::MavlinkStatus getMavStatus();

private:
    void enqueueAck(const mavlink_message_t& msg);
    void enqueueWack(const mavlink_message_t& msg, uint8_t errorId);
    void enqueueNack(const mavlink_message_t& msg, uint8_t errorId);

    void enqueueMessage(const mavlink_message_t& msg);
    void flushMessages();

    void handleMessage(const mavlink_message_t& msg);
    void handleCommand(const mavlink_message_t& msg);
    void handleConrigState(const mavlink_message_t& msg);

    void enqueueRegistry();

    bool enqueueSystemTm(uint8_t tmId);
    bool enqueueSensorTm(uint8_t tmId);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");

    Boardcore::CircularBuffer<mavlink_message_t,
                              Config::Radio::CIRCULAR_BUFFER_SIZE>
        queuedMessages;

    std::atomic<bool> started{false};
    std::unique_ptr<Boardcore::SX1278Lora> radio;
    std::unique_ptr<MavDriver> mavDriver;

    // Last time a ConRIG state triggered an actuation [ns]
    long long lastManualActuation = 0;
    // Last ConRIG state received and processed
    mavlink_conrig_state_tc_t lastConrigState;
};

}  // namespace RIGv2
