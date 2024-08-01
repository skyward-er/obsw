/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Main/Actuators/Actuators.h>
#include <Main/BoardScheduler.h>
#include <Main/Buses.h>
#include <Main/CanHandler/CanHandler.h>
#include <Main/Configs/RadioConfig.h>
#include <Main/PinHandler/PinHandler.h>
#include <Main/Sensors/Sensors.h>
#include <Main/StateMachines/ADAController/ADAController.h>
#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <Main/StateMachines/NASController/NASController.h>
#include <common/Mavlink.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Fsk.h>
#include <scheduler/TaskScheduler.h>

namespace Main
{

using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Fsk::MTU,
                                           Config::Radio::MAV_OUT_QUEUE_SIZE,
                                           Config::Radio::MAV_MAX_LENGTH>;

class Radio : public Boardcore::InjectableWithDeps<
                  Buses, BoardScheduler, Actuators, PinHandler, CanHandler,
                  Sensors, FlightModeManager, ADAController, NASController>
{
public:
    Radio() {}

    bool isStarted();

    [[nodiscard]] bool start();

    Boardcore::MavlinkStatus getMavStatus();

private:
    void enqueueAck(const mavlink_message_t& msg);
    void enqueueNack(const mavlink_message_t& msg);

    void enqueuePacket(const mavlink_message_t& msg);
    void flushPackets();

    void handleMessage(const mavlink_message_t& msg);
    void handleCommand(const mavlink_message_t& msg);

    bool enqueueSystemTm(uint8_t tmId);
    bool enqueueSensorsTm(uint8_t tmId);

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");

    Boardcore::CircularBuffer<mavlink_message_t,
                              Config::Radio::CIRCULAR_BUFFER_SIZE>
        queuedPackets;

    std::atomic<bool> started{false};
    std::unique_ptr<Boardcore::SX1278Fsk> radio;
    std::unique_ptr<MavDriver> mavDriver;
};

}  // namespace Main