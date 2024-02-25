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

#include <RIGv2/Configs/RadioConfig.h>
#include <common/Mavlink.h>
#include <interfaces-impl/hwmapping.h>
#include <radio/MavlinkDriver/MavlinkDriver.h>
#include <radio/SX1278/SX1278Lora.h>
#include <utils/collections/CircularBuffer.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIGv2
{
using MavDriver = Boardcore::MavlinkDriver<Boardcore::SX1278Lora::MTU,
                                           Config::Radio::MAV_OUT_QUEUE_SIZE,
                                           Config::Radio::MAV_MAX_LENGTH>;

class Radio : public Boardcore::Module
{
public:
    Radio() {}

    [[nodiscard]] bool start();

    void stop();

    bool isStarted();

    Boardcore::MavlinkStatus getMavStatus();

private:
    void sendAck(const mavlink_message_t& msg);
    void sendNack(const mavlink_message_t& msg);

    void enqueuePacket(const mavlink_message_t &msg);
    void flushPackets();

    void handleMessage(const mavlink_message_t& msg);
    void handleCommand(const mavlink_message_t& msg);
    void handleConrigState(const mavlink_message_t& msg);
    
    bool packSystemTm(uint8_t tmId, mavlink_message_t& msg);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("radio");

    Boardcore::CircularBuffer<mavlink_message_t,
                              Config::Radio::CIRCULAR_BUFFER_SIZE>
        queuedPackets;

    std::atomic<bool> started{false};
    std::unique_ptr<Boardcore::SX1278Lora> radio;
    std::unique_ptr<MavDriver> mavDriver;
};

}  // namespace RIGv2