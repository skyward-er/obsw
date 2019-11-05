/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "DeathStack/events/Events.h"
#include "DeathStack/configs/TMTCConfig.h"
#include "events/FSM.h"

#include <drivers/mavlink/MavlinkDriver.h>
#include <DeathStack/LoggerService/LoggerService.h>
#include <interfaces-impl/hwmapping.h>

namespace DeathStackBoard
{
/**
 * @brief This class handles the communication with the Ground Station.
 *        Uses Gamma868 transceiver with the Mavlink protocol.
 */
class TMTCManager : public FSM<TMTCManager>
{
    using Mav = MavlinkDriver<MAV_PKT_SIZE, MAV_OUT_QUEUE_LEN>;
public:
    TMTCManager();
    ~TMTCManager();

    /* Non-blocking send, logs status on the logger*/
    bool send(mavlink_message_t& msg);

    /* Status getter */
    MavlinkStatus getStatus() { return channel->getStatus(); }

    /* AO methods */
    bool start() override
    {
        device->start();
        bool ret = channel->start();
        ret      = (ret && FSM::start());
        return ret;
    }

    void stop() override
    {
        channel->stop();
        FSM::stop();
    }

private:
    Xbee_t* device;
    Mav* channel;

    LoggerService& logger = *(LoggerService::getInstance());

    /* State handlers */
    // void stateIdle(const Event& ev);
    void stateGroundTM(const Event& ev);
    void stateFlightTM(const Event& ev);
    void stateTestTM(const Event& ev);

    inline void packHRTelemetry(uint8_t* packet, unsigned int index);
    inline void packLRTelemetry(uint8_t* packet);

    uint16_t lr_event_id = 0;
    uint16_t hr_event_id = 0;
    uint16_t test_tm_event_id = 0;

    uint8_t hr_tm_index = 0;

    mavlink_hr_tm_t hr_tm_packet;
    mavlink_lr_tm_t lr_tm_packet;

    mavlink_message_t auto_telemetry_msg;
};

} /* namespace DeathStackBoard */
