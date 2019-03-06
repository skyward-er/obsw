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

#include "events/FSM.h"
#include "DeathStack/Events.h"
#include "DeathStack/configs/TMTCConfig.h"

#include <drivers/gamma868/Gamma868.h>
#include <drivers/mavlink/MavChannel.h>
#include <DeathStack/LogProxy/LogProxy.h>

namespace DeathStackBoard
{
/**
 * @brief This class handles the communication with the Ground Station.
 *        Uses Gamma868 transceiver with the Mavlink protocol.
 */
class TMTCManager : public FSM<TMTCManager>
{

public:
    TMTCManager();
    ~TMTCManager();

    /* Non-blocking send, logs status on the logger*/
    bool send(mavlink_message_t& msg);

    /* Status getter */
    MavStatus getStatus() {return channel->getStatus();}

private:
    Gamma868* device;
    MavChannel* channel;

    LoggerProxy& logger = *(LoggerProxy::getInstance());

    /* State handlers */
    void stateIdle(const Event& ev);
    void stateHighRateTM(const Event& ev);
    void stateLowRateTM(const Event& ev);
    void stateLanded(const Event& ev);
};

} /* namespace DeathStackBoard */
