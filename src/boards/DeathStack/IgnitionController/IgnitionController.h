/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include "IgnitionStatus.h"

#include <events/FSM.h>
#include <DeathStack/LogProxy/LogProxy.h>
#include <DeathStack/Canbus/CanProxy.h>

namespace DeathStackBoard
{

class IgnitionController : public FSM<IgnitionController>
{

public:
    /**
     * @brief Initialize the controller and subscribe to EventBroker topics
     */
    IgnitionController(CanProxy* canbus);
    ~IgnitionController() {}

private:
    void stateIdle(const Event& ev);
    void stateAborted(const Event& ev);
    void stateEnd(const Event& ev);

    /**
     * @brief Put timestamp and log the component's status 
     * @param None
     * @return None
     */
    void logStatus();

    /**
     * @brief Updates the status of the ignition board if received on the canbus
     *        and logs it on the logger.
     *
     * @param ev     The NEW_CAN_MSG event received
     * @return true  the received event contained an IGN_STATUS message: updated
     *                internal board status struct
     * @return false the received message was not directed to me: not handled
     */
    bool updateIgnBoardStatus(const Event& ev);

    // Internal stats
    IgnCtrlStatus status;
    // Status received from the board, with timestamp
    IgnBoardLoggableStatus loggable_board_status;

    CanProxy* canbus;
    LoggerProxy& logger = *(LoggerProxy::getInstance());

    // Id of the IGN_OFFLINE delayed event posted in the Event Broker
    // NOTE: this is needed to cancel the delayed event
    uint16_t ign_offline_delayed_id = 0;
    // Id of the GET_IGN_STATUS delayed event posted in the Event Broker'
    // NOTE: this is needed to cancel the delayed event
    uint16_t get_status_delayed_id  = 0;
};

}  // namespace DeathStackBoard