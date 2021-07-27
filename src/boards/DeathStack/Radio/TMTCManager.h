/* Copyright (c) 2018-2020 Skyward Experimental Rocketry
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

#include "DeathStack.h"

#include "configs/TMTCConfig.h"
#include "events/Events.h"
#include "events/FSM.h"
#include "DeathStack.h"

#include "LoggerService/LoggerService.h"

namespace DeathStackBoard
{

/**
 * @brief This class handles the communication with the Ground Station via the
 * Mavlink protocol. It is responsible of:
 *
 * - sending periodic telemetries to GS
 * - receiving and handling commands coming from GS
 * - fetching the last logged values when sending telemetries
 */
class TMTCManager : public FSM<TMTCManager>
{
public:
    /* Constructor */
    TMTCManager();

    /* Destructor */
    ~TMTCManager();
private:
/**
     * Non-blocking send. Adds the message to an out queue and logs the status.
     *
     * @param tm_id  the id of the TM to be sent. Sends a NACK if the tm was
     *               not found.
     * @return false if the message could not be enqueued for sending
     */
    bool send(const uint8_t tm_id);

    LoggerService& logger = *(LoggerService::getInstance());
    PrintLogger log = Logging::getLogger("deathstack.fsm.tmtc");

    uint16_t periodicHrEvId = 0;
    uint16_t periodicLrEvId = 0;
    uint16_t periodicSensEvId = 0;
    uint16_t periodicTestEvId = 0;

    uint8_t hrPktCounter    = 0;

    /* State handlers */
    void stateGroundTM(const Event& ev);
    void stateSensorTM(const Event& ev);
    void stateFlightTM(const Event& ev);
};

} /* namespace DeathStackBoard */
