/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
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
#ifndef SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H
#define SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H

#include "Singleton.h"

#include "FMMStatus.h"
#include "events/Event.h"
#include "events/HSM.h"
#include "DeathStack/LogProxy/LogProxy.h"

#include <miosix.h>

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

/**
 * Implementation of the Flight Mode Manager Finite State Machine
 */
class FlightModeManager : public HSM<FlightModeManager>
{
public:
    FlightModeManager();
    ~FlightModeManager();

    /// ON-GROUND 

    /* Handle TC_BOARD_RESET and TC_FORCE_LIFTOFF (super-state) */
    State state_onGround(const Event& ev);

    /* First state, wait for sensors and objects initialization */
    State state_init(const Event& ev);
    /* "Low power" state, log only if requested by TC */
    State state_initDone(const Event& ev);
    /* Init error, get stuck */
    State state_initError(const Event& ev);

    /* Test mode, listen to serial and print stuff on serial */
    State state_testMode(const Event& ev);

    /* Calibrating ADA with pressure samples */
    State state_calibrating(const Event& ev);
    /* All good, waiting for arm */
    State state_disarmed(const Event& ev);
    /* Ready to launch, listening detachment pin (or command) */
    State state_armed(const Event& ev);

    /// FLYING 

    /* Handle TC_OPEN and END_MISSION (super-state) */
    State state_flying(const Event& ev);                // super-state
    /* Liftoff */
    State state_ascending(const Event& ev);
    /* Apogee reached, deploy drogue and wait half altitude (or manual mode) */
    State state_drogueDescent(const Event& ev);

    /* Descent super-state */
    State state_terminalDescent(const Event& ev);       // super-state
    /* We don't trust the software: prevent ADA to deploy rogallo */
    State state_manualDescent(const Event& ev);
    /* We trust the software */
    State state_rogalloDescent(const Event& ev);

    /// LANDED

    /* Close file descriptors */
    State state_landed(const Event& ev);

    FMMStatus getStatus()
    {
        return status;
    }
private:
    void logState(FMMState current_state);

    LoggerProxy& logger;

    FMMStatus status;

    uint16_t id_delayed_arm_timeout = 0;
    uint16_t id_delayed_dpl_timeout = 0;
};

}  // namespace DeathStackBoard

#endif /* SRC_SHARED_BOARDS_HOMEONE_FLIGHTMODEMANAGER_FSM_H */
