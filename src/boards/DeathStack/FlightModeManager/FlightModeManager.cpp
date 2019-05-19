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

#include <DeathStack/FlightModeManager/FlightModeManager.h>
#include <events/EventBroker.h>

#include "DeathStack/Events.h"
#include "DeathStack/Topics.h"
#include "DeathStack/configs/FMMConfig.h"

#include "Debug.h"

namespace DeathStackBoard
{

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_onGround),
      logger(*(LoggerProxy::getInstance()))
{
    sEventBroker->subscribe(this, TOPIC_ADA);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_FMM);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
}

FlightModeManager::~FlightModeManager()
{
    // Unsubscribe from all topics
    sEventBroker->unsubscribe(this);
}

void FlightModeManager::logState(FMMState current_state)
{
    status.timestamp = miosix::getTick();
    status.state     = current_state;

    logger.log(status);
}


/* Handle TC_BOARD_RESET and TC_FORCE_LIFTOFF (super-state) */
State FlightModeManager::state_onGround(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::ON_GROUND);
            TRACE("Entering state_onGround\n");
            break;
        }
        case EV_INIT:  /* This is a super-state, so move to the first sub-state */
        {
            // Announce liftoff to all components
            sEventBroker->post(Event{EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

            retState = transition(&FlightModeManager::state_init);
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_BOARD_RESET:
        {
            miosix::reboot();
            break;
        }
        case EV_TC_LAUNCH:
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            // Since this is an outer super-state, the parent is HSM_top
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

/* First state, wait for sensors and objects initialization */
State FlightModeManager::state_init(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::INIT);
            TRACE("Entering state_init\n");
            break;
        }
        case EV_INIT:  /* No sub-states */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_INIT_ERROR:
        {
            retState = transition(&FlightModeManager::state_initError);
            break;
        }
        case EV_INIT_OK:
        {
            retState = transition(&FlightModeManager::state_initDone);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_initError(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_ERROR);
            TRACE("Entering state_initError\n");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_FORCE_INIT:
        {
            retState = transition(&FlightModeManager::state_initDone);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}



State FlightModeManager::state_initDone(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_DONE);
            TRACE("Entering state_initDone\n");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_START_CALIB:
        {
            retState = transition(&FlightModeManager::state_calibrating);
            break;
        }
        case EV_TC_TEST_MODE:
        {
            retState = transition(&FlightModeManager::state_testing);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

/* Just wait the ADA event */
State FlightModeManager::state_calibrating(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::CALIBRATING);
            TRACE("Entering calibration\n");
            break;
        }
        case EV_INIT:  /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_ADA_READY:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_disarmed(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::DISARMED);
            TRACE("Entering disarmed\n");
            break;
        }
        case EV_INIT:  /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_RESET_CALIB:
        {
            retState = transition(&FlightModeManager::state_calibrating);
            break;
        }
        case EV_TC_ARM:
        {
            retState = transition(&FlightModeManager::state_armed);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_armed(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::ARMED);

            // Announce liftoff to all components
            sEventBroker->post(Event{EV_ARMED}, TOPIC_FLIGHT_EVENTS);

            TRACE("Entering armed\n");
            break;
        }
        case EV_INIT:  /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_DISARM:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        // TODO controllare polarità
        case EV_UMBILICAL_DETACHED:
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

// ACTUATORS test
State FlightModeManager::state_testing(const Event& ev)
{
    // TODO test state
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::TESTING);
            TRACE("Entering testing\n");
            break;
        }
        case EV_INIT:   /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_NC_OPEN:
        {
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_NC_CLOSE:
        {
            sEventBroker->post(Event{EV_NC_CLOSE}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_FIRST_DROGUE:
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_MAIN:
        {
            sEventBroker->post(Event{EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_TEST_SERVO:
        {
            // TODO test servo!
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_onGround);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_flying(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::FLYING);

            // Start timeout for closing file descriptors
            sEventBroker->postDelayed({EV_TIMEOUT_END_MISSION}, TOPIC_FMM,
                                      TIMEOUT_FMM_END_MISSION);

            TRACE("Entering flying\n");
            break;
        }
        case EV_INIT: 
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_NC_OPEN:
        {
            // Open nosecone command sent by GS in case of problems 
            // during flight
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_END_MISSION:
        case EV_TIMEOUT_END_MISSION:
        {
            retState = transition(&FlightModeManager::state_landed);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_ascending(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::ASCENDING);
            TRACE("Entering ascending\n");
            break;
        }
        case EV_INIT:  /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_NC_OPEN:
        case EV_ADA_APOGEE_DETECTED:
        {
            // Notify of apogee all components
            sEventBroker->post(Event{EV_APOGEE}, TOPIC_FLIGHT_EVENTS);

            retState = transition(&FlightModeManager::state_drogueDescent);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_flying);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_drogueDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::DROGUE_DESCENT);

            // Open nosecone
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);

            TRACE("Entering drogueDescent\n");
            break;
        }
        case EV_INIT:  /* No sub-state */
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_MANUAL_MODE:  // Manual mode = we don't trust ADA for deployment
        case EV_TC_ABORT_ROGALLO: // Abort rogallo = don't deploy, or cut if deployed
        {
            retState = transition(&FlightModeManager::state_manualDescent);
            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        case EV_TC_CUT_FIRST_DROGUE:
        {
            retState = transition(&FlightModeManager::state_rogalloDescent);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_flying);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_terminalDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::TERMINAL_DESCENT);

            TRACE("Entering terminalDescent\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_CUT_FIRST_DROGUE: // if you want to repeat cutting
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_MAIN:       // remove rogallo
        {
            sEventBroker->post(Event{EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_flying);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_rogalloDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::ROGALLO_DESCENT);

            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            sEventBroker->post(Event{EV_DPL_ALTITUDE}, TOPIC_FLIGHT_EVENTS);

            TRACE("Entering rogalloDescent\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_ABORT_ROGALLO:  // GS doesn't trust rogallo
        case EV_ABORT_ROGALLO:     // We are not in safe area
        {
            sEventBroker->post(Event{EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_terminalDescent);
            break;
        }
    }
    return retState;
}

/*  */
State FlightModeManager::state_manualDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::MANUAL_DESCENT);

            TRACE("Entering manualDescent\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_terminalDescent);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_landed(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:  /* Executed everytime state is entered */
        {
            logState(FMMState::LANDED);

            // Announce landing to all components
            sEventBroker->post(Event{EV_LANDED}, TOPIC_FLIGHT_EVENTS);
            logger.stop();

            TRACE("Entering landed\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:  /* Executed everytime state is exited */
        {
            break;
        }
        default:       /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

}  // namespace DeathStackBoard
