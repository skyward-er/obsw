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

#include "DeathStack/System/StackLogger.h"
#include "DeathStack/configs/FMMConfig.h"
#include "DeathStack/events/Events.h"
#include "DeathStack/events/Topics.h"

#include "Debug.h"

namespace DeathStackBoard
{

FlightModeManager::FlightModeManager()
    : HSM(&FlightModeManager::state_initialization, 4096, 2),
      logger(*(LoggerService::getInstance()))
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
    StackLogger::getInstance()->updateStack(THID_FMM_FSM);
}

State FlightModeManager::state_initialization(const Event& ev)
{
    // Nothing to do during initialization

    UNUSED(ev);
    return transition(&FlightModeManager::state_onGround);
}

/* Handle TC_BOARD_RESET and TC_FORCE_LIFTOFF (super-state) */
State FlightModeManager::state_onGround(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::ON_GROUND);
            TRACE("[FMM] Entering state_onGround\n");
            break;
        }
        case EV_INIT: /* This is a super-state, so move to the first sub-state
                       */
        {
            TRACE("[FMM] Init state_onGround\n");

            retState = transition(&FlightModeManager::state_init);

            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting state_onGround\n");

            break;
        }
        case EV_TC_BOARD_RESET:
        {
            logger.stop();
            miosix::reboot();
            break;
        }
        case EV_TC_LAUNCH:
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT);
            TRACE("[FMM] Entering state_init\n");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exit state_init\n");

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
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_ERROR);
            TRACE("[FMM] Entering state_initError\n");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exit state_initError\n");

            break;
        }
        case EV_TC_FORCE_INIT:
        {
            retState = transition(&FlightModeManager::state_initDone);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_DONE);
            TRACE("[FMM] Entering state_initDone\n");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exit state_initDone\n");

            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            retState = transition(&FlightModeManager::state_calibrating);
            break;
        }
        case EV_TC_TEST_MODE:
        {
            retState = transition(&FlightModeManager::state_testing);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::CALIBRATING);

            sEventBroker->post({EV_CALIBRATE_ADA}, TOPIC_ADA);

            TRACE("[FMM] Entering calibration\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exit calibration\n");

            break;
        }
        case EV_ADA_READY:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post({EV_DISARMED}, TOPIC_FLIGHT_EVENTS);
            logState(FMMState::DISARMED);
            TRACE("[FMM] Entering disarmed\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting disarmed\n");

            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            retState = transition(&FlightModeManager::state_calibrating);
            break;
        }
        case EV_TC_ARM:
        {
            retState = transition(&FlightModeManager::state_armed);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post({EV_ARMED}, TOPIC_FLIGHT_EVENTS);
            logState(FMMState::ARMED);

            TRACE("[FMM] Entering armed\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting armed\n");

            break;
        }
        case EV_TC_DISARM:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        case EV_UMBILICAL_DETACHED:
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

// ACTUATORS test
State FlightModeManager::state_testing(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::TESTING);

            sEventBroker->post({EV_TEST_MODE}, TOPIC_FLIGHT_EVENTS);

            TRACE("[FMM] Entering testing\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting testing\n");

            break;
        }
        case EV_TC_NC_OPEN:
        {
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_DROGUE:
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_PRIMARY:
        {
            sEventBroker->post(Event{EV_CUT_PRIMARY}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CUT_BACKUP:
        {
            sEventBroker->post(Event{EV_CUT_BACKUP}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_TEST_CUTTER_PRIMARY:
        {
            sEventBroker->post(Event{EV_TEST_CUTTER_PRIMARY}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_TEST_CUTTER_BACKUP:
        {
            sEventBroker->post(Event{EV_TEST_CUTTER_BACKUP}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_RESET_SERVO:
        {
            sEventBroker->post(Event{EV_RESET_SERVO}, TOPIC_DEPLOYMENT);
            break;
        }
        case EV_TC_CLOSE_LOG:
        {
            logger.stop();
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::FLYING);

            sEventBroker->post({EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);
            // Start timeout for closing file descriptors
            end_mission_d_event_id =
                sEventBroker->postDelayed<TIMEOUT_FMM_END_MISSION>(
                    {EV_TIMEOUT_END_MISSION}, TOPIC_FMM);

            TRACE("[FMM] Entering flying\n");
            break;
        }
        case EV_INIT:
        {
            TRACE("[FMM] Init flying\n");

            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting flying\n");

            sEventBroker->removeDelayed(end_mission_d_event_id);
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
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::ASCENDING);
            TRACE("[FMM] Entering ascending\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exit ascending\n");

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
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            // Open nosecone
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);

            logState(FMMState::DROGUE_DESCENT);
            TRACE("[FMM] Entering drogueDescent\n");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            TRACE("[FMM] Exiting drogueDescent\n");

            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        case EV_TC_CUT_DROGUE:
        {
            retState = transition(&FlightModeManager::state_terminalDescent);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);

            logState(FMMState::TERMINAL_DESCENT);

            TRACE("[FMM] Entering terminalDescent\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            break;
        }
        case EV_TC_CUT_DROGUE:  // if you want to repeat cutting
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::state_flying);
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
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::LANDED);

            // Announce landing to all components
            sEventBroker->post(Event{EV_LANDED}, TOPIC_FLIGHT_EVENTS);
            logger.stop();

            TRACE("[FMM] Entering landed\n");
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

}  // namespace DeathStackBoard
