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
    : HSM(&FlightModeManager::state_initialization),
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

State FlightModeManager::state_initialization(const Event& ev)
{
    (void)ev; // Avoid unused warning
    return transition(&FlightModeManager::state_startup);
}

State FlightModeManager::state_startup(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::STARTUP);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_INIT_ERROR:
        {
            retState = transition(&FlightModeManager::state_error);
            break;
        }
        case EV_INIT_OK:
        {
            retState = transition(&FlightModeManager::state_online);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_error(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::ERROR);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_BOARD_RESET:
        {
            //Reset the board
            miosix::reboot();
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_online(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::ONLINE);
            break;
        }
        case EV_INIT:
        {
            retState = transition(&FlightModeManager::state_ADAConfig);
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_BOARD_RESET:
        {
            //Reset the board
            miosix::reboot();
            break;
        }
        // Goto error state if one of these 2 events
        case EV_IGN_ABORTED:
        case EV_IGN_OFFLINE:
        {
            retState = transition(&FlightModeManager::state_error);
            break;
        }
        case EV_TC_SETUP_MODE:
        {
            retState = transition(&FlightModeManager::state_testing);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_ADAConfig(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::ADA_CONFIG);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_ADA_READY:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::state_online);
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
        case EV_ENTRY:
        {
            logState(FMMState::DISARMED);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_ARM:
        {
            retState = transition(&FlightModeManager::state_armed);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::state_online);
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
        case EV_ENTRY:
        {
            logState(FMMState::ARMED);

            sEventBroker->post(Event{EV_ARMED}, TOPIC_FLIGHT_EVENTS);
            delayed_event_id = sEventBroker->postDelayed(
                Event{EV_TIMEOUT_ARM}, TOPIC_FMM, TIMEOUT_FMM_AUTO_DISARM);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            sEventBroker->removeDelayed(delayed_event_id);
            break;
        }
        case EV_TC_DISARM:
        case EV_TIMEOUT_ARM:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        case EV_TC_LAUNCH:
        {
            //Convert event to EV_LAUNCH
            LaunchEvent lev = static_cast<const LaunchEvent&>(ev);
            lev.sig = EV_LAUNCH;

            // Post launch event to the ignition controller
            sEventBroker->post(lev, TOPIC_IGNITION);

            retState = transition(&FlightModeManager::state_launching);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::state_online);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_testing(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::TESTING);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_BOARD_RESET:
        {
            //Reset the board
            miosix::reboot();
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
        case EV_TC_NC_STOP:
        {
            sEventBroker->post(Event{EV_NC_STOP}, TOPIC_DEPLOYMENT);
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
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_launching(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::LAUNCHING);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_DISARM:
        {
            retState = transition(&FlightModeManager::state_disarmed);
            break;
        }
        case EV_IGN_ABORTED:
        {
            retState = transition(&FlightModeManager::state_error);
            break;
        }
        case EV_UMBILICAL_DETACHED:
        {
            // Notify of LIFTOFF
            sEventBroker->post(Event{EV_LIFTOFF}, TOPIC_FLIGHT_EVENTS);

            retState = transition(&FlightModeManager::state_flying);
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
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
        case EV_ENTRY:
        {
            logState(FMMState::FLYING);
            break;
        }
        case EV_INIT:
        {
            retState = transition(&FlightModeManager::state_ascending);
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_NC_OPEN:
        {
            // Open nosecone override in case of problems during flight
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);
            break;
        }
        default:
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
        case EV_ENTRY:
        {
            logState(FMMState::ASCENDING);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_ADA_APOGEE_DETECTED:
        {
            // Notify of apogee
            sEventBroker->post(Event{EV_APOGEE}, TOPIC_FLIGHT_EVENTS);
            
            retState = transition(&FlightModeManager::state_drogueDescent);
            break;
        }
        default:
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
        case EV_ENTRY:
        {
            //Open nosecone
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DEPLOYMENT);

            logState(FMMState::DROGUE_DESCENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_MANUAL_MODE:
        case EV_TC_ABORT_LAUNCH:
        {
            retState = transition(&FlightModeManager::state_manualDescent);
            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        case EV_TIMEOUT_DPL_ALT:
        case EV_TC_CUT_FIRST_DROGUE:
        {
            retState = transition(&FlightModeManager::state_rogalloDescent);
            break;
        }
        default:
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
        case EV_ENTRY:
        {
            logState(FMMState::TERMINAL_DESCENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
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
        case EV_TC_END_MISSION:
        case EV_TIMEOUT_END_MISSION:
        {
            retState = transition(&FlightModeManager::state_landed);
            break;
        }
        default:
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
        case EV_ENTRY:
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DEPLOYMENT);
            sEventBroker->post(Event{EV_DPL_ALTITUDE}, TOPIC_FLIGHT_EVENTS);

            logState(FMMState::ROGALLO_DESCENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        case EV_TC_ABORT_LAUNCH:
        {
            sEventBroker->post(Event{EV_CUT_MAIN}, TOPIC_DEPLOYMENT);
        }
        default:
        {
            retState = tran_super(&FlightModeManager::state_terminalDescent);
            break;
        }
    }
    return retState;
}

State FlightModeManager::state_manualDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logState(FMMState::MANUAL_DESCENT);
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        default:
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
        case EV_ENTRY:
        {
            sEventBroker->post(Event{EV_LANDED}, TOPIC_FLIGHT_EVENTS);

            logState(FMMState::LANDED);    

            // Stop the logger.
            logger.stop();
            break;
        }
        case EV_INIT:
        {
            break;
        }
        case EV_EXIT:
        {
            break;
        }
        default:
        {
            retState = tran_super(&FlightModeManager::Hsm_top);
            break;
        }
    }
    return retState;
}

}  // namespace DeathStackBoard
