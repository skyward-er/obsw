/* Copyright (c) 2018-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio
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

#include <FlightModeManager/FMMController.h>
#include <System/StackLogger.h>
#include <configs/FMMConfig.h>
#include <events/EventBroker.h>
#include <events/Events.h>
#include <events/Topics.h>
#include <miosix.h>

namespace DeathStackBoard
{

FMMController::FMMController()
    : HSM(&FMMController::state_initialization, STACK_MIN_FOR_SKYWARD,
          FMM_PRIORITY),
      logger(*(LoggerService::getInstance()))
{
    sEventBroker->subscribe(this, TOPIC_ADA);
    sEventBroker->subscribe(this, TOPIC_NAS);
    sEventBroker->subscribe(this, TOPIC_TMTC);
    sEventBroker->subscribe(this, TOPIC_FMM);
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
}

FMMController::~FMMController()
{
    // Unsubscribe from all topics
    sEventBroker->unsubscribe(this);
}

void FMMController::logState(FMMState current_state)
{
    status.timestamp = TimestampTimer::getTimestamp();
    status.state     = current_state;

    logger.log(status);
    StackLogger::getInstance()->updateStack(THID_FMM_FSM);
}

State FMMController::state_initialization(const Event& ev)
{
    // Nothing to do during initialization

    UNUSED(ev);
    return transition(&FMMController::state_onGround);
}

State FMMController::state_onGround(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::ON_GROUND);
            LOG_DEBUG(log, "Entering state_onGround");
            break;
        }
        case EV_INIT: /* This is a super-state, so move to the first sub-state
                       */
        {
            LOG_DEBUG(log, "Init state_onGround");

            retState = transition(&FMMController::state_init);

            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting state_onGround");

            break;
        }
        case EV_TC_RESET_BOARD:
        {
            logger.stop();
            miosix::reboot();
            break;
        }
        case EV_TC_LAUNCH:
        {
            retState = transition(&FMMController::state_flying);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            // Since this is an outer super-state, the parent is HSM_top
            retState = tran_super(&FMMController::Hsm_top);
            break;
        }
    }
    return retState;
}

State FMMController::state_init(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT);
            LOG_DEBUG(log, "Entering state_init");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit state_init");

            break;
        }
        case EV_INIT_ERROR:
        {
            retState = transition(&FMMController::state_initError);
            break;
        }
        case EV_INIT_OK:
        {
            retState = transition(&FMMController::state_initDone);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_initError(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_ERROR);
            LOG_DEBUG(log, "Entering state_initError");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit state_initError");

            break;
        }
        case EV_TC_FORCE_INIT:
        {
            retState = transition(&FMMController::state_initDone);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_initDone(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::INIT_DONE);
            LOG_DEBUG(log, "Entering state_initDone");
            break;
        }
        case EV_INIT: /* No sub-states */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit state_initDone");

            break;
        }
        case EV_TC_CALIBRATE_SENSORS:
        {
            retState = transition(&FMMController::state_sensorsCalibration);
            break;
        }
        case EV_TC_TEST_MODE:
        {
            retState = transition(&FMMController::state_testMode);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_sensorsCalibration(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::SENSORS_CALIBRATION);

            LOG_DEBUG(log, "Entering sensors_calibration");

            // TODO : CALIBRATE SENSORS
            DeathStack::getInstance()
                ->sensors->imu_bmx160_with_correction->calibrate();
            sEventBroker->post({EV_SENSORS_READY}, TOPIC_FLIGHT_EVENTS);

            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit sensors_calibration");

            break;
        }
        case EV_TC_CALIBRATE_SENSORS:
        {
            retState = transition(&FMMController::state_sensorsCalibration);
            break;
        }
        case EV_SENSORS_READY:
        {
            retState = transition(&FMMController::state_algosCalibration);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_algosCalibration(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::ALGOS_CALIBRATION);

            sEventBroker->post({EV_CALIBRATE_ADA}, TOPIC_ADA);
            sEventBroker->post({EV_CALIBRATE_NAS}, TOPIC_NAS);

            LOG_DEBUG(log, "Entering algos_calibration");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit algos_calibration");

            break;
        }
        case EV_TC_CALIBRATE_ALGOS:
        {
            retState = transition(&FMMController::state_algosCalibration);
            break;
        }
        case EV_ADA_READY:
        {
            ada_ready = true;
            if (nas_ready)
            {
                sEventBroker->post(Event{EV_CALIBRATION_OK},
                                   TOPIC_FLIGHT_EVENTS);
            }
            break;
        }
        case EV_NAS_READY:
        {
            nas_ready = true;
            if (ada_ready)
            {
                sEventBroker->post(Event{EV_CALIBRATION_OK},
                                   TOPIC_FLIGHT_EVENTS);
            }
            break;
        }
        case EV_CALIBRATION_OK:
        {
            retState = transition(&FMMController::state_disarmed);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_disarmed(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post({EV_DISARMED}, TOPIC_FLIGHT_EVENTS);
            logState(FMMState::DISARMED);
            LOG_DEBUG(log, "Entering disarmed");

            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting disarmed");

            break;
        }
        case EV_TC_CALIBRATE_ALGOS:
        {
            ada_ready = false;
            nas_ready = false;

            retState = transition(&FMMController::state_algosCalibration);
            break;
        }
        case EV_TC_CALIBRATE_SENSORS:
        {
            retState = transition(&FMMController::state_sensorsCalibration);
            break;
        }
        case EV_TC_ARM:
        {
            retState = transition(&FMMController::state_armed);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_armed(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post({EV_ARMED}, TOPIC_FLIGHT_EVENTS);
            logState(FMMState::ARMED);

            LOG_DEBUG(log, "Entering armed");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting armed");

            break;
        }
        case EV_TC_DISARM:
        {
            retState = transition(&FMMController::state_disarmed);
            break;
        }
        case EV_UMBILICAL_DETACHED:
        case EV_TC_LAUNCH:
        {
            retState = transition(&FMMController::state_flying);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::Hsm_top);
            break;
        }
    }
    return retState;
}

State FMMController::state_testMode(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::TESTING);

            LOG_DEBUG(log, "Entering testing");

            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting testing");

            break;
        }
        case EV_TC_NC_OPEN:
        {
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DPL);
            break;
        }
        case EV_TC_DPL_WIGGLE_SERVO:
        {
            sEventBroker->post(Event{EV_WIGGLE_SERVO}, TOPIC_DPL);
            break;
        }
        case EV_TC_DPL_RESET_SERVO:
        {
            sEventBroker->post(Event{EV_RESET_SERVO}, TOPIC_DPL);
            break;
        }
        case EV_TC_TEST_CUT_PRIMARY:
        {
            sEventBroker->post(Event{EV_TEST_CUT_PRIMARY}, TOPIC_DPL);
            break;
        }
        case EV_TC_TEST_CUT_BACKUP:
        {
            sEventBroker->post(Event{EV_TEST_CUT_BACKUP}, TOPIC_DPL);
            break;
        }
        case EV_TC_CUT_DROGUE:
        {
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DPL);
            break;
        }
        case EV_TC_ABK_WIGGLE_SERVO:
        {
            sEventBroker->post(Event{EV_WIGGLE_SERVO}, TOPIC_ABK);
            break;
        }
        case EV_TC_ABK_RESET_SERVO:
        {
            sEventBroker->post(Event{EV_RESET_SERVO}, TOPIC_ABK);
            break;
        }
        case EV_TC_TEST_ABK:
        {
            sEventBroker->post(Event{EV_TEST_ABK}, TOPIC_ABK);
            break;
        }
        case EV_TC_CLOSE_LOG:
        {
            logger.stop();
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_onGround);
            break;
        }
    }
    return retState;
}

State FMMController::state_flying(const Event& ev)
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
                sEventBroker->postDelayed<TIMEOUT_END_MISSION>(
                    {EV_TIMEOUT_END_MISSION}, TOPIC_FMM);

            LOG_DEBUG(log, "Entering flying");
            break;
        }
        case EV_INIT:
        {
            LOG_DEBUG(log, "Init flying");

            retState = transition(&FMMController::state_ascending);
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting flying");

            sEventBroker->removeDelayed(end_mission_d_event_id);
            break;
        }
        case EV_TC_NC_OPEN:
        {
            // Open nosecone command sent by GS in case of problems
            // during flight
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DPL);
            break;
        }
        case EV_TC_ABK_DISABLE:
        {
            // Disable aerobrakes command sent by GS in case of problems
            // during flight
            sEventBroker->post(Event{EV_DISABLE_ABK}, TOPIC_ABK);
            break;
        }
        case EV_TC_END_MISSION:
        case EV_TIMEOUT_END_MISSION:
        {
            retState = transition(&FMMController::state_landed);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::Hsm_top);
            break;
        }
    }
    return retState;
}

State FMMController::state_ascending(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            logState(FMMState::ASCENDING);
            LOG_DEBUG(log, "Entering ascending");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exit ascending");

            break;
        }
        case EV_TC_NC_OPEN:
        case EV_ADA_APOGEE_DETECTED:
        {
            // Notify of apogee all components
            sEventBroker->post(Event{EV_APOGEE}, TOPIC_FLIGHT_EVENTS);

            retState = transition(&FMMController::state_drogueDescent);
            break;
        }
        case EV_ADA_DISABLE_ABK:
        {
            // Send disable aerobrakes
            sEventBroker->post(Event{EV_DISABLE_ABK}, TOPIC_ABK);

            retState = transition(&FMMController::state_ascending);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_flying);
            break;
        }
    }
    return retState;
}

State FMMController::state_drogueDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            // Open nosecone
            sEventBroker->post(Event{EV_NC_OPEN}, TOPIC_DPL);

            logState(FMMState::DROGUE_DESCENT);
            LOG_DEBUG(log, "Entering drogueDescent");
            break;
        }
        case EV_INIT: /* No sub-state */
        {
            break;
        }
        case EV_EXIT: /* Executed everytime state is exited */
        {
            LOG_DEBUG(log, "Exiting drogueDescent");

            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        case EV_TC_CUT_DROGUE:
        {
            retState = transition(&FMMController::state_terminalDescent);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_flying);
            break;
        }
    }
    return retState;
}

State FMMController::state_terminalDescent(const Event& ev)
{
    State retState = HANDLED;
    switch (ev.sig)
    {
        case EV_ENTRY: /* Executed everytime state is entered */
        {
            sEventBroker->post({EV_DPL_ALTITUDE}, TOPIC_FLIGHT_EVENTS);

            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DPL);

            logState(FMMState::TERMINAL_DESCENT);

            LOG_DEBUG(log, "Entering terminalDescent");
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
            sEventBroker->post(Event{EV_CUT_DROGUE}, TOPIC_DPL);
            break;
        }
        default: /* If an event is not handled here, try with super-state */
        {
            retState = tran_super(&FMMController::state_flying);
            break;
        }
    }
    return retState;
}

State FMMController::state_landed(const Event& ev)
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

            LOG_DEBUG(log, "Entering landed");
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
            retState = tran_super(&FMMController::Hsm_top);
            break;
        }
    }
    return retState;
}

}  // namespace DeathStackBoard
