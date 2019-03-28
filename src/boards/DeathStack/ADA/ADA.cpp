/* Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli
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

#include <DeathStack/ADA/ADA.h>

#include <events/EventBroker.h>

#include <DeathStack/Events.h>
#include <DeathStack/Topics.h>

#include "Debug.h"

namespace DeathStackBoard
{

/* --- LIFE CYCLE --- */
ADA::ADA()
    : FSM(&ADA::stateCalibrating),
      filter(A_INIT, C_INIT, V1_INIT, V2_INIT, P_INIT)
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_ADA);
}

void ADA::updateFilter(float altitude)
{
    MatrixBase<float, 1, 1> y{altitude};
    filter.update(y);

    last_kalman_state.x0 = filter.X(0);
    last_kalman_state.x1 = filter.X(1);
    last_kalman_state.x2 = filter.X(2);

    logger.log(last_kalman_state);
}

void ADA::setTargetDPLAltitude(uint16_t altitude)
{
    dpl_target_altitude = altitude;
    logger.log(TargetDeploymentAltitude{altitude});
}

/* --- INSTANCE METHODS --- */

void ADA::update(float altitude)
{
    switch (status.state)
    {
        case ADAState::CALIBRATING:
        {
            // Calibrating state: update calibration data
            stats.add(altitude);
            calibrationData.stats = calibrationData.getStats();

            // Log calibration data
            logger.log(calibrationData);

            // Send event if calibration samples number is reached and
            // deployment altitude is set
            if (calibrationData.n_samples >= CALIBRATION_N_SAMPLES &&
                status.dpl_altitude_set)
            {
                sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);
            }
            break;
        }

        case ADAState::IDLE:
        {
            // Idle state: do nothing
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            updateFilter(altitude);

            // Check if the vertical speed is negative
            if (filter.X(1) < 0)
            {
                // Log
                ApogeeDetected apogee{status.state, miosix::getTick()};
                logger.log(apogee);
            }
            break;
        }

        case ADAState::ACTIVE:
        {
            // Active state send notifications for apogee
            updateFilter(altitude);
            // Check if the vertical speed is negative
            if (filter.X(1) < 0)
            {

                EventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                status.apogee_reached = true; 

                // Log
                ApogeeDetected apogee{status.state, miosix::getTick()};
                logger.log(apogee);
            }
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:
        {
            // Descent state: send notifications for target altitude reached
            updateFilter(altitude);

            if (filter.X(0) <= dpl_target_altitude)
            {
                sEventBroker->post({EV_DPL_ALTITUDE}, TOPIC_ADA);
                status.dpl_altitude_reached = true;

                // Log
                DplAltitudeReached dpl_alt{miosix::getTick()};
                logger.log(dpl_alt);
            }
            break;
        }

        case ADAState::END:
        {
            // End state: do nothing
            break;
        }

        case ADAState::UNDEFINED:
        {
            TRACE("ADA Update: Undefined state value \n");
        }

        default:
        {
            TRACE("ADA Update: Unexpected state value \n");
        }
    }
}

void ADA::logStatus(ADAState state)
{
    status.timestamp = miosix::getTick();
    status.state     = state;

    logger.log(status);
}

/* --- STATES --- */
/**
 * \brief Calibrating state: the ADA calibrates the initial state. This is the
 * initial state.
 *
 * In this state a call to update() will result in a altitude sample being added
 * to the average.
 * The exiting transition to the idle state is triggered at the first sample
 * update after having set the deployment altitude and having reached the
 * minimum number of calibration samples.
 */
void ADA::stateCalibrating(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateCalibrating\n");
            logStatus(ADAState::CALIBRATING);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateCalibrating\n");
            break;
        }
        case EV_ADA_READY:
        {
            transition(&ADA::stateIdle);
            break;
        }
        case EV_TC_SET_DPL_ALTITUDE:
        {
            const DeploymentAltitudeEvent& dpl_ev =
                static_cast<const DeploymentAltitudeEvent&>(ev);
            dpl_target_altitude = dpl_ev.dplAltitude;
            dpl_altitude_set    = true;

            break;
        }
        case EV_TC_RESET_CALIBRATION:
        {
            calibrationStats.stats.reset();
        }
        default:
        {
            TRACE("ADA stateCalibrating: %d event not handled", ev.sig);
            break;
        }
    }
}

/**
 * \brief Idle state:  ADA is ready and waiting for liftoff
 *
 * In this state a call to update() will have no effect.
 * The exiting transition to the shadow mode state is triggered by the liftoff
 * event.
 */
void ADA::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateIdle\n");
            filter.X(0) = calibrationData.stats.mean;  // Initialize the state with the average
            logStatus(ADAState::IDLE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateIdle\n");
            break;
        }
        case EV_LIFTOFF:
        {
            transition(&ADA::stateShadowMode);
            break;
        }
        case EV_TC_SET_DPL_ALTITUDE:
        {
            const DeploymentAltitudeEvent& dpl_ev =
                static_cast<const DeploymentAltitudeEvent&>(ev);
            dpl_target_altitude = dpl_ev.dplAltitude;
            break;
        }
        case EV_TC_RESET_CALIBRATION:
        {
            calibrationData.stats.reset();
            transition(&ADA::stateCalibrating);
            break;
        }
        default:
        {
            TRACE("ADA stateIdle: %d event not handled", ev.sig);
            break;
        }
    }
}

/**
 * \brief Shadow mode state:  ADA is running and logging apogees detected, but
 * is not generating events
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of vertical speed sign.
 * The exiting transition to the active state is triggered by a timeout event.
 */
void ADA::stateShadowMode(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateShadowMode\n");
            cal_delayed_event_id = ev_broker->postDelayed(EV_TIMEOUT_SHADOW_MODE, TOPIC_ADA, TIMEOUT_ADA_SHADOW_MODE);
            logStatus(ADAState::SHADOW_MODE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateShadowMode\n");
            ev_broker->removeDelayed(cal_delayed_event_id);
            break;
        }
        case EV_TIMEOUT_SHADOW_MODE:
        {
            transition(&ADA::stateActive);
            break;
        }
        default:
        {
            TRACE("ADA stateShadowMode: %d event not handled", ev.sig);
            break;
        }
    }
}

/**
 * \brief Active state:  ADA is running and it generates an event whe apogee is
 * detected
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of vertical speed sign.
 * The exiting transition to the descent state is triggered by the apogee
 * reached event (NOT self generated!)
 */
void ADA::stateActive(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateActive\n");
            logStatus(ADAState::ACTIVE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateActive\n");
            break;
        }
        case EV_APOGEE:
        {
            transition(&ADA::stateFirstDescentPhase);
            break;
        }
        default:
        {
            TRACE("ADA stateActive: %d event not handled", ev.sig);
            break;
        }
    }
}

/**
 * \brief First descent phase state:  ADA is running and it generates an event
 * when a set altitude is reached
 *
 * In this state a call to update() will trigger a one step update of the kalman
 * filter followed by a check of the altitude.
 * The exiting transition to the stop state is triggered by the parachute
 * deployment altitude reached event (NOT self generated!)
 */
void ADA::stateFirstDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateFirstDescentPhase\n");
            logState(ADAState::FIRST_DESCENT_PHASE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateFirstDescentPhase\n");
            break;
        }
        case EV_DPL_ALTITUDE:
        {
            transition(&ADA::stateEnd);
            break;
        }
        default:
        {
            TRACE("ADA stateFirstDescentPhase: %d event not handled", ev.sig);
            break;
        }
    }
}

/**
 * \brief End state:  ADA is stopped
 *
 * In this state a call to update() will have no effect.
 * This is the final state
 */
void ADA::stateEnd(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("ADA: Entering stateEnd\n");
            logState(ADAState::END);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateEnd\n");
            break;
        }
        default:
        {
            TRACE("ADA stateEnd: %d event not handled", ev.sig);
            break;
        }
    }
}

}  // namespace DeathStackBoard
