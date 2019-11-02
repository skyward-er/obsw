/* Copyright (c) 2018,2019 Skyward Experimental Rocketry
 * Authors: Luca Mozzarelli, Luca Erbetta
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

#include <DeathStack/ADA/ADAController.h>
#include <events/EventBroker.h>
#include <utils/aero/AeroUtils.h>
#include "DeathStack/System/StackLogger.h"
#include "Debug.h"

using miosix::Lock;

namespace DeathStackBoard
{

/* --- LIFE CYCLE --- */

ADAController::ADAController()
    : FSM(&ADAController::stateIdle, 4096, 2), ada(ReferenceValues{})
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_ADA);

    status.state = ADAState::IDLE;
}

/* --- SENSOR UPDATE METHODS --- */

void ADAController::updateGPS(double lat, double lon, bool hasFix)
{
    // Update gps regardless of the current state
    ada.updateGPS(lat, lon, hasFix);
}

void ADAController::updateBaro(float pressure)
{
    ADAState state = status.state;

    switch (state)
    {
        case ADAState::IDLE:
        {
            break;
        }
        case ADAState::CALIBRATING:
        {
            bool end_calib = false;
            {
                Lock<FastMutex> l(calibrator_mutex);

                // Add samples to the calibration
                calibrator.addBaroSample(pressure);

                // Save the state of calibration to release mutex
                end_calib = calibrator.calibIsComplete();
            }

            if (end_calib)
            {
                // If samples are enough and dpl altitude has been set init ada
                finalizeCalibration();
            }
            break;
        }

        case ADAState::READY:
        {
            // Log the altitude & vertical speed but don't use kalman pressure
            // while we are on the ramp
            ADAData d;
            d.timestamp = miosix::getTick();

            d.msl_altitude = ada.pressureToAltitude(pressure);
            d.vert_speed   = 0;

            d.acc_msl_altitude = 0;
            d.acc_vert_speed   = 0;

            ADA::AltitudeDPL ad   = ada.altitudeMSLtoDPL(d.msl_altitude);
            d.dpl_altitude        = ad.altitude;
            d.is_dpl_altitude_agl = ad.is_agl;

            logger.log(d);
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            ada.updateBaro(pressure);

            // Check if the vertical speed smaller than the target apogee speed
            if (ada.getVerticalSpeed() < APOGEE_VERTICAL_SPEED_TARGET)
            {
                // Log
                ApogeeDetected apogee{status.state, miosix::getTick()};
                logger.log(apogee);
            }

            logData(ada.getKalmanState(), ada.getADAData());

            break;
        }

        case ADAState::ACTIVE:
        {
            ada.updateBaro(pressure);
            // Check if we reached apogee
            if (ada.getVerticalSpeed() < APOGEE_VERTICAL_SPEED_TARGET)
            {
                n_samples_going_down = n_samples_going_down + 1;
                if (n_samples_going_down >= APOGEE_N_SAMPLES)
                {
                    // Active state send notifications for apogee
                    sEventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                    status.apogee_reached = true;
                }

                // Log
                ApogeeDetected apogee{status.state, miosix::getTick()};
                logger.log(apogee);
            }
            else if (n_samples_going_down != 0)
            {
                n_samples_going_down = 0;
            }

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:
        {
            // Descent state: send notifications for target altitude reached
            ada.updateBaro(pressure);

            if (ada.getAltitudeForDeployment().altitude <= deployment_altitude)
            {
                // TODO: DEPLOY!
            }

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }
        case ADAState::END:
        {
            // Continue updating the filter for logging & telemetry purposes
            ada.updateBaro(pressure);

            logData(ada.getKalmanState(), ada.getADAData());
            break;
        }
        case ADAState::UNDEFINED:
        {
            TRACE("[ADA] Update Baro: Undefined state value \n");
            break;
        }

        default:
        {
            TRACE("[ADA] Update Baro: Unexpected state value \n");
            break;
        }
    }
}

void ADAController::updateAcc(float ax)
{
    if (status.state == ADAState::SHADOW_MODE ||
        status.state == ADAState::ACTIVE ||
        status.state == ADAState::FIRST_DESCENT_PHASE ||
        status.state == ADAState::END)
    {
        ada.updateAcc(ax);
    }
}

/* --- TC --- */
void ADAController::setReferenceTemperature(float ref_temp)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        {
            Lock<FastMutex> l(calibrator_mutex);
            calibrator.setReferenceTemperature(ref_temp);
        }

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

void ADAController::setReferenceAltitude(float ref_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        {
            Lock<FastMutex> l(calibrator_mutex);
            calibrator.setReferenceAltitude(ref_alt);
        }

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

void ADAController::setDeploymentAltitude(float dpl_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        deployment_altitude     = dpl_alt;
        deployment_altitude_set = true;

        logger.log(TargetDeploymentAltitude{dpl_alt});

        TRACE("[ADA] Deployment altitude set to %.3f m\n", dpl_alt);

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

/* --- CALIBRATION --- */
void ADAController::finalizeCalibration()
{
    Lock<FastMutex> l(calibrator_mutex);

    if (calibrator.calibIsComplete() && deployment_altitude_set)
    {
        // If samples are enough and dpl altitude has been set init ada
        ada = ADA{calibrator.getReferenceValues()};

        // ADA READY!
        sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);

        logger.log(calibrator.getReferenceValues());
        logger.log(ada.getKalmanState());
    }
}

void ADAController::resetCalibration()
{
    Lock<FastMutex> l(calibrator_mutex);
    calibrator.resetBaro();
}

/* --- STATES --- */
/**
 * \brief Idle state: the ADA waits for a command to start calibration. This is
 * the initial state.
 */
void ADAController::stateIdle(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[ADA] Entering stateIdle\n");
            logStatus(ADAState::IDLE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateIdle\n");
            break;
        }
        case EV_CALIBRATE_ADA:
        {
            transition(&ADAController::stateCalibrating);
            break;
        }
        default:
        {
            // TRACE("[ADA] stateIdle: %d event not handled\n", ev.sig);
            break;
        }
    }
}

/**
 * \brief Calibrating state: the ADA calibrates the initial Kalman state.
 *
 * In this state a call to update() will result in a altitude sample being added
 * to the average.
 * The exiting transition to the idle state is triggered at the first sample
 * update after having set the deployment altitude and having reached the
 * minimum number of calibration samples.
 */
void ADAController::stateCalibrating(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[ADA] Entering stateCalibrating\n");
            logStatus(ADAState::CALIBRATING);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateCalibrating\n");
            break;
        }
        case EV_ADA_READY:
        {
            transition(&ADAController::stateReady);
            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            resetCalibration();
            break;
        }
        default:
        {
            // TRACE("ADA stateCalibrating: %d event not handled\n", ev.sig);
            break;
        }
    }
}

/**
 * \brief Ready state:  ADA is ready and waiting for liftoff
 *
 * In this state a call to update() will have no effect.
 * The exiting transition to the shadow mode state is triggered by the liftoff
 * event.
 */
void ADAController::stateReady(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(ADAState::READY);
            TRACE("[ADA] Entering stateReady\n");
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateReady\n");
            break;
        }
        case EV_LIFTOFF:
        {
            transition(&ADAController::stateShadowMode);
            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            resetCalibration();
            transition(&ADAController::stateCalibrating);
            break;
        }
        default:
        {
            // TRACE("ADA stateIdle: %d event not handled\n", ev.sig);
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
void ADAController::stateShadowMode(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            shadow_delayed_event_id =
                sEventBroker->postDelayed<TIMEOUT_ADA_SHADOW_MODE>(
                    {EV_TIMEOUT_SHADOW_MODE}, TOPIC_ADA);
            logStatus(ADAState::SHADOW_MODE);
            TRACE("[ADA] Entering stateShadowMode\n");
            break;
        }
        case EV_EXIT:
        {
            sEventBroker->removeDelayed(shadow_delayed_event_id);
            TRACE("[ADA] Exiting stateShadowMode\n");
            break;
        }
        case EV_TIMEOUT_SHADOW_MODE:
        {
            transition(&ADAController::stateActive);
            break;
        }
        default:
        {
            // TRACE("ADA stateShadowMode: %d event not handled\n", ev.sig);
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
void ADAController::stateActive(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            logStatus(ADAState::ACTIVE);
            TRACE("[ADA] Entering stateActive\n");
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateActive\n");
            break;
        }
        case EV_ADA_APOGEE_DETECTED:
        {
            transition(&ADAController::stateFirstDescentPhase);
            break;
        }
        default:
        {
            // TRACE("ADA stateActive: %d event not handled\n", ev.sig);
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
void ADAController::stateFirstDescentPhase(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[ADA] Entering stateFirstDescentPhase\n");
            logStatus(ADAState::FIRST_DESCENT_PHASE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateFirstDescentPhase\n");
            break;
        }
        case EV_ADA_DPL_ALT_DETECTED:
        {
            status.dpl_altitude_reached = true;
            logStatus();

            // Log
            DplAltitudeReached dpl_alt{miosix::getTick()};
            logger.log(dpl_alt);

            transition(&ADAController::stateEnd);
            break;
        }
        default:
        {
            // TRACE("ADA stateFirstDescentPhase: %d event not handled\n",
            // ev.sig);
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
void ADAController::stateEnd(const Event& ev)
{
    switch (ev.sig)
    {
        case EV_ENTRY:
        {
            TRACE("[ADA] Entering stateEnd\n");
            logStatus(ADAState::END);
            break;
        }
        case EV_EXIT:
        {
            TRACE("[ADA] Exiting stateEnd\n");
            break;
        }
        default:
        {
            // TRACE("ADA stateEnd: %d event not handled\n", ev.sig);
            break;
        }
    }
}

/* --- LOGGER --- */
void ADAController::logStatus(ADAState state)
{
    status.state = state;
    logStatus();
}

void ADAController::logStatus()
{
    status.timestamp = miosix::getTick();
    logger.log(status);

    StackLogger::getInstance()->updateStack(THID_ADA_FSM);
}

void ADAController::logData(KalmanState s, ADAData d)
{
    logger.log(s);
    logger.log(d);
}

}  // namespace DeathStackBoard
