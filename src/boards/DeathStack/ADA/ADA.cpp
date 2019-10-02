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

#include <DeathStack/ADA/ADA.h>
#include <events/EventBroker.h>
#include <utils/aero/AeroUtils.h>
#include "DeathStack/System/StackLogger.h"
#include "Debug.h"

using miosix::Lock;

namespace DeathStackBoard
{

/* --- LIFE CYCLE --- */

ADA::ADA()
    : FSM(&ADA::stateIdle, 4096, 2),
      filter(A_INIT, C_INIT, V1_INIT, V2_INIT, P_INIT), rogallo_dts()
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_ADA);

    status.state = ADAState::IDLE;

    memset(&calibration_data, 0, sizeof(ADACalibrationData));
}

/* --- SENSOR UPDATE METHODS --- */

void ADA::updateGPS(double lat, double lon, bool hasFix)
{
    // Update gps regardless of the current state
    rogallo_dts.updateGPS(lat, lon, hasFix);
}

void ADA::updateBaro(float pressure)
{
    switch (status.state)
    {
        case ADAState::IDLE:
        {
            break;
        }
        case ADAState::CALIBRATING:
        {
            Lock<FastMutex> l(calib_mutex);

            // Calibrating state: update calibration data if not enough values
            if (calibration_data.pressure_calib.nSamples <
                CALIBRATION_BARO_N_SAMPLES)
            {
                pressure_stats.add(pressure);

                calibration_data.pressure_calib = pressure_stats.getStats();

                // Log calibration data
                logger.log(calibration_data);
            }
            else
            {
                finalizeCalibration();
            }
            break;
        }

        case ADAState::READY:
        {
            // Log the altitude & vertical speed but don't use kalman pressure
            // while we are on the ramp
            updateAltitude(pressure, 0);
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            updateFilter(pressure);
            updateAltitude(filter.X(0, 0), filter.X(1, 0));

            // Check if the vertical speed is negative (so when the derivative
            // of the pressure is > 0)
            if (filter.X(1, 0) > APOGEE_PRESSURE_VARIATION_TARGET)
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
            updateFilter(pressure);
            updateAltitude(filter.X(0, 0), filter.X(1, 0));

            // Check if we reached apogee
            if (filter.X(1, 0) > APOGEE_PRESSURE_VARIATION_TARGET)
            {
                n_samples_going_down = n_samples_going_down + 1;
                if (n_samples_going_down >= APOGEE_N_SAMPLES)
                {
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
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:

        {
            // Descent state: send notifications for target altitude reached
            updateFilter(pressure);
            float altitude = updateAltitude(filter.X(0, 0), filter.X(1, 0));

            rogallo_dts.updateAltitude(altitude);
            break;
        }
        case ADAState::END:
        {
            // Continue updating the filter for logging & telemetry purposes
            updateFilter(pressure);
            updateAltitude(filter.X(0, 0), filter.X(1, 0));
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

void ADA::setReferenceTemperature(float ref_temp)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        Lock<FastMutex> l(calib_mutex);

        // Sanity check: Obey to the laws of thermodynamics
        if (ref_temp + 273.15 > 0)
        {
            temperature_ref     = ref_temp + 273.15;  // Celsius to Kelvin
            status.ref_temp_set = true;

            TRACE("[ADA] Reference temperature set to %.3f K\n",
                  temperature_ref);

            reference_values.ref_temperature = ref_temp;
            logger.log(reference_values);
            logStatus();
        }

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

void ADA::setReferenceAltitude(float ref_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        Lock<FastMutex> l(calib_mutex);

        altitude_ref            = ref_alt;
        status.ref_altitude_set = true;

        TRACE("[ADA] Reference altitude set to %.3f m\n", altitude_ref);

        reference_values.ref_altitude = ref_alt;
        logger.log(reference_values);
        logStatus();

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

void ADA::setDeploymentAltitude(float dpl_alt)
{
    if (status.state == ADAState::CALIBRATING ||
        status.state == ADAState::READY)
    {
        Lock<FastMutex> l(calib_mutex);

        rogallo_dts.setDeploymentAltitudeAgl(dpl_alt);
        TargetDeploymentAltitude tda;
        tda.deployment_altitude = dpl_alt;
        status.dpl_altitude_set = true;

        logger.log(tda);
        logStatus();

        TRACE("[ADA] Deployment altitude set to %.3f m\n",
              tda.deployment_altitude);

        if (status.state == ADAState::READY)
        {
            // Update the calibration parameters if a calibration has already
            // been completed
            finalizeCalibration();
        }
    }
}

void ADA::finalizeCalibration()
{
    // Set calibration only if we have enough samples
    if (calibration_data.pressure_calib.nSamples >=
            CALIBRATION_BARO_N_SAMPLES &&
        status.dpl_altitude_set && status.ref_altitude_set &&
        status.ref_temp_set)
    {
        float pressure_ref = pressure_stats.getStats().mean;

        // Calculat MSL values for altitude calculation
        pressure_0 =
            aeroutils::mslPressure(pressure_ref, temperature_ref, altitude_ref);

        temperature_0 =
            aeroutils::mslTemperature(temperature_ref, altitude_ref);

        TRACE("[ADA] Finalized calibration. p_ref: %.3f, p0: %.3f, t0: %.3f\n",
              pressure_ref, pressure_0, temperature_0);

        // Initialize kalman filter
        filter.X(0, 0) = pressure_ref;
        filter.X(1, 0) = 0;
        filter.X(2, 0) = KALMAN_INITIAL_ACCELERATION;

        // Log updated filter & reference values
        reference_values.ref_pressure    = pressure_ref;
        reference_values.msl_pressure    = pressure_0;
        reference_values.msl_temperature = temperature_0;
        logger.log(reference_values);

        last_kalman_state.x0 = filter.X(0, 0);
        last_kalman_state.x1 = filter.X(1, 0);
        last_kalman_state.x2 = filter.X(2, 0);

        logger.log(last_kalman_state);

        // Notify that we are ready
        sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);
    }
}

void ADA::updateFilter(float pressure)
{
    MatrixBase<float, 1, 1> y{pressure};
    filter.update(y);

    last_kalman_state.x0        = filter.X(0, 0);
    last_kalman_state.x1        = filter.X(1, 0);
    last_kalman_state.x2        = filter.X(2, 0);
    last_kalman_state.timestamp = miosix::getTick();

    logger.log(last_kalman_state);
}

void ADA::resetCalibration()
{
    Lock<FastMutex> l(calib_mutex);

    pressure_stats.reset();

    calibration_data.pressure_calib = pressure_stats.getStats();

    logger.log(calibration_data);
}

float ADA::updateAltitude(float p, float dp_dt)
{
    if (p > 0)
    {
        KalmanAltitude kalt;
        kalt.altitude = aeroutils::relAltitude(p, pressure_0, temperature_0);

        kalt.vert_speed =
            aeroutils::verticalSpeed(p, dp_dt, pressure_0, temperature_0);

        kalt.timestamp = miosix::getTick();

        logger.log(kalt);

        return kalt.altitude;
    }
    return 0;
}

/* --- LOGGER HELPERS --- */
void ADA::logStatus(ADAState state)
{
    status.state = state;
    logStatus();
}

void ADA::logStatus()
{
    status.timestamp = miosix::getTick();
    logger.log(status);
    StackLogger::getInstance()->updateStack(THID_ADA_FSM);
}

/* --- STATES --- */
/**
 * \brief Idle state: the ADA waits for a command to start calibration. This is
 * the initial state.
 */
void ADA::stateIdle(const Event& ev)
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
            transition(&ADA::stateCalibrating);
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
void ADA::stateCalibrating(const Event& ev)
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
            transition(&ADA::stateReady);
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
void ADA::stateReady(const Event& ev)
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
            transition(&ADA::stateShadowMode);
            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            resetCalibration();
            transition(&ADA::stateCalibrating);
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
void ADA::stateShadowMode(const Event& ev)
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
            transition(&ADA::stateActive);
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
void ADA::stateActive(const Event& ev)
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
            transition(&ADA::stateFirstDescentPhase);
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
void ADA::stateFirstDescentPhase(const Event& ev)
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

            transition(&ADA::stateEnd);
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
void ADA::stateEnd(const Event& ev)
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

}  // namespace DeathStackBoard
