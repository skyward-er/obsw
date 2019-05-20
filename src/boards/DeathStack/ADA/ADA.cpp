/* Copyright (c) 2018 Skyward Experimental Rocketry
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

#include <DeathStack/Events.h>
#include <DeathStack/Topics.h>
#include <utils/aero/AeroUtils.h>

#include "Debug.h"

using miosix::Lock;

namespace DeathStackBoard
{

/* --- LIFE CYCLE --- */
ADA::ADA()
    : FSM(&ADA::stateCalibrating),
      filter(A_INIT, C_INIT, V1_INIT, V2_INIT, P_INIT), rogallo_dts()
{
    // Subscribe to topics
    sEventBroker->subscribe(this, TOPIC_FLIGHT_EVENTS);
    sEventBroker->subscribe(this, TOPIC_TC);
    sEventBroker->subscribe(this, TOPIC_ADA);
}

void ADA::updateFilter(float pressure)
{
    MatrixBase<float, 1, 1> y{pressure};
    filter.update(y);

    last_kalman_state.x0 = filter.X(0, 0);
    last_kalman_state.x1 = filter.X(1, 0);
    last_kalman_state.x2 = filter.X(2, 0);

    logger.log(last_kalman_state);
}

/* --- INSTANCE METHODS --- */

void ADA::updateGPS(double lat, double lon, double z, bool hasFix)
{
    // Update gps regardless of the current state
    rogallo_dts.updateGPS(lat, lon, hasFix);

    if (hasFix)
    {
        // Update calibration
        if (status.state == ADAState::CALIBRATING)
        {
            Lock<FastMutex> l(calib_mutex);

            if (calibration_data.gps_altitude_calib.nSamples <
                CALIBRATION_GPS_N_SAMPLES)
            {
                gps_altitude_stats.add(z);
                calibration_data.gps_altitude_calib =
                    gps_altitude_stats.getStats();

                logger.log(calibration_data);
            }
            else
            {
                updateCalibration();
            }
        }
    }
}

void ADA::updateBaro(float pressure, float temperature)
{
    switch (status.state)
    {
        case ADAState::CALIBRATING:
        {
            Lock<FastMutex> l(calib_mutex);

            // Calibrating state: update calibration data if not enough values
            if (calibration_data.pressure_calib.nSamples <
                CALIBRATION_BARO_N_SAMPLES)
            {
                pressure_stats.add(pressure);
                temperature_stats.add(temperature);

                calibration_data.pressure_calib = pressure_stats.getStats();
                calibration_data.temperature_calib =
                    temperature_stats.getStats();

                // Log calibration data
                logger.log(calibration_data);
            }
            else
            {
                updateCalibration();
            }
            break;
        }

        case ADAState::IDLE:
        {
            // Don't use kalman pressure while we are on the ramp
            updateAltitude(pressure, 0);
            break;
        }

        case ADAState::SHADOW_MODE:
        {
            // Shadow mode state: update kalman, DO NOT send events
            updateFilter(pressure);
            updateAltitude(filter.X(0, 0), filter.X(1, 0));

            // Check if the vertical speed is negative
            if (filter.X(1, 0) < 0)
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

            // Check if the vertical speed is negative
            if (filter.X(1, 0) < 0)
            {

                sEventBroker->post({EV_ADA_APOGEE_DETECTED}, TOPIC_ADA);
                status.apogee_reached = true;

                // Log
                ApogeeDetected apogee{status.state, miosix::getTick()};
                logger.log(apogee);
            }
            break;
        }

        case ADAState::FIRST_DESCENT_PHASE:
        case ADAState::END:  // Update rogallo DTS even when ada has completed
                             // its job
        {
            // Descent state: send notifications for target altitude reached
            updateFilter(pressure);

            float altitude = updateAltitude(filter.X(0, 0), filter.X(1, 0));

            rogallo_dts.updateAltitude(altitude);
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

void ADA::updateCalibration()
{
    // Set calibration only if we have enough samples
    if (calibration_data.gps_altitude_calib.nSamples >=
            CALIBRATION_GPS_N_SAMPLES &&
        calibration_data.pressure_calib.nSamples >= CALIBRATION_BARO_N_SAMPLES)
    {
        // Set reference to the calibration average
        pressure_ref    = calibration_data.pressure_calib.mean;
        temperature_ref = calibration_data.temperature_calib.mean;

        // TODO: Calibration sanity check

        // Calculat MSL values for altitude calculation
        pressure_0 =
            aeroutils::mslPressure(pressure_ref, temperature_ref,
                                   calibration_data.gps_altitude_calib.mean);

        temperature_0 = aeroutils::mslTemperature(
            temperature_ref, calibration_data.gps_altitude_calib.mean);

        // Initialize kalman filter
        filter.X(0, 0) = pressure_ref;

        // Log reference values
        ReferenceValues rf;
        rf.msl_pressure    = pressure_0;
        rf.msl_temperature = temperature_0;
        rf.ref_altitude    = calibration_data.gps_altitude_calib.mean;
        rf.ref_pressure    = pressure_ref;
        rf.ref_temperature = temperature_ref;

        logger.log(rf);

        // Notify that we are ready
        sEventBroker->post({EV_ADA_READY}, TOPIC_ADA);
    }
}

void ADA::logStatus(ADAState state)
{
    status.timestamp = miosix::getTick();
    status.state     = state;

    logger.log(status);
}

void ADA::logStatus()
{
    status.timestamp = miosix::getTick();

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
            const ConfigurationEvent& dpl_ev =
                static_cast<const ConfigurationEvent&>(ev);

            rogallo_dts.setDeploymentAltitudeAgl(dpl_ev.config);
            status.dpl_altitude_set = true;
            logStatus();
            break;
        }
        case EV_TC_CALIBRATE_ADA:
        {
            resetCalibration();
            break;
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
            const ConfigurationEvent& dpl_ev =
                static_cast<const ConfigurationEvent&>(ev);

            rogallo_dts.setDeploymentAltitudeAgl(dpl_ev.config);
            status.dpl_altitude_set = true;
            logStatus();
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
            shadow_delayed_event_id = sEventBroker->postDelayed(
                {EV_TIMEOUT_SHADOW_MODE}, TOPIC_ADA, TIMEOUT_ADA_SHADOW_MODE);
            logStatus(ADAState::SHADOW_MODE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateShadowMode\n");
            sEventBroker->removeDelayed(shadow_delayed_event_id);
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
        case EV_ADA_APOGEE_DETECTED:  // TODO: Remove this
#warning "Remove EV_ADA_APOGEE_DETECTED in ADA stateActive"
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
            logStatus(ADAState::FIRST_DESCENT_PHASE);
            break;
        }
        case EV_EXIT:
        {
            TRACE("ADA: Exiting stateFirstDescentPhase\n");
            break;
        }
        case EV_DPL_ALTITUDE:
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
            logStatus(ADAState::END);
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

void ADA::resetCalibration()
{
    Lock<FastMutex> l(calib_mutex);

    pressure_stats.reset();
    temperature_stats.reset();
    gps_altitude_stats.reset();

    calibration_data.pressure_calib     = pressure_stats.getStats();
    calibration_data.temperature_calib  = temperature_stats.getStats();
    calibration_data.gps_altitude_calib = gps_altitude_stats.getStats();
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

}  // namespace DeathStackBoard
