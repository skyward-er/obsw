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

#pragma once

#include <DeathStack/ADA/ADAStatus.h>
#include <events/FSM.h>

#include <DeathStack/Events.h>
#include <DeathStack/configs/ADA_config.h>
#include <kalman/Kalman.h>
#include "DeathStack/LogProxy/LogProxy.h"
#include "RogalloDTS/RogalloDTS.h"

#include <miosix.h>

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

class ADA : public FSM<ADA>
{

public:
    /** Constructor
     */
    ADA();

    /** Destructor
     */
    ~ADA() {}

    /** \brief Updates the algorithm with a new sample
     *
     * It's critical that this method is called at regualar intervals during the
     * flight. Call frequency is defined in ADA_config.h The behavior of this
     * function changes depending on the ADA state
     *
     * @param pressure The pressure sample in pascals
     */
    void updateBaro(float pressure);
    void updateGPS(double lat, double lon, bool hasFix);

    /**
     * ADA status
     * @returns A struct containing the time stamp, the ADA FSM state and
     * several flags
     */
    ADAStatus getStatus() { return status; }

    /**
     * Get the latest state estimated by the Kalman filter
     * @returns A struct containing three floats representing the three states
     */
    KalmanState getKalmanState() { return last_kalman_state; }

    /**
     * Get the calibration parameters
     * @returns A struct containing average, number and variance of the
     * calibration samples
     */
    ADACalibrationData getCalibrationData() { return calibration_data; }

    const RogalloDTS& getRogalloDTS() const { return rogallo_dts; }

    /**
     * Sets the reference temperature to be used to calibrate the altimeter
     * @param ref_temp Reference temperature in degrees Celsisus
     */
    void setReferenceTemperature(float ref_temp);

    /**
     * Sets the reference altitude to be used to calibrate the altimeter
     * @param ref_alt Reference altitude in meters above mean sea level
     */
    void setReferenceAltitude(float ref_alt);

    /**
     * Sets the deployment altitude 
     * @param dpl_alt Deployment altitude in meters above GROUND level
     */
    void setDeploymentAltitude(float dpl_alt);

private:
    // FSM States
    void stateIdle(const Event& ev);
    void stateCalibrating(const Event& ev);
    void stateReady(const Event& ev);
    void stateShadowMode(const Event& ev);
    void stateActive(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateEnd(const Event& ev);

    /** Performs a Kalman state update
     * @param pressure The pressure sample in pascal
     */
    void updateFilter(float pressure);

    /** Checks if calibration is complete and if this is the case sends the
     * ADA_READY event.
     */
    void finalizeCalibration();

    /**
     * Calculates altitude and vertical speed based on the current kalman state.
     * Then logs the data and return the current altitude
     * @return Altitude MSL [m]
     *
     * @param p Pressure [Pa]
     * @param dp_dt  Variation of pressure [Pa / s]
     * @return Altitude msl [m]
     */
    float updateAltitude(float p, float dp_dt);

    /** Update and log ADA FSM state
     */
    void logStatus(ADAState state);

    /** Log the ADA FSM state without updating it
     */
    void logStatus();

    void resetCalibration();

    // Event id to store calibration timeout
    uint16_t shadow_delayed_event_id = 0;

    // References for pressure to altitude conversion
    ReferenceValues reference_values;
      float temperature_ref =
        DEFAULT_REFERENCE_TEMPERATURE;  // Reference temperature in K at
                                        // launchpad
    float altitude_ref =
        DEFAULT_REFERENCE_ALTITUDE;  // Reference altitude at launchpad

    // Pressure at mean sea level for altitude calculation, to be updated with
    // launch-day calibration
    float pressure_0    = DEFAULT_MSL_PRESSURE;
    float temperature_0 = DEFAULT_MSL_TEMPERATURE;

    // Filter object
    Kalman<3, 1> filter;

    // Number of consecutive samples in which the speed was negative
    unsigned int n_samples_going_down = 0;

    // Calibration variables
    ADACalibrationData calibration_data;
    Stats pressure_stats;
    FastMutex calib_mutex;  // Mutex for pressure_stats

    // ADA status: timestamp + state
    ADAStatus status;

    // Last kalman state
    KalmanState last_kalman_state;

    // Rogallo deployment and termination system
    RogalloDTS rogallo_dts;

    // Logger
    LoggerProxy& logger = *(LoggerProxy::getInstance());
};

}  // namespace DeathStackBoard