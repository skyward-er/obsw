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

#include <DeathStack/configs/ADA_config.h>
#include <kalman/Kalman.h>
#include "DeathStack/LogProxy/LogProxy.h"

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
     * It's critical that this method is called at regualar intervals during the flight. Call frequency is defined in ADA_config.h
     * The behavior of this function changes depending on the ADA state
     * \param height The altitude sample in meters
     */
    void update(float altitude);

    /**
     * \brief ADA status
     * \returns A struct containing the time stamp and the ADA state
     */
    ADAStatus getStatus() { return status; }

    /**
     * \brief Get the latest state estimated by the Kalman filter
     * \returns A struct containing three floats representing the three states
     */
    KalmanState getKalmanState() { return last_kalman_state; }

    /**
     * \brief Get the calibration parameters
     * \returns A struct containing average, number and variance of the calibration samples
     */
    ADACalibrationData getCalibrationData() { return calibrationData; }

    /**
     * \brief Get the set parachute deployment altitude
     * \returns The altitude at wich the parachute is deployed
     */
    uint16_t getTargetDeploymentAltitude() { return dpl_target_altitude; }

private:
    // FSM States
    void stateCalibrating(const Event& ev);
    void stateIdle(const Event& ev);
    void stateShadowMode(const Event& ev);
    void stateActive(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateEnd(const Event& ev);

    /** \brief Performs a state update
     * \param altitude The altitude sample in meters
     */
    void updateFilter(float altitude);

    /** \brief Log ADA state
     */
    void logStatus(ADAState state);

    /** Set deployment altitude
     * \param altitude Deployment altitude in meters
     */
    void setTargetDPLAltitude(uint16_t altitude);

    // Event id to store calibration timeout
    uint16_t cal_delayed_event_id = 0; 

    // Filter object
    Kalman<3,1> filter;  

    // Calibration variables
    ADACalibrationData calibrationData;
    Stats calibrationStats;

    // ADA status: timestamp + state
    ADAStatus status;         

    // Last kalman state
    KalmanState last_kalman_state;

    // Parachute deployment altitude in meters
    uint16_t dpl_target_altitude = DEFAULT_DPL_ALTITUDE;

    // Logger
    LoggerProxy& logger = *(LoggerProxy::getInstance());
};

}  // namespace DeathStackBoard