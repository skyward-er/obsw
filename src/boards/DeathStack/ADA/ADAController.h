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
#include "ADA.h"
#include "ADACalibrator.h"

#include <DeathStack/configs/ADA_config.h>
#include <DeathStack/events/Events.h>
#include <kalman/Kalman.h>
#include "DeathStack/LoggerService/LoggerService.h"
#include "RogalloDTS/RogalloDTS.h"

#include <miosix.h>

using miosix::FastMutex;
using miosix::Lock;

namespace DeathStackBoard
{

class ADAController : public FSM<ADAController>
{

public:
    ADAController();
    ~ADAController() {}

    /* --- SENSOR UPDATE METHODS --- */
    /*
     * It's critical that this methods are called at regualar intervals during
     * the flight. Call frequency is defined in ADA_config.h The behavior of
     * this functions changes depending on the ADA state
     */
    void updateBaro(float pressure);
    void updateGPS(double lat, double lon, bool hasFix);
    void updateAcc(float ax);

    /* --- TC --- */
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

    const RogalloDTS& getRogalloDTS() const { return rogallo_dts; }

private:
    /* --- FSM STATES --- */
    void stateIdle(const Event& ev);
    void stateCalibrating(const Event& ev);
    void stateReady(const Event& ev);
    void stateShadowMode(const Event& ev);
    void stateActive(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateEnd(const Event& ev);

    uint16_t shadow_delayed_event_id =
        0;  // Event id to store calibration timeout

    ADAStatus status;  // ADA status: timestamp + state

    /* --- CALIBRATION --- */
    FastMutex calibrator_mutex;
    ADACalibrator calibrator;

    void finalizeCalibration();
    void resetCalibration();

    /* --- ALGORITHM --- */
    std::unique_ptr<ADA> ada;
    RogalloDTS rogallo_dts;  // Rogallo deployment and termination system

    unsigned int n_samples_going_down =
        0;  // Number of consecutive samples in which the vertical speed was
            // negative

    KalmanState last_kalman_state;  // Last kalman state

    /* --- LOGGER --- */
    LoggerService& logger = *(LoggerService::getInstance());  // Logger

    void logStatus(ADAState state);  // Update and log ADA FSM state
    void logStatus();  // Log the ADA FSM state without updating it

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
};

}  // namespace DeathStackBoard