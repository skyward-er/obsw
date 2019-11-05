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
    ADAControllerStatus getStatus() { return status; }

private:
    /* --- FSM STATES --- */
    void stateIdle(const Event& ev);
    void stateCalibrating(const Event& ev);
    void stateReady(const Event& ev);
    void stateShadowMode(const Event& ev);
    void stateActive(const Event& ev);
    void stateFirstDescentPhase(const Event& ev);
    void stateEnd(const Event& ev);

    void finalizeCalibration();

    void logStatus(ADAState state);  // Update and log ADA FSM state
    void logStatus();  // Log the ADA FSM state without updating it

    void logData(KalmanState s, ADAData d);

    uint16_t shadow_delayed_event_id =
        0;  // Event id to store calibration timeout

    ADAControllerStatus status;  // ADA status: timestamp + state

    /* --- CALIBRATION --- */
    FastMutex calibrator_mutex;
    ADACalibrator calibrator;

    /* --- ALGORITHM --- */
    ADA ada;

    float deployment_altitude = DEFAULT_DEPLOYMENT_ALTITUDE;
    bool deployment_altitude_set = false;

    // Number of consecutive samples in which apogee is detected
    unsigned int n_samples_apogee_detected = 0;  

    // Number of consecutive samples in which dpl altitude is detected
    unsigned int n_samples_deployment_detected = 0;  

    /* --- LOGGER --- */
    LoggerService& logger = *(LoggerService::getInstance());  // Logger

    
};

}  // namespace DeathStackBoard