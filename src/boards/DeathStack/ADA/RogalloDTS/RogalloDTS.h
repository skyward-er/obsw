/**
 * Rogallo Deployment and Termination System
 * This class is used to determine when it is safe to deploy the Rogallo wing,
 * and to terminate the flight in case safety can no longer be assured.
 *
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#pragma once
#include "ElevationMap.h"
#include "DeathStack/configs/ADA_config.h"

namespace DeathStackBoard
{

/**
 * Rogallo Deployment and Termination System
 * This class receives samples from GPS and Kalman (altitude) in order to deploy
 * the rogallo only when it is safe. If the rogallo is deployed and safety
 * conditions are not met, this class sends an abort event.
 *
 * It is safe to deploy the rogallo if the following conditions apply:
 * - GPS has fix
 * - GPS says we are inside the LHA
 * - The altitude is below the target deployment altitude
 *
 * The rogallo is aborted if:
 * - The rogallo was deployed
 * - GPS says we are outside the LHA or has no fix for LHA_EGRESS_THRESHOLD
 *   consecutive samples
 */
class RogalloDTS
{
public:
    RogalloDTS();
    ~RogalloDTS();

    void updateGPS(double lat, double lon, bool hasFix);
    void updateAltitude(float altitudeMSL);

    void setDeploymentAltitudeAgl(float dpl_altitude);
    float getDeploymentAltitudeAgl();

private:
    void update();

    /**
     * Are the provided coordinates inside the Launch Hazard Area?
     * @param lat Latitude [degrees]
     * @param lon Longitude [degrees]
     * @return Whether the coordinates are inside the LHA
     */
    static bool isInsideLHA(double lat, double lon);

    /**
     * Is the rocket egressing from the Launch Hazard Area?
     * An egression is detected when the rocket is outside the LHA for
     * LHA_EGRESS_THRESHOLD consecutive GPS samples.
     *
     * @return Whether the rocket is egressing the LHA
     */
    bool isEgressing();

    // States
    bool deployed   = false;
    bool terminated = false;

    // Deployment altitude
    bool deployment_altitude_set  = false;
    float deployment_altitude_agl = -1000;

    // Last available data
    bool has_gps_sample = false;  // Do we have at least one sample?

    double last_lat       = 0;
    double last_lon       = 0;
    bool last_fix        = false;
    int last_terran_elev = elevationmap::INVALID_ELEVATION;

    bool has_altitude_sample = false;  // Do we have at least one sample?
    float last_altitude_msl  = 0;

    // Array storing if the last N positions were inside the Launch Hazard
    // Area. Initialized at false.
    bool inside_LHA[LHA_EGRESS_THRESHOLD];
    unsigned int inside_lha_ptr = 0;
};

}  // namespace DeathStackBoard