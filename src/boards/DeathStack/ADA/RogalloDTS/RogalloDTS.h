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

namespace DeathStackBoard
{
constexpr unsigned int LHA_EGRESS_THRESHOLD = 10;
class RogalloDTS
{
public:
    RogalloDTS();
    ~RogalloDTS();

    void updateGPS(float lat, float lon, bool hasFix);
    void updateAltitude(float altitudeMSL);

    void setDeploymentAltitude(float dpl_altitude);
    float getDeploymentAltitude();
private:
    void update();

    static bool isInsideLHA(float lat, float lon);

    /**
     * Is the rocket egressing from the Launch Hazard Area?
     * An egression is detected when the rocket is outside the LHA for 
     * LHA_EGRESS_THRESHOLD consecutive GPS samples.
     * 
     * @return Wether the rocket is egressing the LHA
     */
    bool isEgressing();

    // States
    bool deployed = false;
    bool terminated = false;

    // Deployment altitude
    float deployment_altitude = -1000;
    
    // Last available data
    float last_lat = 0;
    float last_lon = 0;
    bool last_fix = false;

    float last_altitude_msl = 0;
    int last_terran_elev = elevationmap::INVALID_ELEVATION;

    // Array storing if the last N positions were inside the Launch Hazard
    // Area.
    bool inside_LHA[LHA_EGRESS_THRESHOLD];
    unsigned int inside_lha_ptr = 0;
};

}