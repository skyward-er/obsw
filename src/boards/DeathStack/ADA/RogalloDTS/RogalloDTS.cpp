/**
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

#include "RogalloDTS.h"
#include <cstring>
#include "LHCircles.h"

namespace DeathStackBoard
{

RogalloDTS::RogalloDTS() { memset(inside_LHA, true, LHA_EGRESS_THRESHOLD); }

RogalloDTS::~RogalloDTS() {}

void RogalloDTS::setDeploymentAltitude(float dpl_altitude)
{
    deployment_altitude = dpl_altitude;
}

float RogalloDTS::getDeploymentAltitude()
{
    return deployment_altitude;
}

void RogalloDTS::updateGPS(float lat, float lon, bool has_fix)
{
    last_fix = has_fix;

    last_lat = lat;
    last_lon = lon;

    last_terran_elev = elevationmap::getElevation((double)lat, (double)lon);

    // We consider ourselves inside the LHA only if we have fix AND our
    // coordinates are effectively inside
    inside_LHA[inside_lha_ptr++] = isInsideLHA(lat, lon) && has_fix;

    if (inside_lha_ptr >= LHA_EGRESS_THRESHOLD)
    {
        inside_lha_ptr = 0;
    }

    update();
}

void RogalloDTS::updateAltitude(float altitude_msl)
{
    last_altitude_msl = altitude_msl;
    update();
}

void RogalloDTS::update()
{
    float altitude_agl = last_altitude_msl - last_terran_elev;
    // Deploy the rogallo wing
    if (!deployed && !isEgressing())
    {
        if(altitude_agl < deployment_altitude)
        {
            deployed = true;
            // TODO: Deploy rogallo
        }
    }
    
    // Cut the rogallo wing
    if (deployed && isEgressing() && !terminated)
    {
        terminated = true;
        // TODO: terminate rogallo
    }
}

bool RogalloDTS::isInsideLHA(float lat, float lon)
{
    using namespace launchhazard;
    for (int i = 0; i < NUM_CIRCLES; i++)
    {
        if (circles[i].isInside((double)lat, (double)lon))
        {
            return true;
        }
    }

    return false;
}

bool RogalloDTS::isEgressing()
{
    for (int i = 0; i < LHA_EGRESS_THRESHOLD; i++)
    {
        if (inside_LHA[i])
        {
            return false;
        }
    }

    return true;
}

}  // namespace DeathStackBoard