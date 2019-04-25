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
#include <events/EventBroker.h>
#include <cstring>
#include "DeathStack/Events.h"
#include "LHCircles.h"

namespace DeathStackBoard
{

RogalloDTS::RogalloDTS() { memset(inside_LHA, false, LHA_EGRESS_THRESHOLD); }

RogalloDTS::~RogalloDTS() {}

void RogalloDTS::setDeploymentAltitudeAgl(float dpl_altitude)
{
    deployment_altitude_agl = dpl_altitude;
    deployment_altitude_set = true;
}

float RogalloDTS::getDeploymentAltitudeAgl() { return deployment_altitude_agl; }

void RogalloDTS::updateGPS(double lat, double lon, bool has_fix)
{
    has_gps_sample = true;
    last_fix       = has_fix;

    last_lat = lat;
    last_lon = lon;

    last_terran_elev = elevationmap::getElevation(lat, lon);

    // We consider ourselves inside the LHA only if we have fix AND our
    // coordinates are effectively inside
    inside_LHA[inside_lha_ptr++] = isInsideLHA(lat, lon) && has_fix;

    if (inside_lha_ptr >= LHA_EGRESS_THRESHOLD)
    {
        inside_lha_ptr = 0;
    }
  /*  TRACE("iLHA: ");
    for(unsigned int i = 0; i < LHA_EGRESS_THRESHOLD; i++)
    {
        TRACE("%d", inside_LHA[i] ? 1 : 0);
    }
    TRACE("\n");*/

    update();
}

void RogalloDTS::updateAltitude(float altitude_msl)
{
    last_altitude_msl   = altitude_msl;
    has_altitude_sample = true;
    update();
}

void RogalloDTS::update()
{
    // Do things only if we have at least 1 sample from each sensor and the dpl
    // altitude has been set.
    if (has_gps_sample && has_altitude_sample && deployment_altitude_set)
    {
        float altitude_agl = last_altitude_msl - last_terran_elev;
        // Deploy the rogallo wing
        if (!deployed && !isEgressing())
        {
            if (altitude_agl <= deployment_altitude_agl)
            {
                deployed = true;
                sEventBroker->post({EV_ADA_DPL_ALT_DETECTED}, TOPIC_ADA);
            }
        }

        // Cut the rogallo wing
        if (deployed && isEgressing() && !terminated)
        {
            terminated = true;
            sEventBroker->post({EV_ABORT_ROGALLO}, TOPIC_ADA);
        }
    }
}

bool RogalloDTS::isInsideLHA(double lat, double lon)
{
    using namespace launchhazard;

    // Assume we are always inside if no circles are specified
    if (NUM_CIRCLES == 0)
    {
        return true;
    }

    for (int i = 0; i < NUM_CIRCLES; i++)
    {
        if (circles[i].isInside(lat, lon))
        {
            
            return true;
        }
    }

    return false;
}

bool RogalloDTS::isEgressing()
{
    for (unsigned int i = 0; i < LHA_EGRESS_THRESHOLD; i++)
    {
        if (inside_LHA[i])
        {
            return false;
        }
    }

    return true;
}

}  // namespace DeathStackBoard