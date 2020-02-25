/**
 * Rogallo Deployment and Termination System tests
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

#ifdef STANDALONE_CATCH1_TEST
#include "catch/catch-tests-entry.cpp"
#endif
#include <utils/testutils/catch.hpp>

#include <algorithm>

#include "ADA/RogalloDTS/ElevationMap.h"
#include "ADA/RogalloDTS/LHCircles.h"
#include "ADA/RogalloDTS/generated/tests/elevation_map_test_data.h"
#include "ADA/RogalloDTS/generated/tests/lh_circles_test_data.h"

#include "ADA/RogalloDTS/RogalloDTS.h"

#include <utils/EventCounter.h>
#include "events/Events.h"

using namespace DeathStackBoard;

TEST_CASE("[RogalloDTS] Test ground elevation function")
{
    using namespace elevationmap;

    for (size_t i = 0; i < test::TEST_DATA_SIZE; i++)
    {
        double lat = test::test_latitudes[i];
        double lon = test::test_longitudes[i];
        int elev   = test::test_elevations[i];

        if (getElevation(lat, lon) != elev)
        {
            CAPTURE(i);
            CAPTURE(lat);
            CAPTURE(lon);
            REQUIRE(getElevation(lat, lon) == elev);
        }
    }
    SUCCEED();
}

TEST_CASE("[RogalloDTS] Test Launch Hazard Circles")
{
    using namespace launchhazard;
    SECTION("Test distance function")
    {
        for (int i = 0; i < NUM_CIRCLES; i++)
        {
            LHCircle c             = circles[i];
            const double *dist_ptr = test::test_distances[i];

            for (size_t j = 0; j < test::TEST_DATA_SIZE; j++)
            {
                double lat  = test::test_latitudes[j];
                double lon  = test::test_longitudes[j];
                double dist = dist_ptr[j];

                // Max error of 0.1%
                Approx target_dist = Approx(dist).margin(0.001);

                if (c.distance2(lat, lon) != target_dist)
                {
                    CAPTURE(i);
                    CAPTURE(j);
                    CAPTURE(lat);
                    CAPTURE(lon);
                    REQUIRE(c.distance2(lat, lon) == target_dist);
                }
            }
        }
        SUCCEED();
    }

    SECTION("Test inside circle")
    {
        for (int i = 0; i < NUM_CIRCLES; i++)
        {
            LHCircle c          = circles[i];
            const bool *ins_ptr = test::test_inside[i];

            for (size_t j = 0; j < test::TEST_DATA_SIZE; j++)
            {
                double lat  = test::test_latitudes[j];
                double lon  = test::test_longitudes[j];
                bool inside = ins_ptr[j];

                if (c.isInside(lat, lon) != inside)
                {
                    CAPTURE(i);
                    CAPTURE(j);
                    CAPTURE(lat);
                    CAPTURE(lon);
                    REQUIRE(c.isInside(lat, lon) == inside);
                }
            }
        }
        SUCCEED();
    }
}

TEST_CASE("[Rogallo DTS] Test deployment altitude")
{
    EventCounter c{*sEventBroker};
    c.subscribe(TOPIC_ADA);

    RogalloDTS dts;

    SECTION("Getter & Setter")
    {
        dts.setDeploymentAltitudeAgl(1000);
        REQUIRE(dts.getDeploymentAltitudeAgl() == 1000);

        dts.setDeploymentAltitudeAgl(200);
        REQUIRE(dts.getDeploymentAltitudeAgl() == 200);
    }

    // Aremogna town is supposed to be outside the LHA
    float out_lat = 41.823484;
    float out_lon = 14.035092;

    // If there are no circles, any point is inside the LHA...
    float in_lat = out_lat;
    float in_lon = out_lon;

    // If there are, the center of a circle is for sure inside the LHA
    if (launchhazard::NUM_CIRCLES > 0)
    {
        in_lat = (float)launchhazard::circles[0].center_lat;
        in_lon = (float)launchhazard::circles[0].center_lon;
    }

    SECTION("Do not deploy if altitude is not updated")
    {
        dts.setDeploymentAltitudeAgl(100);
        // 10 samples inside the LHA
        for (int i = 0; i < 10; i++)
        {
            dts.updateGPS(in_lat, in_lon, true);
        }

        // No events received
        REQUIRE(c.getTotalCount() == 0);
    }

    SECTION("Do not deploy if deployment altitude is never set")
    {
        // 10 samples inside the LHA & low msl altitude received
        for (int i = 0; i < 10; i++)
        {
            dts.updateGPS(in_lat, in_lon, true);
            dts.updateAltitude(-1000);
        }

        // No events received
        REQUIRE(c.getTotalCount() == 0);
    }

    SECTION("Do deploy if one gps sample is good and we have altitude")
    {
        int gelev                  = elevationmap::getElevation(in_lat, in_lon);
        const int dpl_altitude_agl = 200;
        // Deploy 200 m above ground
        dts.setDeploymentAltitudeAgl(dpl_altitude_agl);

        // One good gps sample
        dts.updateGPS(in_lat, in_lon, true);

        // Too high
        dts.updateAltitude(gelev + dpl_altitude_agl + 200);
        dts.updateAltitude(gelev + dpl_altitude_agl + 150);
        dts.updateAltitude(gelev + dpl_altitude_agl + 100);
        dts.updateAltitude(gelev + dpl_altitude_agl + 50);
        dts.updateAltitude(gelev + dpl_altitude_agl + 1);

        // No events received yet
        REQUIRE(c.getTotalCount() == 0);

        dts.updateAltitude(gelev + dpl_altitude_agl);

        // Received deplyment event
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);

        dts.updateAltitude(gelev + dpl_altitude_agl - 1);
        dts.updateAltitude(gelev + dpl_altitude_agl - 50);
        dts.updateAltitude(gelev + dpl_altitude_agl - 100);

        // No more events for subsequent samples
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);
    }

    SECTION("Do not deploy until we have GPS fix")
    {
        int gelev                  = elevationmap::getElevation(in_lat, in_lon);
        const int dpl_altitude_agl = 200;
        // Deploy 200 m above ground
        dts.setDeploymentAltitudeAgl(dpl_altitude_agl);

        // No fix gps sample
        dts.updateGPS(in_lat, in_lon, false);

        // Too high
        dts.updateAltitude(gelev + dpl_altitude_agl + 200);
        // No events received yet
        REQUIRE(c.getTotalCount() == 0);

        // Good altitude but no fix
        dts.updateAltitude(gelev + dpl_altitude_agl - 100);
        dts.updateGPS(in_lat, in_lon, false);
        // No events received yet
        REQUIRE(c.getTotalCount() == 0);

        // Fix, finally
        dts.updateGPS(in_lat, in_lon, true);

        // Received deplyment event
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);

        dts.updateGPS(in_lat, in_lon, true);
        dts.updateAltitude(gelev + dpl_altitude_agl - 100);

        // No more events for subsequent samples
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);
    }

    SECTION("Do not deploy when we are outside the LHA")
    {
        int gelev = std::min(elevationmap::getElevation(out_lat, out_lon),
                             elevationmap::getElevation(in_lat, in_lon));

        const int dpl_altitude_agl = 200;

        // Deploy 200 m above ground
        dts.setDeploymentAltitudeAgl(dpl_altitude_agl);

        dts.updateGPS(out_lat, out_lon, true);

        // Too high
        dts.updateAltitude(gelev + dpl_altitude_agl + 500);
        // No events received yet
        REQUIRE(c.getTotalCount() == 0);

        // Good altitude but outside
        dts.updateAltitude(gelev + dpl_altitude_agl - 200);
        dts.updateGPS(out_lat, out_lon, true);

        // No events received yet
        REQUIRE(c.getTotalCount() == 0);

        // Go inside the LHA, finally
        dts.updateGPS(in_lat, in_lon, true);

        // Received deplyment event
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);

        dts.updateGPS(in_lat, in_lon, true);
        dts.updateAltitude(gelev + dpl_altitude_agl - 200);

        // No more events for subsequent samples
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);
    }

    SECTION("Do not cut if we are outside for too few samples")
    {
        int gelev = std::min(elevationmap::getElevation(out_lat, out_lon),
                             elevationmap::getElevation(in_lat, in_lon));

        const int dpl_altitude_agl = 200;

        // Deploy 200 m above ground
        dts.setDeploymentAltitudeAgl(dpl_altitude_agl);
        dts.updateGPS(in_lat, in_lon, true);
        dts.updateAltitude(gelev + dpl_altitude_agl - 200);

        // Received deplyment event
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);

        // Outside LHA for LHA_EGRESS_THRESHOLD - 1 samples
        for (unsigned int i = 0; i < LHA_EGRESS_THRESHOLD - 1; i++)
        {
            dts.updateGPS(out_lat, out_lon, true);
        }

        // No abort event received
        REQUIRE(c.getTotalCount() == 1);
        // REQUIRE(c.getCount(EV_ABORT_ROGALLO) == 0);

        // In and back out
        dts.updateGPS(in_lat, in_lon, true);
        dts.updateGPS(out_lat, out_lon, true);

        // Still nothing
        REQUIRE(c.getTotalCount() == 1);
        // REQUIRE(c.getCount(EV_ABORT_ROGALLO) == 0);

        // Outside LHA for LHA_EGRESS_THRESHOLD samples
        for (unsigned int i = 0; i < LHA_EGRESS_THRESHOLD; i++)
        {
            dts.updateGPS(out_lat, out_lon, true);
        }

        // Now we should have aborted
        REQUIRE(c.getTotalCount() == 1);
        // REQUIRE(c.getCount(EV_ABORT_ROGALLO) == 1);
    }

    SECTION("Abort if no fix")
    {
        int gelev = std::min(elevationmap::getElevation(out_lat, out_lon),
                             elevationmap::getElevation(in_lat, in_lon));

        const int dpl_altitude_agl = 200;

        // Deploy 200 m above ground
        dts.setDeploymentAltitudeAgl(dpl_altitude_agl);
        dts.updateGPS(in_lat, in_lon, true);
        dts.updateAltitude(gelev + dpl_altitude_agl - 200);

        // Received deplyment event
        REQUIRE(c.getTotalCount() == 1);
        REQUIRE(c.getCount(EV_ADA_DPL_ALT_DETECTED) == 1);

        // Inside LHA but no fix for LHA_EGRESS_THRESHOLD - 1 samples
        for (unsigned int i = 0; i < LHA_EGRESS_THRESHOLD - 1; i++)
        {
            dts.updateGPS(in_lat, in_lon, false);
        }

        // No abort event received
        REQUIRE(c.getTotalCount() == 1);
        // REQUIRE(c.getCount(EV_ABORT_ROGALLO) == 0);

        // One last sample with no fix
        dts.updateGPS(in_lat, in_lon, false);

        // Now we should have aborted
        REQUIRE(c.getTotalCount() == 1);
        // REQUIRE(c.getCount(EV_ABORT_ROGALLO) == 1);
    }
}
