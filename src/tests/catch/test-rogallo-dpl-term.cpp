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
#include <utils/catch.hpp>
#include "DeathStack/ADA/RDTS/ElevationMap.h"
#include "DeathStack/ADA/RDTS/LHCircles.h"
#include "DeathStack/ADA/RDTS/generated/elevation_map_test_data.h"
#include "DeathStack/ADA/RDTS/generated/lh_circles_test_data.h"

TEST_CASE("Test elevation function")
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
}

TEST_CASE("Test Launch Hazard Circles")
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
    }
}
