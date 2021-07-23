/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <cmath>

namespace launchhazard
{
double DEG2RAD              = M_PI / 180.0;
double EARTH_RADIUS_SQUARED = 40560046730329;
/**
 * Launch Hazard Circle
 */
struct LHCircle
{
    double center_lat;
    double center_lon;
    double radius2;

    /**
     * Calculates the square of the distance between the provided coordinates
     * and the center of the circle.
     * Approximates the curvature of the Earth with a equirectangular
     * projection: Works well only for distances up to a few km.
     *
     * @param lat Latitude of the point
     * @param lon Longitude of the point
     * @return Square of the distance to the center in meters
     */
    double distance2(double lat, double lon)
    {
        lat = lat * DEG2RAD;
        lon = lon * DEG2RAD;

        double lat0 = center_lat * DEG2RAD;
        double lon0 = center_lon * DEG2RAD;

        double x = (lon - lon0) * cos((lat0 + lat) / 2);
        double y = (lat - lat0);

        return (pow(x, 2) + pow(y, 2)) * EARTH_RADIUS_SQUARED;
    }

    /**
     * Checks wheter the provided coordinates fall inside the launch hazard
     * circle.
     *
     * @param lat Latitude of the point
     * @param lon Longitude of the point
     * @return True if the provided coordinates are inside the circle
     */
    bool isInside(double lat, double lon)
    {
        return distance2(lat, lon) < radius2;
    }
};

// Load circles definitions
#include "generated/lh_circles_data.h"

}  // namespace launchhazard
