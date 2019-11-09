/* Copyright (c) 2019 Skyward Experimental Rocketry
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

#include <tests/mock_sensors/test-mock-data.h>

namespace DeathStackBoard
{
class MockGPS
{
public:
    MockGPS() : lat(lat_inside), lon(lon_inside) {}

    bool updateCoordinates()
    {
        // if (inside_lha)
        // {
        //     lat = lat_inside;
        //     lon = lon_inside;
        // }
        // else
        // {
        //     lat = lat_outside;
        //     lon = lon_outside;
        // }
        if (i < DATA_SIZE)
        {
            lat = SIMULATED_LAT[i];
            lon = SIMULATED_LON[i];
            i++;
        }
        return true;
    }

    bool inside_lha = true;

    double lat, lon;
    bool fix = true;
private:
    // Set of coordinates inside the Launch Hazard Area
    // const double lat_inside = 41.807487124105;
    // const double lon_inside = 14.0551665469291;

    // Set of coordinates outside the Launch Hazard Area
    // const double lat_outside = 41.840794;
    // const double lon_outside = 14.003920;

    unsigned int i = 0;
};
}  // namespace DeathStackBoard
