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

#include "elevation_map.h"

#include <cmath>
#include <cstddef>

namespace elevationmap
{

int getElevation(float lat, float lon)
{
    if (lat < SOUTH || lat >= NORTH || lon < WEST || lon >= EAST)
    {
        return INVALID_ELEVATION;
    }

    int x = (int)round((lon - WEST) * (RESOLUTION - 1.0) / LON_DELTA);
    int y = (int)round((lat - SOUTH) * (RESOLUTION - 1.0) / LAT_DELTA);

    // Check bounds
    if (x < 0 || x >= RESOLUTION || y < 0 || y >= RESOLUTION)
    {
        return INVALID_ELEVATION;
    }

    int index = y * RESOLUTION + x;
    return elevations[index];
}
}  // namespace elevationmap