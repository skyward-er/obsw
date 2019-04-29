#include "ElevationMap.h"

#include <cmath>
#include <cstddef>

namespace elevationmap
{

int getElevation(double lat, double lon)
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
} // namespace elevationmap