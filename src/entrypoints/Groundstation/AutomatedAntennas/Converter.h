#pragma once

#include <cmath>

struct NEDCoords
{
    float n = 0;
    float e = 0;
    float d = 0;
};

struct AntennaAngles
{
    float theta1 = 0;
    float theta2 = 0;
};

AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned)
{
    AntennaAngles angles;
    angles.theta1 = std::atan2(ned.n, ned.e);
    angles.theta2 = std::atan2(-ned.d, ned.n);
    return angles;
}