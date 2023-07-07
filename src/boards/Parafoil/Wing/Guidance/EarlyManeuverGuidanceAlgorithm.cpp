/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>

#include <Eigen/Core>

namespace Parafoil
{

EarlyManeuversGuidanceAlgorithm::EarlyManeuversGuidanceAlgorithm(
    const Eigen::Vector2f& EMC, const Eigen::Vector2f& M1,
    const Eigen::Vector2f& M2)
    : EMC(EMC), M1(M1), M2(M2){};

EarlyManeuversGuidanceAlgorithm::~EarlyManeuversGuidanceAlgorithm(){};

float EarlyManeuversGuidanceAlgorithm::calculateTargetAngle(
    const Eigen::Vector3f& position, const Eigen::Vector2f& target,
    Eigen::Vector2f& heading)
{
    float altitude = abs(position[2]);

    if (altitude <= 50)  // Altitude is low, head directly to target
    {
        heading[0] = target[0] - position[0];
        heading[1] = target[1] - position[1];
        return atan2(heading[0], heading[1]);
    }

    // Altitude is low enough to head to the second maneuver point
    if (altitude > 50 && altitude <= 150)
    {
        heading[0] = M2[0] - position[0];
        heading[1] = M2[1] - position[1];
        return atan2(heading[0], heading[1]);
    }

    // Altitude is medium hence head to the second maneuver point
    if (altitude > 150 && altitude <= 250)
    {
        heading[0] = M1[0] - position[0];
        heading[1] = M1[1] - position[1];
        return atan2(heading[0], heading[1]);
    }

    // Altitude is too high, head to the EMC point
    heading[0] = EMC[0] - position[0];
    heading[1] = EMC[1] - position[1];
    return atan2(EMC[0] - position[0], EMC[1] - position[1]);
}

}  // namespace Parafoil