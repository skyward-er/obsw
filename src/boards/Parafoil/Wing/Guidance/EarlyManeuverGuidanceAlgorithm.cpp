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

#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>

#include <Eigen/Core>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Parafoil
{

EarlyManeuversGuidanceAlgorithm::EarlyManeuversGuidanceAlgorithm()
    : activeTarget(Target::EMC), targetAltitudeConfidence(0),
      m2AltitudeConfidence(0), m1AltitudeConfidence(0),
      emcAltitudeConfidence(0){};

EarlyManeuversGuidanceAlgorithm::~EarlyManeuversGuidanceAlgorithm(){};

float EarlyManeuversGuidanceAlgorithm::calculateTargetAngle(
    const Eigen::Vector3f& position, const Eigen::Vector2f& target,
    Eigen::Vector2f& heading)
{
    using namespace Boardcore;

    // Eigen::Vector2f EMC =
    //     ModuleManager::getInstance().get<WingController>()->getEMCPosition();

    // Eigen::Vector2f M1 =
    //     ModuleManager::getInstance().get<WingController>()->getM1Position();

    // Eigen::Vector2f M2 =
    //     ModuleManager::getInstance().get<WingController>()->getM2Position();

    float altitude = abs(position[2]);

    computeActiveTarget(altitude);

    switch (activeTarget)
    {
        case Target::EMC:
            heading[0] = EMC[0] - position[0];
            heading[1] = EMC[1] - position[1];
            break;
        case Target::M1:
            heading[0] = M1[0] - position[0];
            heading[1] = M1[1] - position[1];
            break;
        case Target::M2:
            heading[0] = M2[0] - position[0];
            heading[1] = M2[1] - position[1];
            break;
        case Target::FINAL:
            heading[0] = target[0] - position[0];
            heading[1] = target[1] - position[1];
            break;
    }

    return atan2(heading[1], heading[0]);
}

void EarlyManeuversGuidanceAlgorithm::computeActiveTarget(float altitude)
{
    if (altitude <= 50)  // Altitude is low, head directly to target
    {
        targetAltitudeConfidence++;
    }
    else if (altitude <= 150)  // Altitude is almost okay, go to M2
    {
        m2AltitudeConfidence++;
    }
    else if (altitude <= 250)  // Altitude is high, go to M1
    {
        m1AltitudeConfidence++;
    }
    else
    {
        emcAltitudeConfidence++;  // Altitude is too high, head to the emc
    }

    switch (activeTarget)
    {
        case Target::EMC:
            if (m2AltitudeConfidence >= 15)
            {
                activeTarget          = Target::M2;
                emcAltitudeConfidence = 0;
            }
            if (m1AltitudeConfidence >= 15)
            {
                activeTarget          = Target::M1;
                emcAltitudeConfidence = 0;
            }
            if (targetAltitudeConfidence >= 15)
            {
                activeTarget          = Target::FINAL;
                emcAltitudeConfidence = 0;
            }
            break;
        case Target::M1:
            if (emcAltitudeConfidence >= 15)
            {
                activeTarget         = Target::EMC;
                m1AltitudeConfidence = 0;
            }
            if (m2AltitudeConfidence >= 15)
            {
                activeTarget         = Target::M2;
                m1AltitudeConfidence = 0;
            }
            if (targetAltitudeConfidence >= 15)
            {
                activeTarget         = Target::FINAL;
                m1AltitudeConfidence = 0;
            }
            break;
        case Target::M2:
            if (emcAltitudeConfidence >= 15)
            {
                activeTarget         = Target::EMC;
                m2AltitudeConfidence = 0;
            }
            if (m1AltitudeConfidence >= 15)
            {
                activeTarget         = Target::M1;
                m2AltitudeConfidence = 0;
            }
            if (targetAltitudeConfidence >= 15)
            {
                activeTarget         = Target::FINAL;
                m2AltitudeConfidence = 0;
            }
            break;
        case Target::FINAL:
            if (emcAltitudeConfidence >= 15)
            {
                activeTarget             = Target::EMC;
                targetAltitudeConfidence = 0;
            }
            if (m2AltitudeConfidence >= 15)
            {
                activeTarget             = Target::M2;
                targetAltitudeConfidence = 0;
            }
            if (m1AltitudeConfidence >= 15)
            {
                activeTarget             = Target::M1;
                targetAltitudeConfidence = 0;
            }
    }
}

}  // namespace Parafoil
