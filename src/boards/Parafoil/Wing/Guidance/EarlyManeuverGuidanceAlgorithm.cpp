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

#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>

#include <Eigen/Core>
#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Parafoil::WingConfig;

namespace Parafoil
{

EarlyManeuversGuidanceAlgorithm::EarlyManeuversGuidanceAlgorithm()
    : activeTarget(Target::EMC), targetAltitudeConfidence(0),
      m2AltitudeConfidence(0), m1AltitudeConfidence(0),
      emcAltitudeConfidence(0){};

EarlyManeuversGuidanceAlgorithm::~EarlyManeuversGuidanceAlgorithm(){};

float EarlyManeuversGuidanceAlgorithm::calculateTargetAngle(
    const Eigen::Vector3f& currentPositionNED, Eigen::Vector2f& heading)
{
    using namespace Boardcore;

    float altitude = abs(currentPositionNED[2]);

    computeActiveTarget(altitude);

    switch (activeTarget)
    {
        case Target::EMC:
            heading[0] = EMC[0] - currentPositionNED[0];
            heading[1] = EMC[1] - currentPositionNED[1];
            break;
        case Target::M1:
            heading[0] = M1[0] - currentPositionNED[0];
            heading[1] = M1[1] - currentPositionNED[1];
            break;
        case Target::M2:
            heading[0] = M2[0] - currentPositionNED[0];
            heading[1] = M2[1] - currentPositionNED[1];
            break;
        case Target::FINAL:
            heading[0] = targetNED[0] - currentPositionNED[0];
            heading[1] = targetNED[1] - currentPositionNED[1];
            break;
    }

    return atan2(heading[1], heading[0]);
}

void EarlyManeuversGuidanceAlgorithm::computeActiveTarget(float altitude)
{
    if (altitude <=
        GUIDANCE_TARGET_ALTITUDE_THRESHOLD)  // Altitude is low, head directly
                                             // to target
    {
        targetAltitudeConfidence++;
    }
    else if (altitude <= GUIDANCE_M2_ALTITUDE_THRESHOLD)  // Altitude is almost
                                                          // okay, go to M2
    {
        m2AltitudeConfidence++;
    }
    else if (altitude <=
             GUIDANCE_M1_ALTITUDE_THRESHOLD)  // Altitude is high, go to M1
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
            if (targetAltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget          = Target::FINAL;
                emcAltitudeConfidence = 0;
            }
            else if (m2AltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget          = Target::M2;
                emcAltitudeConfidence = 0;
            }
            else if (m1AltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget          = Target::M1;
                emcAltitudeConfidence = 0;
            }
            break;
        case Target::M1:
            if (targetAltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget         = Target::FINAL;
                m1AltitudeConfidence = 0;
            }
            else if (m2AltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget         = Target::M2;
                m1AltitudeConfidence = 0;
            }
            break;
        case Target::M2:
            if (targetAltitudeConfidence >= GUIDANCE_CONFIDENCE)
            {
                activeTarget         = Target::FINAL;
                m2AltitudeConfidence = 0;
            }
            break;
        case Target::FINAL:
            break;
    }
}

void EarlyManeuversGuidanceAlgorithm::setPoints(Eigen::Vector2f targetNED,
                                                Eigen::Vector2f EMC,
                                                Eigen::Vector2f M1,
                                                Eigen::Vector2f M2)
{
    this->targetNED = targetNED;
    this->EMC       = EMC;
    this->M1        = M1;
    this->M2        = M2;
    printf(
        "targetNED: [ % .3f, % .3f ]\n"
        " EMC: [ % .3f, % .3f ]\n"
        " M1: [ % .3f, % .3f ]\n"
        " M2: [ % .3f, % .3f ]\n",
        targetNED[0], targetNED[1], EMC[0], EMC[1], M1[0], M1[1], M2[0], M2[1]);
}

}  // namespace Parafoil
