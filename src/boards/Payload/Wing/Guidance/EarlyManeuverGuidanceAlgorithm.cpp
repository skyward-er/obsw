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

#include <Payload/Configs/WingConfig.h>
#include <Payload/Wing/Guidance/EarlyManeuversGuidanceAlgorithm.h>

#include <Eigen/Core>

using namespace Payload::Config::Wing;

namespace Payload
{

EarlyManeuversGuidanceAlgorithm::EarlyManeuversGuidanceAlgorithm()
    : activeTarget(Target::EMC), targetAltitudeConfidence(0),
      m2AltitudeConfidence(0), m1AltitudeConfidence(0),
      emcAltitudeConfidence(0){};

EarlyManeuversGuidanceAlgorithm::~EarlyManeuversGuidanceAlgorithm(){};

float EarlyManeuversGuidanceAlgorithm::calculateTargetAngle(
    const Eigen::Vector3f& currentPositionNED, Eigen::Vector2f& heading)
{
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
        Guidance::TARGET_ALTITUDE_THRESHOLD)  // Altitude is low, head directly
                                              // to target
    {
        targetAltitudeConfidence++;
    }
    else if (altitude <= Guidance::M2_ALTITUDE_THRESHOLD)  // Altitude is almost
                                                           // okay, go to M2
    {
        m2AltitudeConfidence++;
    }
    else if (altitude <=
             Guidance::M1_ALTITUDE_THRESHOLD)  // Altitude is high, go to M1
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
            if (targetAltitudeConfidence >= Guidance::CONFIDENCE)
            {
                activeTarget          = Target::FINAL;
                emcAltitudeConfidence = 0;
            }
            else if (m2AltitudeConfidence >= Guidance::CONFIDENCE)
            {
                activeTarget          = Target::M2;
                emcAltitudeConfidence = 0;
            }
            else if (m1AltitudeConfidence >= Guidance::CONFIDENCE)
            {
                activeTarget          = Target::M1;
                emcAltitudeConfidence = 0;
            }
            break;

        case Target::M1:
            if (targetAltitudeConfidence >= Guidance::CONFIDENCE)
            {
                activeTarget         = Target::FINAL;
                m1AltitudeConfidence = 0;
            }
            else if (m2AltitudeConfidence >= Guidance::CONFIDENCE)
            {
                activeTarget         = Target::M2;
                m1AltitudeConfidence = 0;
            }
            break;

        case Target::M2:
            if (targetAltitudeConfidence >= Guidance::CONFIDENCE)
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
}

EarlyManeuversPoints EarlyManeuversGuidanceAlgorithm::getPoints()
{
    return EarlyManeuversPoints{
        .targetN = targetNED[0],
        .targetE = targetNED[1],
        .emcN    = EMC[0],
        .emcE    = EMC[1],
        .m1N     = M1[0],
        .m1E     = M1[1],
        .m2N     = M2[0],
        .m2E     = M2[1],
    };
}

Eigen::Vector2f EarlyManeuversGuidanceAlgorithm::getActiveTarget()
{
    switch (activeTarget)
    {
        case Target::EMC:
            return EMC;
        case Target::M1:
            return M1;
        case Target::M2:
            return M2;
        case Target::FINAL:
            return targetNED;
        default:
            return {0, 0};
    }
}

}  // namespace Payload
