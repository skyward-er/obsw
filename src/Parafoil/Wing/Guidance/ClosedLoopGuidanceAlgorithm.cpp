/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Radu Raul, Davide Basso
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

#include <Parafoil/Wing/Guidance/ClosedLoopGuidanceAlgorithm.h>
#include <math.h>

using namespace Boardcore::Units::Angle;

namespace Parafoil
{

Radian ClosedLoopGuidanceAlgorithm::calculateTargetAngle(
    const Eigen::Vector3f& currentPositionNED, Eigen::Vector2f& heading)
{
    heading[0] = targetNED[0] - currentPositionNED[0];
    heading[1] = targetNED[1] - currentPositionNED[1];
    return Radian{atan2(heading[1], heading[0])};
}

void ClosedLoopGuidanceAlgorithm::setPoints(Eigen::Vector2f targetNED)
{
    this->targetNED = targetNED;
}

}  // namespace Parafoil
