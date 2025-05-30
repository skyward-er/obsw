/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include "CASCA.h"

#include <Parafoil/StateMachines/NASController/NASController.h>

using namespace Boardcore;
using namespace Boardcore::Units::Length;
using namespace Boardcore::Units::Angle;
using namespace Boardcore::Units::Speed;

namespace Parafoil
{

CASCA::Trajectory CASCA::computeTrajectory()
{
    auto* nas     = getModule<NASController>();
    auto nasState = nas->getNasState();

    auto verticalVelocity = MeterPerSecond{-nasState.vd};
    auto inPlaneVelocity  = MeterPerSecond{
        sqrt(nasState.vn * nasState.vn + nasState.ve * nasState.ve)};

    Position currentPosition = {
        .x           = Meter{nasState.n},
        .y           = Meter{nasState.e},
        .orientation = Radian{atan2(nasState.ve, nasState.vn)},
        .altitude    = Meter{-nasState.d}};

    State state = {
        .verticalVelocity = verticalVelocity,
        .inPlaneVelocity  = inPlaneVelocity,
        .glideRatio       = verticalVelocity.value() / inPlaneVelocity.value()};

    BoundaryConditions boundary = {.currentPosition = currentPosition,
                                   .targetPosition  = target};

    Dubins dubins{state, boundary};
    // Compute the dubins trajectory to retrieve the stability margins
    auto dubinsParameters = dubins.computeManuevers(config.minTurnRadius);
    auto dubinsMargins    = dubins.computeMargins(dubinsParameters);

    if (dubinsMargins.eta < 0.f)
    {
        return computeTerminalTrajectory(boundary, state);
    }
    else if (dubinsMargins.eta < config.dubinsEtaThreshold)
    {
        return computeDubinsTrajectory(boundary, state);
    }
    else if (dubinsMargins.eta < config.minControlEtaThreshold)
    {
        return computeMinimumControlTrajectory(boundary, state);
    }
    else
    {
        LOG_WARN(logger, fmt::format("Dubins eta margin is too high: {}",
                                     dubinsMargins.eta));
        return computeMinimumControlTrajectory(boundary, state);
    }
}

void CASCA::setTargetPosition(Position newTarget) { target = newTarget; }

}  // namespace Parafoil
