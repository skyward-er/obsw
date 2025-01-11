/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Niccolò Betto, Davide Basso
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

#include "AutomaticWingAlgorithm.h"

#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/StateMachines/NASController/NASController.h>
#include <Parafoil/StateMachines/WingController/WingController.h>
#include <drivers/timer/TimestampTimer.h>
#include <math.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

using namespace Boardcore;
using namespace Eigen;
using namespace Parafoil::Config::Wing;
using namespace Units::Angle;

namespace Parafoil
{
AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki,
                                               ServosList servo1,
                                               ServosList servo2,
                                               GuidanceAlgorithm& guidance)
    : Super(servo1, servo2), guidance(guidance)
{
    // PIController needs the sample period in floating point seconds
    auto samplePeriod = 1.0f / UPDATE_RATE.value<Hertz>();

    controller = std::make_unique<PIController>(Kp, Ki, samplePeriod,
                                                PI::SATURATION_MIN_LIMIT,
                                                PI::SATURATION_MAX_LIMIT);
}

void AutomaticWingAlgorithm::step()
{
    if (getModule<Sensors>()->getUBXGPSLastSample().fix == 3)
    {
        // The PI calculated result
        auto result = algorithmStep(getModule<NASController>()->getNasState());

        // Actuate the result
        // To see how to interpret the PI output
        // https://www.geogebra.org/calculator/xrhwarpz
        //
        // Reference system
        // N ^
        //   |
        //   |
        //   ----> E
        if (result > 0_deg)
        {
            // Activate the servo2 and reset servo1
            getModule<Actuators>()->setServoAngle(servo1, 0_deg);
            getModule<Actuators>()->setServoAngle(servo2, result);
        }
        else
        {
            // Activate the servo1 and reset servo2
            getModule<Actuators>()->setServoAngle(servo1, -result);
            getModule<Actuators>()->setServoAngle(servo2, 0_deg);
        }

        // Log the servo positions
        {
            miosix::Lock<FastMutex> l(mutex);

            data.timestamp   = TimestampTimer::getTimestamp();
            data.servo1Angle = getModule<Actuators>()->getServoAngle(servo1);
            data.servo2Angle = getModule<Actuators>()->getServoAngle(servo2);
            SDlogger->log(data);
        }
    }
    else
    {
        // If we loose fix we set both servo at 0
        getModule<Actuators>()->setServoAngle(servo1, 0_deg);
        getModule<Actuators>()->setServoAngle(servo2, 0_deg);
    }
}

Degree AutomaticWingAlgorithm::algorithmStep(const NASState& state)
{
    // For some algorithms the third component is needed!
    Vector3f currentPosition(state.n, state.e, state.d);

    Vector2f heading;  // used for logging purposes

    auto targetAngle = guidance.calculateTargetAngle(currentPosition, heading);

    Vector2f relativeVelocity(state.vn, state.ve);

    // Compute the angle of the current velocity
    // All angle are computed as angle from the north direction
    auto velocityAngle =
        Radian(atan2(relativeVelocity[1], relativeVelocity[0]));

    // Compute the angle difference
    auto error = angleDiff(targetAngle, velocityAngle);

    // Call the PI with the just calculated error. The result is in RADIANS,
    // if positive we activate one servo, if negative the other
    // We also need to convert the result from radians back to degrees
    auto result = Degree(Radian(controller->update(error.value<Radian>())));

    // Logs the outputs
    {
        miosix::Lock<FastMutex> l(mutex);
        data.targetX       = heading[0];
        data.targetY       = heading[1];
        data.targetAngle   = targetAngle;
        data.velocityAngle = velocityAngle;
        data.error         = error;
        data.pidOutput     = result;
    }

    return result;
}

Radian AutomaticWingAlgorithm::angleDiff(Radian a, Radian b)
{
    auto diff = (a - b).value<Radian>();

    // Angle difference
    if (diff < -Constants::PI || Constants::PI < diff)
    {
        diff += Constants::PI;
        bool positiveInput = diff > 0;

        diff = diff - floor(diff / (2 * Constants::PI)) * (2 * Constants::PI);

        // diff = fmod(diff, 2 * Constants::PI);
        if (diff == 0 && positiveInput)
            diff = 2 * Constants::PI;

        diff -= Constants::PI;
    }

    return Radian{diff};
}

}  // namespace Parafoil
