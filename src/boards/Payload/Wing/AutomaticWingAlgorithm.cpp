/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Niccolò Betto
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

#include <Payload/Configs/WingConfig.h>
#include <Payload/Sensors/Sensors.h>
#include <Payload/StateMachines/NASController/NASController.h>
#include <Payload/StateMachines/WingController/WingController.h>
#include <Payload/WindEstimationScheme/WindEstimation.h>
#include <drivers/timer/TimestampTimer.h>
#include <math.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

using namespace Boardcore;
using namespace Eigen;
using namespace Payload::Config::Wing;

namespace Payload
{
AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki,
                                               ServosList servo1,
                                               ServosList servo2,
                                               GuidanceAlgorithm& guidance)
    : Super(servo1, servo2), guidance(guidance)
{
    controller = new PIController(Kp, Ki, WING_UPDATE_PERIOD / 1000.0f,
                                  PI_CONTROLLER_SATURATION_MIN_LIMIT,
                                  PI_CONTROLLER_SATURATION_MAX_LIMIT);
}

AutomaticWingAlgorithm::~AutomaticWingAlgorithm() { delete (controller); }

void AutomaticWingAlgorithm::step()
{
    if (getModule<Sensors>()->getUBXGPSLastSample().fix != 0)
    {
        // The PI calculated result
        float result = algorithmStep(
            getModule<NASController>()->getNasState(),
            getModule<WindEstimation>()->getWindEstimationScheme());

        // Actuate the result
        // To see how to interpret the PI output
        // https://www.geogebra.org/calculator/xrhwarpz
        // to system is
        /*    N ^
                |
                |
                ----> E
        */
        if (result > 0)
        {
            // Activate the servo2 and reset servo1
            getModule<Actuators>()->setServoAngle(servo1, 0);
            getModule<Actuators>()->setServoAngle(servo2, result);
        }
        else
        {
            // Activate the servo1 and reset servo2
            getModule<Actuators>()->setServoAngle(servo1, result * -1);
            getModule<Actuators>()->setServoAngle(servo2, 0);
        }

        // Log the servo positions
        {
            miosix::Lock<FastMutex> l(mutex);

            data.timestamp   = TimestampTimer::getTimestamp();
            data.servo1Angle = getModule<Actuators>()->getServoPosition(servo1);
            data.servo2Angle = getModule<Actuators>()->getServoPosition(servo2);
            SDlogger->log(data);
        }
    }
    else
    {
        // If we loose fix we set both servo at 0
        getModule<Actuators>()->setServoAngle(servo1, 0);
        getModule<Actuators>()->setServoAngle(servo2, 0);
    }
}

float AutomaticWingAlgorithm::algorithmStep(NASState state, Vector2f windNED)
{
    float result;
    // For some algorithms the third component is needed!
    Vector3f currentPosition(state.n, state.e, state.d);

    Vector2f heading;  // used for logging purposes

    float targetAngle = guidance.calculateTargetAngle(currentPosition, heading);

    // WES is currently unused
    Vector2f relativeVelocity(state.vn, state.ve);

    // Compute the angle of the current velocity
    float velocityAngle;

    // All angle are computed as angle from the north direction

    velocityAngle = atan2(relativeVelocity[1], relativeVelocity[0]);

    // Compute the angle difference
    float error = angleDiff(targetAngle, velocityAngle);

    // Call the PI with the just calculated error. The result is in RADIANS,
    // if positive we activate one servo, if negative the other
    result = controller->update(error);

    // Convert the result from radians back to degrees
    result = result * (180.f / Constants::PI);

    // Flip the servo orientation
    // result *= -1;
    //  TODO check if this is needed

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

float AutomaticWingAlgorithm::angleDiff(float a, float b)
{
    float diff = a - b;

    // Angle difference
    if (diff < -Constants::PI || Constants::PI < diff)
    {
        diff += Constants::PI;
        bool positiveInput = diff > 0;

        diff = diff - floor(diff / (2 * Constants::PI)) * (2 * Constants::PI);

        // diff = fmod(diff, 2 * Constants::PI);
        if (diff == 0 && positiveInput)
        {
            diff = 2 * Constants::PI;
        }

        diff -= Constants::PI;
    }

    return diff;
}

}  // namespace Payload
