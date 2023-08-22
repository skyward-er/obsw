/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Radu Raul
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
#include <Parafoil/WindEstimationScheme/WindEstimation.h>
#include <drivers/timer/TimestampTimer.h>
#include <math.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/Constants.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace Boardcore;
using namespace Eigen;
using namespace Parafoil::WingConfig;

namespace Parafoil
{
AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki,
                                               ServosList servo1,
                                               ServosList servo2,
                                               GuidanceAlgorithm& guidance)
    : WingAlgorithm(servo1, servo2), guidance(guidance)
{
    controller = new PIController(Kp, Ki, WING_UPDATE_PERIOD / 1000.0f,
                                  PI_CONTROLLER_SATURATION_MIN_LIMIT,
                                  PI_CONTROLLER_SATURATION_MAX_LIMIT);
}

AutomaticWingAlgorithm::~AutomaticWingAlgorithm() { delete (controller); }

void AutomaticWingAlgorithm::step()
{
    ModuleManager& modules = ModuleManager::getInstance();

    if (modules.get<Sensors>()->getUbxGpsLastSample().fix != 0)
    {
        // The PI calculated result
        float result = algorithmStep(
            modules.get<NASController>()->getNasState(),
            modules.get<WindEstimation>()->getWindEstimationScheme());

        // Actuate the result
        if (result > 0)
        {
            // Activate the servo1 and reset servo2
            modules.get<Actuators>()->setServoAngle(servo1, result);
            modules.get<Actuators>()->setServoAngle(servo2, 0);
        }
        else
        {
            // Activate the servo2 and reset servo1
            modules.get<Actuators>()->setServoAngle(servo1, 0);
            modules.get<Actuators>()->setServoAngle(servo2, result * -1);
        }

        // Log the servo positions
        {
            miosix::Lock<FastMutex> l(mutex);

            data.timestamp = TimestampTimer::getTimestamp();
            data.servo1Angle =
                modules.get<Actuators>()->getServoPosition(servo1);
            data.servo2Angle =
                modules.get<Actuators>()->getServoPosition(servo2);
            SDlogger->log(data);
        }
    }
    else
    {
        // If we loose fix we set both servo at 0
        modules.get<Actuators>()->setServoAngle(servo1, 0);
        modules.get<Actuators>()->setServoAngle(servo2, 0);
    }
}

float AutomaticWingAlgorithm::algorithmStep(NASState state, Vector2f windNED)
{
    float result;
    // For some algorithms the third component is needed!
    Vector3f currentPosition(state.n, state.e, state.d);

    Vector2f heading;  // used for logging purposes

    float targetAngle = guidance.calculateTargetAngle(currentPosition, heading);

    Vector2f relativeVelocity(state.vn - windNED[0], state.ve - windNED[1]);

    // Compute the angle of the current velocity
    float velocityAngle;

    // In case of a 0 north velocity i force the angle to 90
    if (relativeVelocity[0] == 0 && relativeVelocity[1] == 0)
    {
        velocityAngle = 0;
    }
    else if (relativeVelocity[1] == 0)
    {
        velocityAngle = (relativeVelocity[0] > 0 ? 1 : -1) * Constants::PI / 2;
    }
    else
    {
        velocityAngle = atan2(relativeVelocity[1], relativeVelocity[0]);
    }

    // Compute the angle difference
    float error = angleDiff(targetAngle, velocityAngle);

    // Call the PI with the just calculated error. The result is in RADIANS,
    // if positive we activate one servo, if negative the other
    result = controller->update(error);

    // Convert the result from radians back to degrees
    result = result * (180.f / Constants::PI);

    // Flip the servo orientation
    result *= -1;

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

}  // namespace Parafoil
