/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Parafoil/ParafoilTest.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <algorithms/NAS/NASState.h>
#include <math.h>
#include <utils/Constants.h>

using namespace Boardcore;
using namespace Eigen;

namespace Parafoil
{
AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki,
                                               ServoInterface* servo1,
                                               ServoInterface* servo2)
    : WingAlgorithm(servo1, servo2, "")
{
    // TODO define umin and umax for antiwindup purposes
    controller = new PIController(Kp, Ki, 0.1, -2.09439, 2.09439);
}

AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki)
    : WingAlgorithm("")
{
    controller = new PIController(Kp, Ki, 0.1, -2.09439, 2.09439);
}

AutomaticWingAlgorithm::~AutomaticWingAlgorithm() { delete (controller); }

void AutomaticWingAlgorithm::step()
{
    // The PI calculated result
    float result;

    // Acquire the last nas state
    NASState state = ParafoilTest::getInstance().algorithms->getNASLastSample();
    UBXGPSData gps = ParafoilTest::getInstance().sensors->getGPSLastSample();

    // Target direction in respect to the current one
    // TODO to be logged
    auto targetPosition =
        ParafoilTest::getInstance().wingController->getTargetPosition();
    Vector2f targetDirection = targetPosition - Vector2f(state.n, state.e);

    // Compute the angle of the target direciton
    float targetAngle = atan2(targetDirection[1], targetDirection[0]);

    // Compute the angle of the current velocity
    float velocityAngle;

    // In case of a 0 north velocity i force the angle to 90
    if (state.vn == 0 && state.ve == 0)
    {
        velocityAngle = 0;
    }
    else if (state.vn == 0)
    {
        velocityAngle = (state.ve > 0 ? 1 : -1) * Constants::PI / 2;
    }
    else
    {
        velocityAngle = atan2(state.ve, state.vn);
    }

    // Compute the angle difference
    float error = targetAngle - velocityAngle;

    // Angle difference
    if (error < -Constants::PI || Constants::PI < error)
    {
        int moduledError = (int)fmod(error, 2 * Constants::PI);
        if (moduledError == 0 && error > 0)
        {
            error = 2 * Constants::PI;
        }
    }

    // Call the PI with the just calculated error. The result is in RADIANS, if
    // positive we activate one servo, if negative the other
    result = controller->update(error);

    // Convert the result from radians back to degrees
    result = (result / (2 * Constants::PI)) * 360;

    // Actuate the result
    if (result < 0)
    {
        // Activate the servo1 and reset servo2
        if (servo1 != NULL)
            servo1->set(-1 * result);
        if (servo2 != NULL)
            servo2->set(WING_SERVO2_RESET_POSITION);
    }
    else
    {
        // Activate the servo2 and reset servo1
        if (servo1 != NULL)
            servo1->set(WING_SERVO1_RESET_POSITION);
        if (servo2 != NULL)
            servo2->set(WING_SERVO2_MAX_POSITION - result);
    }

    // Log the servo positions
    WingAlgorithmData data;
    data.timestamp     = TimestampTimer::getTimestamp();
    data.servo1Angle   = servo1 == NULL ? 0 : servo1->getCurrentPosition();
    data.servo2Angle   = servo2 == NULL ? 0 : servo2->getCurrentPosition();
    data.targetX       = targetDirection[0];
    data.targetY       = targetDirection[1];
    data.targetAngle   = targetAngle;
    data.velocityAngle = velocityAngle;
    data.error         = error;
    data.pidOutput     = result;
    SDlogger->log(data);
}

}  // namespace Parafoil