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

#include <Parafoil/Actuators/Actuators.h>
#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/NASController/NASController.h>
#include <Parafoil/Sensors/Sensors.h>
#include <Parafoil/Wing/AutomaticWingAlgorithm.h>
#include <Parafoil/Wing/WingController.h>
#include <algorithms/NAS/NASState.h>
#include <drivers/timer/TimestampTimer.h>
#include <math.h>
#include <utils/Constants.h>

using namespace Boardcore;
using namespace Eigen;

namespace Parafoil
{

AutomaticWingAlgorithm::AutomaticWingAlgorithm(float Kp, float Ki)
    : WingAlgorithm("")
{
    controller = new PIController(
        Kp, Ki, WingConfig::WING_UPDATE_PERIOD / 1000.0f, -2.09439, 2.09439);
}

AutomaticWingAlgorithm::~AutomaticWingAlgorithm() { delete (controller); }

void AutomaticWingAlgorithm::step()
{
    auto nas = NASController::getInstance().getNasState();

    // Target direction in respect to the current one
    // TODO: To be logged
    auto targetPosition = WingController::getInstance().getTargetPosition();
    Vector2f targetDirection = targetPosition - Vector2f(nas.n, nas.e);

    // Compute the angle of the target direction
    float targetAngle = atan2(targetDirection[1], targetDirection[0]);

    // Compute the angle of the current velocity
    float velocityAngle;

    // In case of a 0 north velocity i force the angle to 90
    if (nas.vn == 0 && nas.ve == 0)
        velocityAngle = 0;
    else if (nas.vn == 0)
        velocityAngle = (nas.ve > 0 ? 1 : -1) * Constants::PI / 2;
    else
        velocityAngle = atan2(nas.ve, nas.vn);

    // Compute the angle difference
    float error = targetAngle - velocityAngle;

    // Angle difference
    if (error < -Constants::PI || Constants::PI < error)
    {
        int errorModule = (int)fmod(error, 2 * Constants::PI);
        if (errorModule == 0 && error > 0)
            error = 2 * Constants::PI;
    }

    // Call the PI with the just calculated error. The result is in servo
    // percentage position [0-1]
    float result = controller->update(error);

    // Actuate the result
    if (result < 0)
    {
        // Activate the servo1 and reset servo2
        Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, -result);
        Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, 0);
    }
    else
    {
        // Activate the servo2 and reset servo1
        Actuators::getInstance().setServo(PARAFOIL_LEFT_SERVO, 0);
        Actuators::getInstance().setServo(PARAFOIL_RIGHT_SERVO, result);
    }

    WingAlgorithmData data;
    data.timestamp = TimestampTimer::getTimestamp();
    data.servo1Angle =
        Actuators::getInstance().getServoPosition(PARAFOIL_LEFT_SERVO);
    data.servo2Angle =
        Actuators::getInstance().getServoPosition(PARAFOIL_RIGHT_SERVO);
    data.targetX       = targetDirection[0];
    data.targetY       = targetDirection[1];
    data.targetAngle   = targetAngle;
    data.velocityAngle = velocityAngle;
    data.error         = error;
    data.pidOutput     = result;
    Logger::getInstance().log(data);
}

}  // namespace Parafoil
