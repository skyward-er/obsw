/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Raul Radu
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

#include "AutocodedWingAlgorithm.h"

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

namespace Parafoil
{

using namespace Units::Length;

AutocodedWingAlgorithm::AutocodedWingAlgorithm(ServosList servoLeft,
                                               ServosList servoRight)
    : Super(servoLeft, servoRight)
{
}

void AutocodedWingAlgorithm::setTargetNED(Meter n, Meter e)
{
    controller.setPRF_Reference(PRFReference{
        .WindDirection = 0, .TargetPositionNED = {n.value(), e.value()}});
}

bool AutocodedWingAlgorithm::init()
{
    // Initialize the wing algorithm
    if (!Super::init())
    {
        LOG_ERR(logger, "Failed to initialize the wing algorithm");
        return false;
    }

    // Initialize the controller
    controller.initialize();

    return true;
}

void AutocodedWingAlgorithm::step()
{
    auto nasdaqState = getModule<NASController>()->getNASDAQState();

    PRFIn input = {
        .NASDAQPosition = {nasdaqState.n, nasdaqState.e, nasdaqState.d},
        .NASDAQVelocity = {nasdaqState.vn, nasdaqState.ve, nasdaqState.vd},
    };

    controller.setPRF_In(input);

    controller.step();

    // retrieve data
    PRFLogsData logsData{
        TimestampTimer::getTimestamp(),
        controller.getPRF_Logs_OBSW(),
    };

    // updated servo positions
    Radian leftCommand(logsData.PRFLogs.ServoCommands[0] *
                       Config::Wing::SERVOS_MAX_ANGLE *
                       Config::Wing::SERVO_LIMITER_PERCENTAGE);
    Radian rightCommand(logsData.PRFLogs.ServoCommands[1] *
                        Config::Wing::SERVOS_MAX_ANGLE *
                        Config::Wing::SERVO_LIMITER_PERCENTAGE);

    getModule<Actuators>()->setServoAngle(PARAFOIL_LEFT_SERVO, leftCommand);
    getModule<Actuators>()->setServoAngle(PARAFOIL_RIGHT_SERVO, rightCommand);
    // Log data
    Logger::getInstance().log(logsData);
}

}  // namespace Parafoil
