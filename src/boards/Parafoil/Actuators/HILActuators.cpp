/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include "HILActuators.h"

#include <Parafoil/Configs/ActuatorsConfigs.h>
#include <common/LedConfig.h>
#include <interfaces-impl/bsp_impl.h>
#include <logger/Logger.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;
using namespace Parafoil::ActuatorsConfigs;

namespace Parafoil
{

bool HILActuators::setServo(ServosList servoId, float percentage)
{
    if (percentage > WingConfig::MAX_SERVO_APERTURE)
        percentage = WingConfig::MAX_SERVO_APERTURE;
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(percentage);
            Logger::getInstance().log(leftServo.getState());
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(percentage);
            Logger::getInstance().log(rightServo.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool HILActuators::setServoAngle(ServosList servoId, float angle)
{
    if (angle > WingConfig::MAX_SERVO_APERTURE * LEFT_SERVO_ROTATION)
        angle = WingConfig::MAX_SERVO_APERTURE * LEFT_SERVO_ROTATION;
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(angle / LEFT_SERVO_ROTATION);
            Logger::getInstance().log(leftServo.getState());
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(angle / RIGHT_SERVO_ROTATION);
            Logger::getInstance().log(rightServo.getState());
            break;
        default:
            return false;
    }

    return true;
}

HILActuators::HILActuators()
    : leftServo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                LEFT_SERVO_MAX_PULSE),
      rightServo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                 RIGHT_SERVO_MAX_PULSE)
{
}
}  // namespace Parafoil
