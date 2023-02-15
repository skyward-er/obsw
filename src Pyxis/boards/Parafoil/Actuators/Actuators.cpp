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

#include "Actuators.h"

#include <Parafoil/Configs/ActuatorsConfigs.h>

#ifndef COMPILE_FOR_HOST
#include <interfaces-impl/hwmapping.h>
#endif

using namespace miosix;
using namespace Parafoil::ActuatorsConfigs;

namespace Parafoil
{

bool Actuators::setServo(ServosList servoId, float percentage)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            servo1.setPosition(percentage);
            break;
        case PARAFOIL_RIGHT_SERVO:
            servo2.setPosition(percentage);
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            servo1.setPosition(angle / SERVO_1_ROTATION);
            break;
        case PARAFOIL_RIGHT_SERVO:
            servo2.setPosition(angle / SERVO_2_ROTATION);
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::wiggleServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            servo1.setPosition(1);
            Thread::sleep(1000);
            servo1.setPosition(0);
            break;
        case PARAFOIL_RIGHT_SERVO:
            servo2.setPosition(1);
            Thread::sleep(1000);
            servo2.setPosition(0);
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::enableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            servo1.enable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            servo2.enable();
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::disableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            servo1.enable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            servo1.enable();
            break;
        default:
            return false;
    }

    return true;
}

float Actuators::getServoPosition(ServosList servoId)
{

    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            return servo1.getPosition();
        case PARAFOIL_RIGHT_SERVO:
            return servo2.getPosition();
        default:
            return 0;
    }

    return 0;
}

Actuators::Actuators()
    : servo1(SERVO_1_TIMER, SERVO_1_PWM_CH, SERVO_1_MIN_PULSE,
             SERVO_1_MAX_PULSE),
      servo2(SERVO_2_TIMER, SERVO_2_PWM_CH, SERVO_2_MIN_PULSE,
             SERVO_2_MAX_PULSE)
{
}

}  // namespace Parafoil
