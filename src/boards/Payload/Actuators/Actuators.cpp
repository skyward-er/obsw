/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Alberto Nidasio, Federico Lolli
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

#include <Payload/BoardScheduler.h>
#include <Payload/Configs/ActuatorsConfigs.h>
#include <Payload/Configs/WingConfig.h>
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Payload::ActuatorsConfigs;

namespace Payload
{

Actuators::Actuators()
{
    leftServo  = new Servo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                           LEFT_SERVO_MAX_PULSE);
    rightServo = new Servo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                           RIGHT_SERVO_MAX_PULSE);
}

bool Actuators::start()
{
    // Servos
    enableServo(PARAFOIL_LEFT_SERVO);
    setServo(PARAFOIL_LEFT_SERVO, 0);
    enableServo(PARAFOIL_RIGHT_SERVO);
    setServo(PARAFOIL_RIGHT_SERVO, 0);
}

bool Actuators::setServo(ServosList servoId, float percentage)
{
    percentage += offset;
    if (percentage > WingConfig::MAX_SERVO_APERTURE)
    {
        percentage = WingConfig::MAX_SERVO_APERTURE;
    }

    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo->setPosition(percentage);
            Logger::getInstance().log(leftServo->getState());
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo->setPosition(percentage);
            Logger::getInstance().log(rightServo->getState());
            break;
        }
        default:
        {
            return false;
        }
    }

    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
        {
            return Actuators::setServo(servoId, angle / LEFT_SERVO_ROTATION);
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            return Actuators::setServo(servoId, angle / RIGHT_SERVO_ROTATION);
        }
        default:
        {
            return false;
        }
    }
}

bool Actuators::wiggleServo(ServosList servoId)
{

    if (!setServo(servoId, 1))
    {
        return false;
    }
    Thread::sleep(1000);
    return setServo(servoId, 0);
}

bool Actuators::enableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo->enable();
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo->enable();
            break;
        }
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
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo->disable();
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo->disable();
            break;
        }
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
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            return leftServo->getPosition();
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            return rightServo->getPosition();
        }
        default:
            return 0;
    }

    return 0;
}

float Actuators::getServoAngle(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            return leftServo->getPosition() * LEFT_SERVO_ROTATION;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            return rightServo->getPosition() * RIGHT_SERVO_ROTATION;
        }
        default:
            return 0;
    }

    return 0;
}

void Actuators::setServosOffset(float offset) { this->offset = offset; }

float Actuators::getServosOffset() { return offset; }

}  // namespace Payload
