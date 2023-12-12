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
#include <common/LedConfig.h>
#include <interfaces-impl/bsp_impl.h>
#include <logger/Logger.h>

using namespace miosix;
using namespace Boardcore;
using namespace Common;
using namespace Parafoil::ActuatorsConfigs;

namespace Parafoil
{

bool Actuators::startModule()
{
    return (
        enableServo(PARAFOIL_LEFT_SERVO) && setServo(PARAFOIL_LEFT_SERVO, 0) &&
        enableServo(PARAFOIL_RIGHT_SERVO) && setServo(PARAFOIL_RIGHT_SERVO, 0));
}

bool Actuators::enableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.enable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.enable();
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::setServo(ServosList servoId, float percentage)
{
    percentage = percentage + offset;
    if (percentage > WingConfig::MAX_SERVO_APERTURE)
    {
        percentage = WingConfig::MAX_SERVO_APERTURE;
    }
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.setPosition(percentage);
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.setPosition(percentage);
            break;
        default:
            return false;
    }

    Logger::getInstance().log(getServoState(servoId));
    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    if (angle > WingConfig::MAX_SERVO_APERTURE * LEFT_SERVO_ROTATION)
    {
        angle = WingConfig::MAX_SERVO_APERTURE * LEFT_SERVO_ROTATION;
    }

    return setServo(servoId, angle / LEFT_SERVO_ROTATION);
}

bool Actuators::wiggleServo(ServosList servoId)
{
    bool result;
    result = setServo(servoId, 1.0);
    Logger::getInstance().log(getServoState(servoId));
    Thread::sleep(1000);
    result = result && setServo(servoId, 0.0);
    Logger::getInstance().log(getServoState(servoId));
    return result;
}

bool Actuators::disableServo(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            leftServo.enable();
            break;
        case PARAFOIL_RIGHT_SERVO:
            rightServo.enable();
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
            return leftServo.getPosition();
        case PARAFOIL_RIGHT_SERVO:
            return rightServo.getPosition();
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
            return leftServo.getPosition() * LEFT_SERVO_ROTATION;
        case PARAFOIL_RIGHT_SERVO:
            return rightServo.getPosition() * RIGHT_SERVO_ROTATION;
        default:
            return 0;
    }

    return 0;
}

ServoData Actuators::getServoState(ServosList servoId)
{

    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            return leftServo.getState();
        case PARAFOIL_RIGHT_SERVO:
            return rightServo.getState();
        default:
            return ServoData{};
    }
}

void Actuators::setOffset(float off) { offset = off; }

float Actuators::getOffset() { return offset; }

void Actuators::startTwirl()
{

    setServo(PARAFOIL_LEFT_SERVO, SERVO_TWIRL_RADIUS);
    setServo(PARAFOIL_RIGHT_SERVO, 0);
}

void Actuators::stopTwirl()
{
    setServo(PARAFOIL_LEFT_SERVO, 0);
    setServo(PARAFOIL_RIGHT_SERVO, 0);
}

Actuators::Actuators()
    : leftServo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                LEFT_SERVO_MAX_PULSE),
      rightServo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                 RIGHT_SERVO_MAX_PULSE),
      offset(0.0)
{
}
}  // namespace Parafoil
