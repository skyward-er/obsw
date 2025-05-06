/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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

#include <Parafoil/BoardScheduler.h>
#include <Parafoil/Configs/ActuatorsConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;
using namespace Boardcore::Units::Angle;
using namespace Boardcore::Units::Time;
namespace config = Parafoil::Config::Actuators;

namespace Parafoil
{

Actuators::Actuators()
{
    leftServo.servo = std::make_unique<Servo>(
        config::LeftServo::TIMER, config::LeftServo::PWM_CH,
        Microsecond{config::LeftServo::MIN_PULSE}.value(),
        Microsecond{config::LeftServo::MAX_PULSE}.value());
    leftServo.fullRangeAngle = config::LeftServo::ROTATION;

    rightServo.servo = std::make_unique<Servo>(
        config::RightServo::TIMER, config::RightServo::PWM_CH,
        Microsecond{config::RightServo::MIN_PULSE}.value(),
        Microsecond{config::RightServo::MAX_PULSE}.value());
    rightServo.fullRangeAngle = config::RightServo::ROTATION;
}

bool Actuators::start()
{
    enableServo(PARAFOIL_LEFT_SERVO);
    enableServo(PARAFOIL_RIGHT_SERVO);

    setServoPosition(PARAFOIL_LEFT_SERVO, 0.0f);
    setServoPosition(PARAFOIL_RIGHT_SERVO, 0.0f);

    started = true;
    return true;
}

bool Actuators::isStarted() { return started; }

bool Actuators::setServoPosition(ServosList servoId, float position)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(position);
    Logger::getInstance().log(actuator->servo->getState());

    return true;
}

bool Actuators::setServoAngle(ServosList servoId, Degree angle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(angle.value() /
                                 actuator->fullRangeAngle.value());
    Logger::getInstance().log(actuator->servo->getState());

    return true;
}

float Actuators::getServoPosition(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return -1.f;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servo->getPosition();
}

Degree Actuators::getServoAngle(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return Degree(-1.f);

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servo->getPosition() * actuator->fullRangeAngle;
}

bool Actuators::wiggleServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(1.0f);
    Thread::sleep(1000);
    actuator->servo->setPosition(0.0f);

    return true;
}

bool Actuators::enableServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->enable();

    return true;
}

bool Actuators::disableServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->disable();

    return true;
}

Actuators::ServoActuator* Actuators::getServoActuator(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            assert(leftServo.servo);
            return &leftServo;
        case PARAFOIL_RIGHT_SERVO:
            assert(rightServo.servo);
            return &rightServo;
        default:
            return nullptr;
    }
}

void Actuators::startTwirl()
{
    leftServo.servo->setPosition(config::SERVO_TWIRL_RADIUS);
    rightServo.servo->setPosition(0);
}

void Actuators::stopTwirl()
{
    leftServo.servo->setPosition(0);
    rightServo.servo->setPosition(0);
}

}  // namespace Parafoil
