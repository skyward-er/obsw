/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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
#include <Payload/Configs/ActuatorsConfig.h>
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimerUtils.h>
#include <interfaces-impl/hwmapping.h>

using namespace miosix;
using namespace Boardcore;
namespace config = Payload::Config::Actuators;

namespace Payload
{

Actuators::Actuators()
{
    // Left servo is servo 1
    leftServo.servo = std::make_unique<Servo>(
        MIOSIX_PARAFOIL_SERVO_1_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_1_CHANNEL,
        config::LeftServo::MIN_PULSE.count(),
        config::LeftServo::MAX_PULSE.count());
    leftServo.fullRangeAngle = config::LeftServo::ROTATION;

    // Right servo is servo 2
    rightServo.servo = std::make_unique<Servo>(
        MIOSIX_PARAFOIL_SERVO_2_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_2_CHANNEL,
        config::RightServo::MIN_PULSE.count(),
        config::RightServo::MAX_PULSE.count());
    rightServo.fullRangeAngle = config::RightServo::ROTATION;

    buzzer =
        std::make_unique<PWM>(MIOSIX_BUZZER_TIM, config::Buzzer::FREQUENCY);
    buzzer->setDutyCycle(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL,
                         config::Buzzer::DUTY_CYCLE);
}

bool Actuators::start()
{
    auto& scheduler = getModule<BoardScheduler>()->actuators();

    leftServo.servo->enable();
    rightServo.servo->enable();

    leftServo.servo->setPosition(0);
    rightServo.servo->setPosition(0);

    cameraOff();
    cuttersOff();
    buzzerOff();
    statusOff();

    auto buzzerTaskId = scheduler.addTask([this]() { updateBuzzer(); },
                                          config::Buzzer::UPDATE_PERIOD,
                                          TaskScheduler::Policy::RECOVER);
    if (buzzerTaskId == 0)
    {
        LOG_ERR(logger, "Failed to start buzzer task");
        return false;
    }

    auto statusTaskId = scheduler.addTask([this]() { updateStatusLed(); },
                                          config::StatusLed::UPDATE_PERIOD,
                                          TaskScheduler::Policy::RECOVER);
    if (statusTaskId == 0)
    {
        LOG_ERR(logger, "Failed to start status LED task");
        return false;
    }

    started = true;
    return true;
}

bool Actuators::isStarted() { return started; }

bool Actuators::setServoPosition(ServosList servoId, float position)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return false;
    }

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(position);
    Logger::getInstance().log(actuator->servo->getState());
    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return false;
    }

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(angle / actuator->fullRangeAngle);
    Logger::getInstance().log(actuator->servo->getState());

    return true;
}

float Actuators::getServoPosition(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return -1.f;
    }

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servo->getPosition();
}

float Actuators::getServoAngle(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return -1.f;
    }

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servo->getPosition() * actuator->fullRangeAngle;
}

bool Actuators::wiggleServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return false;
    }

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setPosition(1.0f);
    Thread::sleep(1000);
    actuator->servo->setPosition(0.0f);

    return true;
}

bool Actuators::disableServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
    {
        return false;
    }

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

void Actuators::setStatusOff() { statusLedThreshold = 0; }

void Actuators::setStatusOk()
{
    statusLedThreshold = config::StatusLed::OK_PERIOD.count();
}

void Actuators::setStatusError()
{
    statusLedThreshold = config::StatusLed::ERROR_PERIOD.count();
}

void Actuators::setBuzzerOff() { buzzerThreshold = 0; }

void Actuators::setBuzzerOnLand()
{
    buzzerThreshold = config::Buzzer::ON_LAND_PERIOD.count();
}

void Actuators::setBuzzerArmed()
{
    buzzerThreshold = config::Buzzer::ARMED_PERIOD.count();
}

void Actuators::cameraOn() { gpios::camEnable::high(); }

void Actuators::cameraOff() { gpios::camEnable::low(); }

void Actuators::cuttersOn() { gpios::mainDeploy::high(); }

void Actuators::cuttersOff() { gpios::mainDeploy::low(); }

void Actuators::statusOn() { gpios::statusLed::high(); }

void Actuators::statusOff() { gpios::statusLed::low(); }

void Actuators::buzzerOn()
{
    buzzer->enableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Actuators::buzzerOff()
{
    buzzer->disableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Actuators::updateBuzzer()
{
    if (buzzerThreshold == 0)
    {
        buzzerOff();
        return;
    }

    if (buzzerCounter >= buzzerThreshold)
    {
        // Enable the buzzer for this period to emit a short beep
        buzzerOn();
        buzzerCounter = 0;
    }
    else
    {
        buzzerOff();
        buzzerCounter += config::Buzzer::UPDATE_PERIOD.count();
    }
}

void Actuators::updateStatusLed()
{
    if (statusLedThreshold == 0)
    {
        statusOff();
        return;
    }

    if (statusLedCounter >= statusLedThreshold)
    {
        statusOn();
    }
    else
    {
        statusOff();
    }

    if (statusLedCounter >= statusLedThreshold * 2)
    {
        statusLedCounter = 0;
    }
    else
    {
        statusLedCounter += config::StatusLed::UPDATE_PERIOD.count();
    }
}

}  // namespace Payload
