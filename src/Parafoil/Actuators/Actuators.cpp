/* Copyright (c) 2024-2026 Skyward Experimental Rocketry
 * Author: Niccolò Betto, Raul Radu
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
#include <drivers/timer/PWM.h>
#include <drivers/timer/TimerUtils.h>
#include <interfaces-impl/hwmapping.h>

using namespace std::chrono;
using namespace miosix;
using namespace Boardcore;
using namespace Boardcore::Units::Frequency;
namespace config = Parafoil::Config::Actuators;

namespace Parafoil
{

Actuators::Actuators()
{
    // Left servo is servo 2
    leftServo.servo = std::make_unique<ServoWinch>(
        MIOSIX_PARAFOIL_SERVO_2_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_2_CHANNEL,
        config::LeftServo::MIN_PULSE, config::LeftServo::MAX_PULSE,
        config::LeftServo::HERTZ);

    leftServo.servoTrigger = std::make_unique<SchmittTrigger>(
        Units::Angle::Radian(config::LeftServo::SCHMITT_THRESHOLD_LOW).value(),
        Units::Angle::Radian(config::LeftServo::SCHMITT_THRESHOLD_HIGH)
            .value());

    leftServo.servoTrigger->setTargetState(
        Radian(config::LeftServo::INITIAL_ANGLE).value());
    leftServo.angleData.setInitialState(config::LeftServo::INITIAL_ANGLE);

    leftServo.highServoVelocity = config::LeftServo::HIGH_THRESHOLD_VELOCITY;
    leftServo.lowServoVelocity  = config::LeftServo::LOW_THRESHOLD_VELOCITY;
    leftServo.stopServoVelocity = config::LeftServo::STOP_THRESHOLD_VELOCITY;
    leftServo.wiggleAngle       = config::LeftServo::WIGGLE_ANGLE;

    // Right servo is servo 1
    rightServo.servo = std::make_unique<ServoWinch>(
        MIOSIX_PARAFOIL_SERVO_1_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_1_CHANNEL,
        config::RightServo::MIN_PULSE, config::RightServo::MAX_PULSE,
        config::RightServo::HERTZ);

    rightServo.servoTrigger = std::make_unique<SchmittTrigger>(
        Units::Angle::Radian(config::RightServo::SCHMITT_THRESHOLD_LOW).value(),
        Units::Angle::Radian(config::RightServo::SCHMITT_THRESHOLD_HIGH)
            .value());

    rightServo.highServoVelocity = config::RightServo::HIGH_THRESHOLD_VELOCITY;
    rightServo.lowServoVelocity  = config::RightServo::LOW_THRESHOLD_VELOCITY;
    rightServo.stopServoVelocity = config::RightServo::STOP_THRESHOLD_VELOCITY;
    rightServo.wiggleAngle       = config::RightServo::WIGGLE_ANGLE;

    rightServo.servoTrigger->setTargetState(
        Radian(config::RightServo::INITIAL_ANGLE).value());
    rightServo.angleData.setInitialState(config::RightServo::INITIAL_ANGLE);

    auto frequency =
        static_cast<uint16_t>(Hertz{config::Buzzer::FREQUENCY}.value());
    buzzer = std::make_unique<PWM>(MIOSIX_BUZZER_TIM, frequency);
    buzzer->setDutyCycle(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL,
                         config::Buzzer::DUTY_CYCLE);
}

bool Actuators::start()
{
    using namespace std::chrono;

    auto& scheduler = getModule<BoardScheduler>()->actuators();

    leftServo.servo->enable();
    rightServo.servo->enable();

    leftServo.servoTrigger->begin();
    rightServo.servoTrigger->begin();

    leftServo.servo->setVelocity(0.5);
    rightServo.servo->setVelocity(0.5);

    cameraOff();
    cuttersOff();
    buzzerOff();
    statusOff();

    // Use SKIP policy to highlight missed executions (the LED will stay on or
    // off for longer)
    auto boardLedTaskId = scheduler.addTask(
        [on = true]() mutable
        {
            if (on)
                miosix::gpios::boardLed::high();
            else
                miosix::gpios::boardLed::low();
            on = !on;
        },
        1s, TaskScheduler::Policy::SKIP);

    if (boardLedTaskId == 0)
    {
        LOG_ERR(logger, "Failed to start board LED task");
        return false;
    }

    auto buzzerTaskId = scheduler.addTask([this]() { updateBuzzer(); },
                                          config::Buzzer::UPDATE_RATE,
                                          TaskScheduler::Policy::RECOVER);
    if (buzzerTaskId == 0)
    {
        LOG_ERR(logger, "Failed to start buzzer task");
        return false;
    }

    auto statusTaskId = scheduler.addTask([this]() { updateStatusLed(); },
                                          config::StatusLed::UPDATE_RATE,
                                          TaskScheduler::Policy::RECOVER);
    if (statusTaskId == 0)
    {
        LOG_ERR(logger, "Failed to start status LED task");
        return false;
    }

    started = true;
    return true;
}

bool Actuators::servoIsMoving(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servoTrigger->getOutput() !=
           SchmittTrigger::Activation::STOP;
}

bool Actuators::isStarted() { return started; }

bool Actuators::setServoVelocity(ServosList servoId, float velocity)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servo->setVelocity(velocity);
    Logger::getInstance().log(actuator->servo->getState());
    return true;
}

bool Actuators::setServoAngle(ServosList servoId, Radian angle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->servoTrigger->setTargetState(angle.value());

    Logger::getInstance().log(actuator->servo->getState());

    return true;
}

float Actuators::getServoVelocity(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return -1.f;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->servo->getVelocity();
}

Radian Actuators::getServoAngle(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return 0_rad;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    return actuator->angleData.getLastReading();
}

bool Actuators::wiggleServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    setServoAngle(servoId, actuator->wiggleAngle);

    do
    {
        Thread::sleep(100);
    } while (actuator->servoTrigger->getOutput() !=
             SchmittTrigger::Activation::STOP);

    setServoAngle(servoId, Radian(0));

    do
    {
        Thread::sleep(100);
    } while (actuator->servoTrigger->getOutput() !=
             SchmittTrigger::Activation::STOP);

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

void Actuators::updateServoState(ServosList servoId, Radian angle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return;

    auto estimatedAngle = actuator->angleData.getUpdatedAngle(angle);
    actuator->servoTrigger->setCurrentState(estimatedAngle.value());
    actuator->servoTrigger->update();
    auto triggerOutput = actuator->servoTrigger->getOutput();

    switch (triggerOutput)
    {
        case SchmittTrigger::Activation::HIGH:
        {
            actuator->servo->setVelocity(actuator->highServoVelocity);
            break;
        }
        case SchmittTrigger::Activation::LOW:
        {
            actuator->servo->setVelocity(actuator->lowServoVelocity);

            break;
        }
        case SchmittTrigger::Activation::STOP:
        {
            actuator->servo->setVelocity(actuator->stopServoVelocity);
            break;
        }
    }
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

void Actuators::setBuzzerLanded()
{
    // buzzerThreshold = config::Buzzer::LANDED_PERIOD.count();
    const uint32_t BUZZER_SEQUENCE[] = {25, 50, 100, 1000, 25, 50, 100, 1000};
    std::memcpy(this->buzzerSequence, BUZZER_SEQUENCE, 8 * sizeof(uint32_t));
}

void Actuators::setBuzzerArmed()
{
    const uint32_t BUZZER_SEQUENCE[] = {50, 50, 50, 2500, 50, 50, 0, 0};
    std::memcpy(this->buzzerSequence, BUZZER_SEQUENCE, 8 * sizeof(uint32_t));
    buzzerThreshold = buzzerSequence[0];
}

void Actuators::setBuzzerNoseconeDetached()
{
    const uint32_t BUZZER_SEQUENCE[] = {50, 1000, 50, 1000, 50, 1000, 50, 1000};
    std::memcpy(this->buzzerSequence, BUZZER_SEQUENCE, 8 * sizeof(uint32_t));
    buzzerThreshold = buzzerSequence[0];
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
        buzzerCounter         = 0;
        buzzerSequenceCounter = (buzzerSequenceCounter + 1) % 8;
        buzzerThreshold       = buzzerSequence[buzzerSequenceCounter];
    }
    else
    {
        buzzerOff();
        constexpr auto period =
            1000ms / static_cast<milliseconds::rep>(
                         Hertz{config::Buzzer::UPDATE_RATE}.value());
        buzzerCounter += period.count();
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
        statusOn();
    else
        statusOff();

    if (statusLedCounter >= statusLedThreshold * 2)
    {
        statusLedCounter = 0;
    }
    else
    {
        constexpr auto period =
            1000ms / static_cast<milliseconds::rep>(
                         Hertz{config::StatusLed::UPDATE_RATE}.value());
        statusLedCounter += period.count();
    }
}

}  // namespace Parafoil
