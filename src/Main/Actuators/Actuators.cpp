/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Main/Configs/ActuatorsConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Main;
using namespace Boardcore;
using namespace miosix;
using namespace Boardcore::Units::Angle;
using namespace Config::Actuators;

Actuators::Actuators()
{
    // LEFT SERVO
    leftServo.servo = std::make_unique<ServoWinch>(
        MIOSIX_PARAFOIL_SERVO_1_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_1_CHANNEL,
        PrfServo::MIN_PULSE, PrfServo::MAX_PULSE, PrfServo::HERTZ);

    leftServo.servoTrigger = std::make_unique<SchmittTrigger>(
        Units::Angle::Radian(PrfServo::SCHMITT_THRESHOLD_LOW).value(),
        Units::Angle::Radian(PrfServo::SCHMITT_THRESHOLD_HIGH).value());

    leftServo.maxAngle  = Radian(PrfServo::MAX_ANGLE);
    leftServo.minAngle  = Radian(PrfServo::LEFT_MIN_ANGLE);
    leftServo.direction = PrfServo::LEFT_SERVO_DIRECTION;

    leftServo.servoTrigger->setTargetState(
        Radian(PrfServo::INITIAL_ANGLE).value());
    leftServo.angleData.setInitialState(PrfServo::INITIAL_ANGLE);

    // RIGHT SERVO
    rightServo.servo = std::make_unique<ServoWinch>(
        MIOSIX_PARAFOIL_SERVO_0_TIM,
        TimerUtils::Channel::MIOSIX_PARAFOIL_SERVO_0_CHANNEL,
        PrfServo::MIN_PULSE, PrfServo::MAX_PULSE, PrfServo::HERTZ);

    rightServo.servoTrigger = std::make_unique<SchmittTrigger>(
        Units::Angle::Radian(PrfServo::SCHMITT_THRESHOLD_LOW).value(),
        Units::Angle::Radian(PrfServo::SCHMITT_THRESHOLD_HIGH).value());

    rightServo.maxAngle  = Radian(PrfServo::MAX_ANGLE);
    rightServo.minAngle  = Radian(PrfServo::RIGHT_MIN_ANGLE);
    rightServo.direction = PrfServo::RIGHT_SERVO_DIRECTION;

    rightServo.servoTrigger->setTargetState(
        Radian(PrfServo::INITIAL_ANGLE).value());
    rightServo.angleData.setInitialState(PrfServo::INITIAL_ANGLE);

    // cppcheck-suppress useInitializationList
    servoAbk = std::make_unique<Servo>(
        MIOSIX_AIRBRAKES_TIM, TimerUtils::Channel::MIOSIX_AIRBRAKES_CHANNEL,
        Config::Actuators::ABK_MIN_PULSE, Config::Actuators::ABK_MAX_PULSE);
    // cppcheck-suppress useInitializationList
    buzzer = std::make_unique<PWM>(MIOSIX_BUZZER_TIM,
                                   Config::Actuators::BUZZER_FREQUENCY);
    buzzer->setDutyCycle(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL,
                         Config::Actuators::BUZZER_DUTY_CYCLE);
}

bool Actuators::isStarted() { return started; }

bool Actuators::start()
{
    leftServo.servo->enable();
    rightServo.servo->enable();

    leftServo.servoTrigger->begin();
    rightServo.servoTrigger->begin();

    leftServo.servo->setVelocity(0.5);
    rightServo.servo->setVelocity(0.5);

    TaskScheduler& scheduler =
        getModule<BoardScheduler>()->getLowPriorityActuatorsScheduler();

    expander = &getModule<GpioExpander>()->getExpander();

    servoAbk->enable();

    camOff();
    expulsionOff();
    releaserOff();
    statusOff();
    buzzerOff();

    uint8_t result = scheduler.addTask([this]() { updateBuzzer(); },
                                       Config::Actuators::BUZZER_UPDATE_RATE,
                                       TaskScheduler::Policy::RECOVER);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add updateBuzzer task");
        return false;
    }

    result = scheduler.addTask([this]() { updateStatus(); },
                               Config::Actuators::STATUS_UPDATE_RATE,
                               TaskScheduler::Policy::RECOVER);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add updateStatus task");
        return false;
    }

    result = scheduler.addTask(
        [this]
        {
            if (leftServo.enabled)
            {
                auto servoLeftAngle =
                    getModule<Sensors>()->getAS5047DLeftLastSample();

                updateServoState(
                    PARAFOIL_LEFT_SERVO,
                    Radian(servoLeftAngle.angle - leftServo.zeroAngle.value()));
            }

            if (rightServo.enabled)
            {
                auto servoRightAngle =
                    getModule<Sensors>()->getAS5047DRightLastSample();

                updateServoState(PARAFOIL_RIGHT_SERVO,
                                 Radian(servoRightAngle.angle -
                                        rightServo.zeroAngle.value()));
            }
        },
        PrfServo::UPDATE_RATE);

    if (result == 0)
    {
        LOG_ERR(logger, "Failed to add parafoil servo task");
        return false;
    }

    started = true;
    return true;
}

bool Actuators::setPrfServoAngle(ServosList servoId, Radian angle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    auto capped_angle =
        std::min(actuator->maxAngle,
                 std::max(angle + actuator->minAngle, actuator->minAngle));

    if (actuator->direction == Config::Actuators::ServoDirection::CCW)
        capped_angle *= -1;

    actuator->servoTrigger->setTargetState(capped_angle.value());

    Logger::getInstance().log(actuator->servo->getState());

    return true;
}

bool Actuators::wigglePrfServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return false;

    setPrfServoAngle(servoId, PrfServo::MAX_ANGLE);
    do
    {
        Thread::sleep(1000);
    } while (actuator->servoTrigger->getOutput() !=
             SchmittTrigger::Activation::STOP);

    setPrfServoAngle(servoId, Radian(0));

    do
    {
        Thread::sleep(1000);
    } while (actuator->servoTrigger->getOutput() !=
             SchmittTrigger::Activation::STOP);

    return true;
}

void Actuators::setPrfServoZero()
{
    leftServo.zeroAngle =
        Radian(getModule<Sensors>()->getAS5047DLeftLastSample().angle);

    rightServo.zeroAngle =
        Radian(getModule<Sensors>()->getAS5047DRightLastSample().angle);
}

void Actuators::enablePrfServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->enabled = true;
}

void Actuators::disablePrfServo(ServosList servoId)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return;

    miosix::Lock<miosix::FastMutex> lock(actuator->mutex);

    actuator->enabled = false;
}

void Actuators::setAbkPosition(float position)
{
    Lock<FastMutex> lock{servosMutex};
    unsafeSetServoPosition(servoAbk.get(), position);
}

void Actuators::wiggleServo(ServosList servo)
{
    Lock<FastMutex> lock{servosMutex};
    Servo* info = getServo(servo);
    if (info != nullptr)
    {
        unsafeSetServoPosition(info, 1.0f);
        Thread::sleep(1000);
        unsafeSetServoPosition(info, 0.0f);
    }
    else
    {
        // Wiggle via CAN, maybe someone else has it
        wiggleCanServo(servo);
    }
}

void Actuators::wiggleCanServo(ServosList servo)
{
    getModule<CanHandler>()->sendServoOpenCommand(servo, 1000);
}

float Actuators::getServoPosition(ServosList servo)
{
    Lock<FastMutex> lock{servosMutex};
    Servo* info = getServo(servo);
    return info ? info->getPosition() : 0.0f;
}

void Actuators::setStatusOff() { statusOverflow = 0; }

void Actuators::setStatusOk()
{
    statusOverflow = Config::Actuators::STATUS_OK_RATE;
}

void Actuators::setStatusErr()
{
    statusOverflow = Config::Actuators::STATUS_ERR_RATE;
}

void Actuators::setBuzzerOff() { buzzerOverflow = 0; }

void Actuators::setBuzzerArmed()
{
    buzzerOverflow = Config::Actuators::BUZZER_ARM_RATE;
}

void Actuators::setBuzzerLand()
{
    buzzerOverflow = Config::Actuators::BUZZER_LAND_RATE;
}

void Actuators::camOn() { getModule<GpioExpander>()->camOn(); }

void Actuators::camOff() { getModule<GpioExpander>()->camOff(); }

bool Actuators::getCamState()
{
    return getModule<GpioExpander>()->getCamState();
}

void Actuators::expulsionOn() { gpios::expulsion::high(); }

void Actuators::expulsionOff() { gpios::expulsion::low(); }

bool Actuators::getExpulsionState() { return gpios::expulsion::value() != 0; }

void Actuators::releaserOn() { gpios::releaser::high(); }

void Actuators::releaserOff() { gpios::releaser::low(); }

bool Actuators::getReleaserState() { return gpios::releaser::value() != 0; }

void Actuators::statusOn() { getModule<GpioExpander>()->statusLedOn(); }

void Actuators::statusOff() { getModule<GpioExpander>()->statusLedOff(); }

void Actuators::buzzerOn()
{
    buzzer->enableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Actuators::buzzerOff()
{
    buzzer->disableChannel(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL);
}

void Actuators::unsafeSetServoPosition(Servo* servo, float position)
{
    servo->setPosition(position);
    sdLogger.log(servo->getState());
}

void Actuators::updateServoState(ServosList servoId, Radian encoderAngle)
{
    auto actuator = getServoActuator(servoId);
    if (!actuator)
        return;

    auto estimatedAngle = actuator->angleData.getUpdatedAngle(encoderAngle);

    actuator->servoTrigger->setCurrentState(estimatedAngle.value());
    actuator->servoTrigger->update();
    auto triggerOutput = actuator->servoTrigger->getOutput();

    switch (triggerOutput)
    {
        case SchmittTrigger::Activation::HIGH:
        {
            actuator->servo->setVelocity(
                Config::Actuators::PrfServo::HIGH_THRESHOLD_VELOCITY);
            break;
        }
        case SchmittTrigger::Activation::LOW:
        {
            actuator->servo->setVelocity(
                Config::Actuators::PrfServo::LOW_THRESHOLD_VELOCITY);

            break;
        }
        case SchmittTrigger::Activation::STOP:
        {
            actuator->servo->setVelocity(
                Config::Actuators::PrfServo::STOP_THRESHOLD_VELOCITY);
            break;
        }
    }
}

Actuators::ServoActuator* Actuators::getServoActuator(ServosList servoId)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
            return &leftServo;
        case PARAFOIL_RIGHT_SERVO:
            return &rightServo;
        default:
            return nullptr;
    }
}

Servo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case AIR_BRAKES_SERVO:
            return servoAbk.get();
        default:
            return nullptr;
    }
}

void Actuators::updateBuzzer()
{
    if (buzzerOverflow == 0)
    {
        buzzerOff();
    }
    else if (buzzerCounter >= buzzerOverflow)
    {
        // Enable the channel for this period
        buzzerOn();
        buzzerCounter = 0;
    }
    else
    {
        buzzerOff();
        buzzerCounter += 1;
    }
}

void Actuators::updateStatus()
{
    if (statusOverflow == 0)
    {
        statusOff();
    }
    else
    {
        if (statusCounter >= statusOverflow)
            statusOn();
        else
            statusOff();

        if (statusCounter >= statusOverflow * 2)
        {
            // Reset the counter
            statusCounter = 0;
        }
        else
        {
            statusCounter += 1;
        }
    }
}
