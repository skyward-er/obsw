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
#include <interfaces-impl/hwmapping.h>

#include <utils/ModuleManager/ModuleManager.hpp>

using namespace miosix;
using namespace Boardcore;
using namespace Payload::ActuatorsConfigs;

namespace Payload
{

Actuators::Actuators(Boardcore::TaskScheduler* sched)
    : leftServo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                LEFT_SERVO_MAX_PULSE),
      rightServo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                 RIGHT_SERVO_MAX_PULSE),
      scheduler(sched), buzzer(BUZZER_TIMER, BUZZER_FREQUENCY)
{
    buzzer.setDutyCycle(BUZZER_CHANNEL, BUZZER_DUTY_CYCLE);
    camOff();
}

bool Actuators::start()
{
    // Servos
    enableServo(PARAFOIL_LEFT_SERVO);
    setServo(PARAFOIL_LEFT_SERVO, 0);
    enableServo(PARAFOIL_RIGHT_SERVO);
    setServo(PARAFOIL_RIGHT_SERVO, 0);

    // Signaling Devices configurations
    ledArmedTaskId =
        scheduler->addTask([&]() { toggleLed(); }, ROCKET_SS_ARMED_PERIOD);
    scheduler->disableTask(ledArmedTaskId);
    ledErrorTaskId =
        scheduler->addTask([&]() { toggleLed(); }, ROCKET_SS_ERROR_PERIOD);
    scheduler->disableTask(ledErrorTaskId);

    return ledArmedTaskId != 0 && ledErrorTaskId != 0 &&
           scheduler->addTask([&]() { updateBuzzer(); },
                              BUZZER_UPDATE_PERIOD) != 0;
}

bool Actuators::setServo(ServosList servoId, float percentage)
{
    switch (servoId)
    {
        case PARAFOIL_LEFT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo.setPosition(percentage);
            Logger::getInstance().log(leftServo.getState());
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo.setPosition(percentage);
            Logger::getInstance().log(rightServo.getState());
            break;
        }
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
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo.setPosition(angle / LEFT_SERVO_ROTATION);
            Logger::getInstance().log(leftServo.getState());
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo.setPosition(angle / RIGHT_SERVO_ROTATION);
            Logger::getInstance().log(rightServo.getState());
            break;
        }
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
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo.setPosition(1);
            Logger::getInstance().log(leftServo.getState());
            Thread::sleep(1000);
            leftServo.setPosition(0);
            Logger::getInstance().log(leftServo.getState());
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo.setPosition(1);
            Logger::getInstance().log(rightServo.getState());
            Thread::sleep(1000);
            rightServo.setPosition(0);
            Logger::getInstance().log(rightServo.getState());
            break;
        }
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
        {
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo.enable();
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo.enable();
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
            leftServo.disable();
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo.disable();
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
            return leftServo.getPosition();
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            return rightServo.getPosition();
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
            return leftServo.getPosition() * LEFT_SERVO_ROTATION;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            return rightServo.getPosition() * RIGHT_SERVO_ROTATION;
        }
        default:
            return 0;
    }

    return 0;
}

void Actuators::cuttersOn() { gpios::cut_trigger::high(); }

void Actuators::cuttersOff() { gpios::cut_trigger::low(); }

void Actuators::camOn() { gpios::camera_enable::high(); }

void Actuators::camOff() { gpios::camera_enable::low(); }

void Actuators::rocketSDArmed()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    // disable ledErrorTask if active
    if (blinkState == RocketSignalingState::ERROR)
    {
        scheduler->disableTask(ledErrorTaskId);
    }
    // enable ledArmedTask
    if (blinkState != RocketSignalingState::ARMED)
    {
        scheduler->enableTask(ledArmedTaskId);
    }
    // Set the counter with respect to the update function period
    buzzerCounterOverflow = ROCKET_SS_ARMED_PERIOD / BUZZER_UPDATE_PERIOD;

    blinkState = RocketSignalingState::ARMED;
}

void Actuators::rocketSDDisarmed()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);

    // disable ledErrorTask if active
    if (blinkState == RocketSignalingState::ERROR)
    {
        scheduler->disableTask(ledErrorTaskId);
    }
    // enable ledArmedTask
    if (blinkState != RocketSignalingState::ARMED)
    {
        scheduler->disableTask(ledArmedTaskId);
    }
    gpios::status_led::high();
    ledState = true;

    buzzerCounterOverflow = 0;
    blinkState            = RocketSignalingState::DISARMED;
}

void Actuators::rocketSDError()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    // disable ledArmedTask if active
    if (blinkState == RocketSignalingState::ARMED)
    {
        scheduler->disableTask(ledArmedTaskId);
    }
    // enable ledErrorTask
    if (blinkState != RocketSignalingState::ERROR)
    {
        scheduler->enableTask(ledErrorTaskId);
    }
    blinkState = RocketSignalingState::ERROR;
}

void Actuators::rocketSDLanded()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    // disable ledArmedTask if active
    if (blinkState == RocketSignalingState::ARMED)
    {
        scheduler->disableTask(ledArmedTaskId);
    }
    // disable ledErrorTask
    if (blinkState != RocketSignalingState::ERROR)
    {
        scheduler->disableTask(ledErrorTaskId);
    }

    gpios::status_led::high();
    // Set the counter with respect to the update function period
    buzzerCounterOverflow = ROCKET_SS_LAND_PERIOD / BUZZER_UPDATE_PERIOD;

    blinkState = RocketSignalingState::ERROR;
}

void Actuators::rocketSDOff()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    if (blinkState == RocketSignalingState::ARMED)
    {
        scheduler->disableTask(ledArmedTaskId);
    }
    if (blinkState == RocketSignalingState::ERROR)
    {
        scheduler->disableTask(ledErrorTaskId);
    }
    buzzerCounterOverflow = 0;
    gpios::status_led::low();
    blinkState = RocketSignalingState::OFF;
}

void Actuators::toggleLed()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    if (ledState)
        gpios::status_led::low();
    else
        gpios::status_led::high();

    ledState = !ledState;
}

void Actuators::updateBuzzer()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    if (buzzerCounterOverflow == 0)
    {
        // The buzzer is deactivated thus the channel is disabled
        buzzer.disableChannel(BUZZER_CHANNEL);
    }
    else
    {
        if (buzzerCounter >= buzzerCounterOverflow)
        {
            // Enable the channel for this period
            buzzer.enableChannel(BUZZER_CHANNEL);
            buzzerCounter = 0;
        }
        else
        {
            buzzer.disableChannel(BUZZER_CHANNEL);
            buzzerCounter++;
        }
    }
}

}  // namespace Payload
