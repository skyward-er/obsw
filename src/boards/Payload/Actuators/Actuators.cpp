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

Actuators::Actuators(Boardcore::TaskScheduler& sched) : scheduler(sched)
{
    leftServo  = new Servo(SERVO_1_TIMER, SERVO_1_PWM_CH, LEFT_SERVO_MIN_PULSE,
                           LEFT_SERVO_MAX_PULSE);
    rightServo = new Servo(SERVO_2_TIMER, SERVO_2_PWM_CH, RIGHT_SERVO_MIN_PULSE,
                           RIGHT_SERVO_MAX_PULSE);
    buzzer     = new PWM(BUZZER_TIMER, BUZZER_FREQUENCY);
    buzzer->setDutyCycle(BUZZER_CHANNEL, BUZZER_DUTY_CYCLE);
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

    return scheduler.addTask([this]() { updateBuzzer(); }, BUZZER_UPDATE_PERIOD,
                             TaskScheduler::Policy::RECOVER) != 0;
}

bool Actuators::setServo(ServosList servoId, float percentage)
{
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
            miosix::Lock<miosix::FastMutex> ll(leftServoMutex);
            leftServo->setPosition(angle / LEFT_SERVO_ROTATION);
            Logger::getInstance().log(leftServo->getState());
            break;
        }
        case PARAFOIL_RIGHT_SERVO:
        {
            miosix::Lock<miosix::FastMutex> lr(rightServoMutex);
            rightServo->setPosition(angle / RIGHT_SERVO_ROTATION);
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

void Actuators::cuttersOn() { gpios::cut_trigger::high(); }

void Actuators::cuttersOff() { gpios::cut_trigger::low(); }

void Actuators::camOn() { gpios::camera_enable::high(); }

void Actuators::camOff() { gpios::camera_enable::low(); }

void Actuators::buzzerArmed()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    // Set the counter with respect to the update function period
    buzzerCounterOverflow = ROCKET_SS_ARMED_PERIOD / BUZZER_UPDATE_PERIOD;
}

void Actuators::buzzerLanded()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    buzzerCounterOverflow = ROCKET_SS_LAND_PERIOD / BUZZER_UPDATE_PERIOD;
}

void Actuators::buzzerOff()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    buzzerCounterOverflow = 0;
}

void Actuators::updateBuzzer()
{
    miosix::Lock<miosix::FastMutex> l(rocketSignalingStateMutex);
    if (buzzerCounterOverflow == 0)
    {
        // The buzzer is deactivated thus the channel is disabled
        buzzer->disableChannel(BUZZER_CHANNEL);
    }
    else
    {
        if (buzzerCounter >= buzzerCounterOverflow)
        {
            // Enable the channel for this period
            buzzer->enableChannel(BUZZER_CHANNEL);
            buzzerCounter = 0;
        }
        else
        {
            buzzer->disableChannel(BUZZER_CHANNEL);
            buzzerCounter++;
        }
    }
}

}  // namespace Payload
