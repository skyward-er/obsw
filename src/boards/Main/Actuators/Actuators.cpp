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

Actuators::Actuators(TaskScheduler &scheduler) : scheduler{scheduler} {
    servoAbk = std::make_unique<Servo>(
        MIOSIX_AIRBRAKES_TIM, TimerUtils::Channel::MIOSIX_AIRBRAKES_CHANNEL,
        Config::Actuators::ABK_MIN_PULSE, Config::Actuators::ABK_MAX_PULSE);

    servoExp = std::make_unique<Servo>(
        MIOSIX_EXPULSION_TIM, TimerUtils::Channel::MIOSIX_EXPULSION_CHANNEL,
        Config::Actuators::EXP_MIN_PULSE, Config::Actuators::EXP_MAX_PULSE);

    buzzer = std::make_unique<PWM>(MIOSIX_BUZZER_TIM,
                                   Config::Actuators::BUZZER_FREQUENCY);
    buzzer->setDutyCycle(TimerUtils::Channel::MIOSIX_BUZZER_CHANNEL,
                         Config::Actuators::BUZZER_DUTY_CYCLE);
}

[[nodiscard]] bool Actuators::start()
{
    servoAbk->enable();
    servoExp->enable();

    camOff();
    currerOff();
    statusOff();
    buzzerOff();

    return true;
}

void Actuators::setAbkPosition(float position)
{
    servoAbk->setPosition(position);
}

void Actuators::setExpPosition(float position)
{
    servoExp->setPosition(position);
}

bool Actuators::wiggleServo(ServosList servo)
{
    Servo *info = getServo(servo);
    if (info != nullptr)
    {
        info->setPosition(1.0f);
        Thread::sleep(1000);
        info->setPosition(0.0f);

        return true;
    }
    else
    {
        return false;
    }
}

void Actuators::camOn() { gpios::camEnable::high(); }

void Actuators::camOff() { gpios::camEnable::low(); }

void Actuators::cutterOn() { gpios::mainDeploy::high(); }

void Actuators::currerOff() { gpios::mainDeploy::low(); }

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

Servo *Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case AIR_BRAKES_SERVO:
            return servoAbk.get();
        case EXPULSION_SERVO:
            return servoExp.get();
        default:
            return nullptr;
    }
}
