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

#include <Main/BoardScheduler.h>
#include <Main/Configs/ActuatorsConfig.h>
#include <interfaces-impl/hwmapping.h>

using namespace Main;
using namespace Boardcore;
using namespace miosix;

[[nodiscard]] bool Actuators::start()
{
    ModuleManager &modules = ModuleManager::getInstance();
    TaskScheduler &scheduler =
        modules.get<BoardScheduler>()->getLowPriorityActuatorsScheduler();

    servoAbk->enable();
    servoExp->enable();

    camOff();
    currerOff();
    statusOff();
    buzzerOff();

    uint8_t result1 = scheduler.addTask([this]() { updateBuzzer(); },
                                        Config::Actuators::BUZZER_UPDATE_PERIOD,
                                        TaskScheduler::Policy::RECOVER);

    uint8_t result2 = scheduler.addTask([this]() { updateStatus(); },
                                        Config::Actuators::STATUS_UPDATE_PERIOD,
                                        TaskScheduler::Policy::RECOVER);

    return result1 != 0 && result2 != 0;
}

void Actuators::setAbkPosition(float position)
{
    servoAbk->setPosition(position);
}

void Actuators::openExpulsion() { servoExp->setPosition(1.0f); }

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
        // TODO(davide.mor): Send the command via CanBus, maybe someone else has
        // this servo
        return false;
    }
}

void Actuators::setStatusOff() { statusOverflow = 0; }

void Actuators::setStatusOk()
{
    statusOverflow = Config::Actuators::STATUS_OK_PERIOD;
}

void Actuators::setStatusErr()
{
    statusOverflow = Config::Actuators::STATUS_ERR_PERIOD;
}

void Actuators::setBuzzerOff() { buzzerOverflow = 0; }

void Actuators::setBuzzerArmed()
{
    buzzerOverflow = Config::Actuators::BUZZER_ARM_PERIOD;
}

void Actuators::setBuzzerLand()
{
    buzzerOverflow = Config::Actuators::BUZZER_LAND_PERIOD;
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

void Actuators::updateBuzzer()
{
    if (buzzerOverflow == 0)
    {
        buzzerOff();
    }
    else
    {
        if (buzzerCounter >= buzzerOverflow)
        {
            // Enable the channel for this period
            buzzerOn();
            buzzerCounter = 0;
        }
        else
        {
            buzzerOff();
            buzzerCounter += Config::Actuators::BUZZER_UPDATE_PERIOD;
        }
    }
}

void Actuators::updateStatus()
{
    if (buzzerOverflow == 0)
    {
        statusOff();
    }
    else
    {
        if (buzzerCounter >= buzzerOverflow)
        {
            statusOn();
        }
        else
        {
            statusOff();
        }

        if (buzzerCounter >= buzzerOverflow * 2)
        {
            // Reset the counter
            buzzerCounter = 0;
        }
        else
        {
            buzzerCounter += Config::Actuators::BUZZER_UPDATE_PERIOD;
        }
    }
}
