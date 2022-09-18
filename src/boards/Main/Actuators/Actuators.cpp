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

#include <Main/BoardScheduler.h>
#include <Main/Configs/ActuatorsConfigs.h>

#ifndef COMPILE_FOR_HOST
#include <interfaces-impl/hwmapping.h>
#endif

using namespace miosix;
using namespace Boardcore;
using namespace Main::ActuatorsConfigs;

namespace Main
{

bool Actuators::setServo(ServosList servoId, float percentage)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            servoAirbrakes.setPosition(percentage);
            Logger::getInstance().log(servoAirbrakes.getState());
            break;
        case EXPULSION_SERVO:
            servoExpulsion.setPosition(percentage);
            Logger::getInstance().log(servoExpulsion.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            servoAirbrakes.setPosition(angle / ABK_SERVO_ROTATION);
            Logger::getInstance().log(servoAirbrakes.getState());
            break;
        case EXPULSION_SERVO:
            servoExpulsion.setPosition(angle / DPL_SERVO_ROTATION);
            Logger::getInstance().log(servoExpulsion.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::wiggleServo(ServosList servoId)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            servoAirbrakes.setPosition(1);
            Logger::getInstance().log(servoAirbrakes.getState());
            Thread::sleep(ABK_WIGGLE_TIME);
            servoAirbrakes.setPosition(0);
            Logger::getInstance().log(servoAirbrakes.getState());
            break;
        case EXPULSION_SERVO:
            servoExpulsion.setPosition(1);
            Logger::getInstance().log(servoExpulsion.getState());
            Thread::sleep(DPL_WIGGLE_TIME);
            servoExpulsion.setPosition(0);
            Logger::getInstance().log(servoExpulsion.getState());
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::enableServo(ServosList servoId)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            servoAirbrakes.enable();
            break;
        case EXPULSION_SERVO:
            servoExpulsion.enable();
            break;
        default:
            return false;
    }

    return true;
}

bool Actuators::disableServo(ServosList servoId)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            servoAirbrakes.enable();
            break;
        case EXPULSION_SERVO:
            servoAirbrakes.enable();
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
        case AIR_BRAKES_SERVO:
            return servoAirbrakes.getPosition();
        case EXPULSION_SERVO:
            return servoExpulsion.getPosition();
        default:
            return 0;
    }

    return 0;
}

float Actuators::getServoAngle(ServosList servoId)
{
    switch (servoId)
    {
        case AIR_BRAKES_SERVO:
            return servoAirbrakes.getPosition() * ABK_SERVO_ROTATION;
        case EXPULSION_SERVO:
            return servoExpulsion.getPosition() * DPL_SERVO_ROTATION;
        default:
            return 0;
    }

    return 0;
}

void Actuators::buzzerError()
{
    buzzerCounterOverflow = BUZZER_ERROR_PERIOD;
    buzzerCounter         = 0;
}

void Actuators::buzzerDisarmed()
{
    buzzerCounterOverflow = BUZZER_DISARMED_PERIOD;
    buzzerCounter         = 0;
}

void Actuators::buzzerArmed()
{
    buzzerCounterOverflow = BUZZER_ARMED_PERIOD;
    buzzerCounter         = 0;
}

void Actuators::buzzerLanded()
{
    buzzerCounterOverflow = BUZZER_LANDED_PERIOD;
    buzzerCounter         = 0;
}

void Actuators::buzzerOff()
{
    buzzerCounterOverflow = -1;
    buzzerCounter         = -1;
    buzzer.disableChannel(BUZZER_CHANNEL);
}

#ifdef HILSimulation
void Actuators::sendToSimulator() { servoAirbrakes.sendToSimulator(); }
#endif  // HILSimulation

Actuators::Actuators()
    : cutter1(cutter::enable::getPin()),
      cutter1Backup(cutter::enable::getPin()),
      servoAirbrakes(ABK_SERVO_TIMER, ABK_SERVO_PWM_CH, ABK_SERVO_MIN_PULSE,
                     ABK_SERVO_MAX_PULSE),
      servoExpulsion(DPL_SERVO_TIMER, DPL_SERVO_PWM_CH, DPL_SERVO_MIN_PULSE,
                     DPL_SERVO_MAX_PULSE),
      buzzer(BUZZER_TIMER)
{
    buzzer.setFrequency(BUZZER_FREQUENCY);
    buzzer.setDutyCycle(BUZZER_CHANNEL, BUZZER_DUTY_CYCLE);

    // Start the buzzer task
    BoardScheduler::getInstance().getScheduler().addTask(
        [&]() { toggleBuzzer(); }, BUZZER_TASK_PERIOD);
}

void Actuators::toggleBuzzer()
{
    if (buzzerCounter == 0)
    {
        buzzer.enableChannel(BUZZER_CHANNEL);
        buzzerCounter = buzzerCounterOverflow / BUZZER_TASK_PERIOD;
    }
    else if (buzzerCounter > 0)
    {
        buzzer.disableChannel(BUZZER_CHANNEL);
        buzzerCounter--;
    }
}

}  // namespace Main
