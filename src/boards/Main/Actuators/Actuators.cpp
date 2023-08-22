/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Main/Actuators/Actuators.h>
#include <Main/Configs/ActuatorsConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace miosix;
using namespace Main::ActuatorsConfig;

namespace Main
{
Actuators::Actuators(TaskScheduler* sched) : scheduler(sched)
{
    // Create the servo instances
    servoAbk = new Servo(SERVO_ABK_TIMER, SERVO_ABK_CHANNEL, ABK_MIN_PULSE,
                         ABK_MAX_PULSE);
    servoExp = new Servo(SERVO_EXP_TIMER, SERVO_EXP_CHANNEL, EXP_MIN_PULSE,
                         EXP_MAX_PULSE);

    // Set the buzzer
    buzzer = new PWM(BUZZER_TIMER, BUZZER_FREQUENCY);
    buzzer->setDutyCycle(BUZZER_CHANNEL, BUZZER_DUTY_CYCLE);

    // Default disable
    camOff();
    // gpios::status_led::low();
}

bool Actuators::start()
{
    servoAbk->enable();
    servoExp->enable();

    // Reset the servo position
    servoAbk->setPosition(0);
    servoExp->setPosition(0);

    return scheduler->addTask([&]() { updateBuzzer(); }, BUZZER_UPDATE_PERIOD);
}

void Actuators::setServoPosition(ServosList servo, float position)
{
    // Lock the mutex for thread sync
    miosix::Lock<FastMutex> l(mutex);
    Servo* requestedServo = getServo(servo);

    if (requestedServo != nullptr)
    {
        requestedServo->setPosition(position, true);
    }

    // Log the position
    Logger::getInstance().log(requestedServo->getState());
}

void Actuators::wiggleServo(ServosList servo)
{
    // Do not lock the mutex due to set position lock
    setServoPosition(servo, 1);
    Thread::sleep(1000);
    setServoPosition(servo, 0);
}

float Actuators::getServoPosition(ServosList servo)
{
    miosix::Lock<FastMutex> l(mutex);
    Servo* requestedServo = getServo(servo);

    if (requestedServo != nullptr)
    {
        return requestedServo->getPosition();
    }
    return -1;
}

void Actuators::camOn()
{
    miosix::Lock<FastMutex> l(mutex);
    gpios::camera_enable::high();
}

void Actuators::camOff()
{
    miosix::Lock<FastMutex> l(mutex);
    gpios::camera_enable::low();
}

void Actuators::toggleLed()
{
    miosix::Lock<FastMutex> l(mutex);

    if (ledState)
    {
        gpios::status_led::low();
    }
    else
    {
        gpios::status_led::high();
    }
    ledState = !ledState;
}

Servo* Actuators::getServo(ServosList servo)
{
    switch (servo)
    {
        case AIR_BRAKES_SERVO:
        {
            return servoAbk;
        }
        case EXPULSION_SERVO:
        {
            return servoExp;
        }
        default:
        {
            return nullptr;
        }
    }
}

void Actuators::setBuzzerArm()
{
    miosix::Lock<FastMutex> l(mutex);
    // Set the counter with respect to the update function period
    buzzerCounterOverflow = BUZZER_ARM_PERIOD / BUZZER_UPDATE_PERIOD;
}

void Actuators::setBuzzerLand()
{
    miosix::Lock<FastMutex> l(mutex);
    // Set the counter with respect to the update function period
    buzzerCounterOverflow = BUZZER_LAND_PERIOD / BUZZER_UPDATE_PERIOD;
}

void Actuators::setBuzzerOff()
{
    miosix::Lock<FastMutex> l(mutex);
    buzzerCounterOverflow = 0;
}

void Actuators::updateBuzzer()
{
    miosix::Lock<FastMutex> l(mutex);

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

}  // namespace Main