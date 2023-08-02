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
#include <interfaces-impl/hwmapping.h>

using namespace Boardcore;
using namespace miosix;
using namespace Main::ActuatorsConfig;

namespace Main
{
Actuators::Actuators()
{
    // Create the servo instances
    servoAbk = new Servo(SERVO_ABK_TIMER, SERVO_ABK_CHANNEL, ABK_MIN_PULSE,
                         ABK_MAX_PULSE);
    servoExp = new Servo(SERVO_EXP_TIMER, SERVO_EXP_CHANNEL, EXP_MIN_PULSE,
                         EXP_MAX_PULSE);

    // Default disable
    camOff();
    // gpios::status_led::low();
}

bool Actuators::start()
{
    servoAbk->enable();
    servoExp->enable();
    return true;
}

void Actuators::setServoPosition(ServosList servo, float position)
{
    // Pause the kernel for faster thread sync than mutex
    PauseKernelLock lock;
    Servo* requestedServo = getServo(servo);

    if (requestedServo != nullptr)
    {
        requestedServo->setPosition(position, true);
    }
}

void Actuators::wiggleServo(ServosList servo)
{
    // Do not pause the kernel due to set position kernel pause
    setServoPosition(servo, 1);
    Thread::sleep(1000);
    setServoPosition(servo, 0);
}

float Actuators::getServoPosition(ServosList servo)
{
    Servo* requestedServo = getServo(servo);

    if (requestedServo != nullptr)
    {
        return requestedServo->getPosition();
    }
    return -1;
}

void Actuators::camOn()
{
    PauseKernelLock lock;
    gpios::camera_enable::high();
}

void Actuators::camOff()
{
    PauseKernelLock lock;
    gpios::camera_enable::low();
}

void Actuators::toggleLed()
{
    PauseKernelLock lock;

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

}  // namespace Main