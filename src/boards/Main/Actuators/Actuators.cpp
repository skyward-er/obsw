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

#include <Main/Configs/ActuatorsConfigs.h>

#ifndef COMPILE_FOR_HOST
#include <interfaces-impl/hwmapping.h>
#endif

using namespace miosix;
using namespace Main::ActuatorsConfigs;

namespace Main
{

bool Actuators::setServoAngle(ServosList servoId, float angle)
{
    switch (servoId)
    {
        case AIRBRAKES_SERVO:
            servoAirbrakes.setPosition(angle / ABK_SERVO_ROTATION);
            break;
        case EXPULSION_SERVO:
            servoExpulsion.setPosition(angle / DPL_SERVO_ROTATION);
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
        case AIRBRAKES_SERVO:
            servoAirbrakes.setPosition(1);
            Thread::sleep(1000);
            servoAirbrakes.setPosition(0);
            break;
        case EXPULSION_SERVO:
            servoExpulsion.setPosition(1);
            Thread::sleep(1000);
            servoExpulsion.setPosition(0);
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
        case AIRBRAKES_SERVO:
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
        case AIRBRAKES_SERVO:
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

#ifndef COMPILE_FOR_HOST

Actuators::Actuators()
    : led1(leds::led_red1::getPin()), led2(leds::led_red2::getPin()),
      led3(leds::led_blue1::getPin()),
      cutter(actuators::nosecone::thermal_cutter_1::enable::getPin()),
      servoAirbrakes(ABK_SERVO_TIMER, ABK_SERVO_PWM_CH, ABK_SERVO_MIN_PULSE,
                     ABK_SERVO_MAX_PULSE),
      servoExpulsion(DPL_SERVO_TIMER, DPL_SERVO_PWM_CH, DPL_SERVO_MIN_PULSE,
                     DPL_SERVO_MAX_PULSE)
{
}

#else

Actuators::Actuators()
    : servoExpulsion(), servoAirbrakes(), led1(GpioPin{0, 0}),
      led2(GpioPin{0, 0}), led3(GpioPin{0, 0}), cutter(GpioPin{0, 0})
{
}

#endif

}  // namespace Main
