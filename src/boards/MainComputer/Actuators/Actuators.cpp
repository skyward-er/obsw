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

#include <MainComputer/StateMachines/AirBrakes/AirBrakesConfig.h>
#include <MainComputer/StateMachines/Deployment/DeploymentConfig.h>

#ifndef COMPILE_FOR_HOST
#include <interfaces-impl/hwmapping.h>
#endif

using namespace miosix;
using namespace MainComputer::DeploymentConfig;
using namespace MainComputer::AirBrakesConfigs;

namespace MainComputer
{

#ifndef COMPILE_FOR_HOST

Actuators::Actuators()
    : servoExpulsion(DPL_SERVO_TIMER, DPL_SERVO_PWM_CH),
      servoAirbrakes(ABK_SERVO_TIMER, ABK_SERVO_PWM_CH),
      led1(leds::led_red1::getPin()), led2(leds::led_red2::getPin()),
      led3(leds::led_blue1::getPin()),
      cutter(actuators::nosecone::thCut1::ena::getPin())
{
}

#else

Actuators::Actuators()
    : servoExpulsion(), servoAirbrakes(), led1(GpioPin{0, 0}),
      led2(GpioPin{0, 0}), led3(GpioPin{0, 0}), cutter(GpioPin{0, 0})
{
}

#endif

}  // namespace MainComputer
