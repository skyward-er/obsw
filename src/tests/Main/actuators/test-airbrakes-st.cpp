/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Alberto Nidasio
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
#include <Main/Sensors/Sensors.h>
#include <actuators/Servo/Servo.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ClockUtils.h>

#include <iostream>

#include "HIL_actuators/HILServo.h"

using namespace Boardcore;
using namespace miosix;

HILServo airbrakesServo(TIM8, TimerUtils::Channel::CHANNEL_2, 1080, 1600);

// Position to cycle through for the servo 1, 2 and 3

void moveServo()
{
    airbrakesServo.setPosition(1);
    Thread::sleep(1000);
    airbrakesServo.setPosition(0);
}

int main()
{
    char c[10];

    // Set the clock divider for the analog circuitry (/8)
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;
    InternalADC internalADC = InternalADC(ADC3, 3.3);
    internalADC.enableChannel(InternalADC::CH5);
    internalADC.init();

    std::function<ADCData()> get_batVoltage_function =
        std::bind(&InternalADC::getVoltage, &internalADC, InternalADC::CH5);

    BatteryVoltageSensor batterySensor(get_batVoltage_function, 5.98);

    // Enable the timers
    airbrakesServo.enable();
    airbrakesServo.setPosition(0);

    while (true)
    {
        // checking for battery charge. if too low for the actuator (< 10.5 V),
        // disable the real actuation of the servo
        internalADC.sample();
        batterySensor.sample();
        float vbat = batterySensor.getLastSample().batVoltage -
                     0.4;  // subtracting 0.4 as offset

        if (vbat < 10.5)
        {
            airbrakesServo.disable();
            printf("Airbrakes can't be attuated, vbat too low\n");
        }

        scanf("%c\n", c);
        moveServo();
        printf("vbat: %f\n", vbat);
    }
}
