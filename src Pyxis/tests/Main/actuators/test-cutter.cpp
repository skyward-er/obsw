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

#include <drivers/adc/InternalADC.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/ClockUtils.h>

#include <iostream>

using namespace Boardcore;
using namespace miosix;

InternalADC adc(ADC1, 3.3);

void print()
{
    adc.sample();
    printf("ADC: %f, Presence: %f\n", adc.getVoltage(InternalADC::CH12).voltage,
           adc.getVoltage(InternalADC::CH15).voltage);
}

int main()
{
    ADC->CCR |= ADC_CCR_ADCPRE_0 | ADC_CCR_ADCPRE_1;

    adc.enableChannel(InternalADC::CH12);
    adc.enableChannel(InternalADC::CH15);
    adc.init();

    TaskScheduler scheduler;
    scheduler.addTask(print, 20);

    printf("Starting...\n");
    scheduler.start();

    while (true)
    {
        int state;
        std::cin >> state;

        if (state == 0)
        {
            cutter::enable::low();
            printf("Disabled cutter\n");
        }
        else
        {
            cutter::enable::high();
            printf("Enabled cutter\n");
        }
    }
}
