/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include <miosix.h>
#include "DeathStack/DeploymentController/ThermalCutter/Cutter.h"
#include <interfaces-impl/hwmapping.h>
#include "DeathStack/SensorManager/Sensors/ADCWrapper.h"
#include <iostream>

using namespace std;
using namespace miosix;
using namespace DeathStackBoard;

void wait()
{
    long long t = getTick();
    long long target = t + 15000;
    while(t < target && inputs::btn1::value() == 1)
    {
        Thread::sleep(50);
        t += 50;
    }
}

bool print = false;

float vToI(uint16_t adc)
{
    return ((float)(adc - 109)) * 19500/510.0f;
}

void csense(void*)
{
    ADCWrapper adc;
    adc.getCurrentSensorPtr()->init();
    
    for(;;)
    {
        adc.getCurrentSensorPtr()->onSimpleUpdate();
        uint16_t current1 = adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_1_value;
        uint16_t current2 = adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_2_value;
        if(print)
            printf("C1: %f\tC2: %d\n", vToI(current1), current1);
        Thread::sleep(100);
    }
}

int main()
{
    Thread* t = Thread::create(csense, 2048);

    for (;;)
    {
        print = false;

        printf("What do you want to cut? (d, r)\n");
        char c;
        cin >> &c;
        print = true;
        
        Cutter cutter;

        if (c == 'D' || c == 'd')
        {
            cutter.startCutDrogue();
            wait();
            cutter.stopCutDrogue();
        }
        else if (c == 'R' || c == 'r')
        {
            cutter.startCutMainChute();
            wait();
            cutter.stopCutMainChute();
        }

        Thread::sleep(3000);
        printf("Done!\n\n\n");
    }
}