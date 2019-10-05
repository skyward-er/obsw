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

#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <iostream>
#include <sstream>

#include "DeathStack/DeploymentController/ThermalCutter/Cutter.h"
#include "DeathStack/SensorManager/Sensors/ADCWrapper.h"

using namespace std;
using namespace miosix;
using namespace DeathStackBoard;

static constexpr int CUT_TIME = 10000;

long long measured_cut_time = 0;
void wait()
{
    long long t0 = getTick();
    for (long long t = t0; t < t0 + CUT_TIME; t += 50)
    {
        if (inputs::btn_open::value() == 0)
        {
            break;
        }
        Thread::sleep(50);
    }
    measured_cut_time = getTick() - t0;
    // printf("Stopped!\n");
}

bool print = false;

float vToI(uint16_t adc) { return ((float)(adc - 109)) * 19500 / 510.0f; }

void csense(void*)
{
    ADCWrapper adc;
    adc.getCurrentSensorPtr()->init();

    for (;;)
    {
        adc.getCurrentSensorPtr()->onSimpleUpdate();
        uint16_t raw1 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->raw_value_1;
        uint16_t raw2 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->raw_value_2;

        float current1 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_1;
        float current2 =
            adc.getCurrentSensorPtr()->getCurrentDataPtr()->current_2;
        if (print)
        {
            printf("%d,%d,%d,%f,%f\n", (int)miosix::getTick(), (int)raw1,
                   (int)raw2, current1, current2);
        }
        Thread::sleep(100);
    }
}

int main()
{
    Thread::create(csense, 2048);

    for (;;)
    {
        print = false;
        printf(
            "Info: To stop cutting (and to stop the cut timer) press the OPEN "
            "button\n");
        printf("What do you want to cut? (1 / 2)\n");
        unsigned int c, freq = 0;
        float duty = 0;
        string temp;
        getline(cin, temp);
        stringstream(temp) >> c;
        cout << "Insert frequency (Hz): \n";
        getline(cin, temp);
        stringstream(temp) >> freq;

        cout << "Insert duty  cycle(%): \n";
        getline(cin, temp);
        stringstream(temp) >> duty;

        printf("Cutting %d, freq: %d, duty: %f\n", c, freq, duty);

        if (!(freq > 1 && freq < 20000 && duty > 0.0f && duty <= 100.0f))
        {
            printf("Wrong inputs!\n");
            continue;
        }

        print = true;

        Cutter cutter{freq, duty / 100};

        if (c == 1)
        {
            cutter.startCutDrogue();
            wait();
            cutter.stopCutDrogue();
        }
        else if (c == 2)
        {
            cutter.startCutMainChute();
            wait();
            cutter.stopCutMainChute();
        }

        Thread::sleep(2000);
        print = false;
        Thread::sleep(500);
        printf("Cut Time: %.2f s\n", (measured_cut_time) / 1000.0f);
        printf("Done!\n\n\n");
    }

    return 0;
}