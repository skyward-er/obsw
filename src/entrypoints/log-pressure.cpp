/**
 * Copyright (c) 2019 Skyward Experimental Rocketry
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

#include "DeathStack/configs/SensorManagerConfig.h"
#include <interfaces-impl/hwmapping.h>
#include "DeathStack/SensorManager/Sensors/AD7994Wrapper.h"
#include <logger/Logger.h>
#include <math/Stats.h>

using namespace miosix;
using namespace DeathStackBoard;

using AD7994_t = AD7994<i2c1, ad7994_busy_pin, ad7994_nconvst>;

float adcToPa(uint16_t adc_raw)
{
    return (adc_raw*4.3f/(4096.0f*5)+0.07739f)*1000/0.007826f;
}

void printStat(const char* title, StatsResult r)
{
    printf("%s:\nMin: %.3f\nMax: %.3f\nMean: %.3f\nStddev:  %.3f\n\n", title,
           r.minValue, r.maxValue, r.mean, r.stdev);
}

int main()
{
    Stats s;
    
    AD7994_t adc{sensors::ad7994::addr};
    if(adc.init())
    {
        printf("Init ok.\n");
    }else{
        printf("Init error.\n");
    }

    adc.enableChannel(AD7994_t::Channel::CH1);
   // adc.enableChannel(AD7994_t::Channel::CH3);

    Thread::sleep(2000);

    long long t = getTick();
    long long target = getTick();
    int slp = 1000/20;

    for(int i = 0; i < 10000; i++)
    {
        t = target;
        target = t + slp;
        adc.onSimpleUpdate();
        AD7994Sample s1 = adc.getLastSample(AD7994_t::Channel::CH1);
        s.add(s1.value);
        //AD7994Sample s2 = adc.getLastSample(AD7994_t::Channel::CH3);
        
        //printf("%f, %f\n", s1.value/4096.0f*4.44f);//, s2.value/4096.0f*4.44f);//, s2.value, s3.value, s4.value);

        //Thread::sleepUntil(target);
    }
    StatsResult r = s.getStats();
    printStat("NXP", r);
    for(;;)
        Thread::sleep(1000);
   /* Logger& logger = Logger::instance();
    AD7994Wrapper adc{sensors::ad7994::addr};
    if(adc.init())
    {
        printf("ADC init ok.\n");
    }else
    {
        printf("ADC init error.\n");
    }

    long long t = getTick();
    long long target = getTick();
    int slp = 1000/20;

    for(;;)
    {
        t = target;
        target = t + slp;
        adc.onSimpleUpdate();
        AD7994WrapperData data = *(adc.getDataPtr());
        printf("%d, %d, %d\n", (int)getTick(), data.nxp_baro_volt, data.honeywell_baro_volt);

        Thread::sleepUntil(target);
    }   */

}