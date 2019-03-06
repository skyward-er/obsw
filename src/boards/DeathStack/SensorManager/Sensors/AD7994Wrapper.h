/*
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

#pragma once

#include "drivers/adc/AD7994.h"
#include "drivers/adc/AD7994Data.h"
#include "AD7994WrapperData.h"

template <typename BusI2C, typename BusyPin, typename CONVST>
class AD7994Wrapper : public Sensor
{
    using AD7994_t = AD7994<BusI2C, BusyPin, CONVST>;
public:
    AD7994Wrapper(uint8_t i2c_address) : adc(i2c_address) {}

    bool init() override
    {
        bool success = adc.init();
        if(success)
        {
            adc.enableChannel(AD7994_t::Channel::CH1);
            adc.enableChannel(AD7994_t::Channel::CH2);
            //TODO: Enable required channels
        }
    }

    bool onSimpleUpdate() override
    {
        bool result = adc.onSimpleUpdate();
        if(result)
        {
            AD7994Sample ch1 = adc.getLastSample(AD7994_t::Channel::CH1);
            AD7994Sample ch2= adc.getLastSample(AD7994_t::Channel::CH2);

            data.timestamp = ch1.timestamp;
            data.ch1_volt = ch1.value;
            data.ch1_flag = ch1.alert_flag;

            data.ch2_volt = ch2.value;
            data.ch2_flag = ch2.alert_flag;

            //TODO: calculate pressures from adc value
            data.ch1_pressure = 0;
            data.ch2_pressure = 0;          
        }
        return result;
    }

    bool selfTest() override
    {
        return adc.selfTest();
    }

    AD7994WrapperData getData()
    {
        return data;
    }
private:
    AD7994_t adc;
    AD7994WrapperData data;
};