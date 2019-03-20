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

#include "AD7994WrapperData.h"
#include "DeathStack/configs/SensorManagerConfig.h"
#include "drivers/adc/AD7994.h"
#include "drivers/adc/AD7994Data.h"



namespace DeathStackBoard
{

using AD7994_t = AD7994<BusI2C1, ad7994_busy_pin, ad7994_nconvst>;

class AD7994Wrapper : public ::Sensor
{

public:
    AD7994Wrapper(uint8_t i2c_address) : adc(i2c_address) {}

    bool init() override
    {
        bool success = adc.init();
        if (success)
        {
            adc.enableChannel(BARO_1_CHANNEL);
            adc.enableChannel(BARO_2_CHANNEL);
        }
        return success;
    }

    bool onSimpleUpdate() override
    {
        bool result = adc.onSimpleUpdate();
        if (result)
        {
            AD7994Sample baro_1 = adc.getLastSample(BARO_1_CHANNEL);
            AD7994Sample baro_2 = adc.getLastSample(BARO_2_CHANNEL);

            data.timestamp = baro_1.timestamp;
            data.baro_1_volt  = baro_1.value;
            data.baro_1_flag  = baro_1.alert_flag;

            data.baro_2_volt = baro_2.value;
            data.baro_2_flag = baro_2.alert_flag;

            // TODO: calculate pressures from adc value
            data.baro_1_pressure = 0;
            data.baro_2_pressure = 0;
        }
        return result;
    }

    bool selfTest() override { return adc.selfTest(); }

    AD7994WrapperData getData() { return data; }

private:
    AD7994_t adc;
    AD7994WrapperData data;

    static constexpr AD7994_t::Channel BARO_1_CHANNEL =
        static_cast<AD7994_t::Channel>(AD7994_BARO_1_CHANNEL);

    static constexpr AD7994_t::Channel BARO_2_CHANNEL =
        static_cast<AD7994_t::Channel>(AD7994_BARO_2_CHANNEL);
};

}  // namespace DeathStackBoard