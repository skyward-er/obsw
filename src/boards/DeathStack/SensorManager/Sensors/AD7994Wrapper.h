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

using AD7994_t = AD7994<i2c1, ad7994_busy_pin, ad7994_nconvst>;

class AD7994Wrapper : public ::Sensor
{

public:
    AD7994Wrapper(uint8_t i2c_address, float v_ref) : adc(i2c_address), v_ref(v_ref) {}

    bool init() override
    {
        bool success = adc.init();
        if (success)
        {
            adc.enableChannel(NXP_BARO_CHANNEL);
            adc.enableChannel(HONEYWELL_BARO_CHANNEL);
        }
        return success;
    }

    bool onSimpleUpdate() override
    {
        bool result = adc.onSimpleUpdate();
        if (result)
        {
            AD7994Sample nxp_baro = adc.getLastSample(NXP_BARO_CHANNEL);
            AD7994Sample hw_baro  = adc.getLastSample(HONEYWELL_BARO_CHANNEL);

            data.timestamp = nxp_baro.timestamp;

            data.nxp_baro_volt = nxp_baro.value;
            data.nxp_baro_flag = nxp_baro.alert_flag;

            data.honeywell_baro_volt = hw_baro.value;
            data.honeywell_baro_flag = hw_baro.alert_flag;

            data.nxp_baro_pressure = nxpRaw2Pressure(data.nxp_baro_volt);
            data.honeywell_baro_pressure =
                hwRaw2Pressure(data.honeywell_baro_volt);
        }
        return result;
    }

    bool selfTest() override { return adc.selfTest(); }

    AD7994WrapperData* getDataPtr() { return &data; }

private:
    float nxpRaw2Pressure(uint16_t raw_val)
    {
        return (raw_val * v_ref / (4096 * 5.0f) + 0.07739f) * 1000 /
               0.007826f;
    }
    float hwRaw2Pressure(uint16_t raw_val)
    {
        return (1.25f * raw_val * v_ref / (4096 * 5.0f) - 0.125f) *
               100000.0f;
    }

    AD7994_t adc;
    AD7994WrapperData data;
    float v_ref;

    static constexpr AD7994_t::Channel NXP_BARO_CHANNEL =
        static_cast<AD7994_t::Channel>(AD7994_NXP_BARO_CHANNEL);

    static constexpr AD7994_t::Channel HONEYWELL_BARO_CHANNEL =
        static_cast<AD7994_t::Channel>(AD7994_HONEYWELL_BARO_CHANNEL);
};

}  // namespace DeathStackBoard