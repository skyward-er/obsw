/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#pragma once

#include <sensors/analog/pressure/AnalogPressureSensor.h>

#include "TankPressureSensor1Data.h"

namespace Boardcore
{

class TankPressureSensor1 : public Sensor<TankPressureSensor1Data>
{
public:
    TankPressureSensor1(std::function<ADCData()> getVoltage,
                        std::function<float(float)> voltageToPressure)
        : getVoltage(getVoltage), voltageToPressure(voltageToPressure)
    {
        lastSample.pressure = 0;
    }

    bool init() override { return true; };

    bool selfTest() override { return true; };

    ///< Converts the voltage value to pressure
    TankPressureSensor1Data sampleImpl() override
    {
        ADCData adc_data = getVoltage();

        TankPressureSensor1Data new_data;
        new_data.pressure          = voltageToPressure(adc_data.voltage);
        new_data.pressureTimestamp = adc_data.voltageTimestamp;

        return new_data;
    };

private:
    ///< Function that returns the adc voltage
    std::function<ADCData()> getVoltage;

    ///< Function that converts adc voltage to current
    std::function<float(float)> voltageToPressure;
};

}  // namespace Boardcore
