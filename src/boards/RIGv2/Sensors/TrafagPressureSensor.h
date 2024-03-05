/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <sensors/Sensor.h>
#include <sensors/SensorData.h>

#include <functional>

namespace RIGv2
{

class TrafagPressureSensor : public Boardcore::Sensor<Boardcore::PressureData>
{
public:
    TrafagPressureSensor(std::function<Boardcore::ADCData()> getVoltage,
                         float shuntResistance, float maxPressure,
                         float minCurrent = 4, float maxCurrent = 20)
        : getVoltage{getVoltage}, shuntResistance{shuntResistance},
          maxPressure{maxPressure}, minCurrent{minCurrent},
          maxCurrent{maxCurrent}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

private:
    Boardcore::PressureData sampleImpl() override
    {
        auto voltage = getVoltage();
        float pressure = voltageToPressure(voltage.voltage);

        return {voltage.voltageTimestamp, pressure};
    }

    float voltageToPressure(float voltage) {
        // First convert voltage to current
        float current = (voltage / shuntResistance) * 1000.0f;

        // Convert to a value between 0 and 1
        float value = (current - minCurrent) / (maxCurrent - minCurrent);
    
        // Scale from 0 to maxPressure
        return value * maxPressure;
    }

    std::function<Boardcore::ADCData()> getVoltage;

    float shuntResistance;
    float maxPressure;
    float minCurrent;
    float maxCurrent;
};

}  // namespace RIGv2