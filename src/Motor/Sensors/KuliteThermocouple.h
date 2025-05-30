/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Motor/Sensors/SensorsData.h>
#include <sensors/Sensor.h>

#include <functional>

namespace Motor
{

class KuliteThermocouple : public Boardcore::Sensor<Boardcore::TemperatureData>
{
public:
    KuliteThermocouple(std::function<Boardcore::ADCData()> getVoltage,
                       float p0Voltage, float p0Temp, float p1Voltage,
                       float p1Temp)
        : getVoltage{getVoltage},
          scale{(p1Temp - p0Temp) / (p1Voltage - p0Voltage)},
          offset{p0Temp - scale * p0Voltage}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

protected:
    Boardcore::TemperatureData sampleImpl() override
    {
        auto voltage = getVoltage();
        auto temp    = voltage.voltage * scale + offset;

        return {voltage.voltageTimestamp, temp};
    }

private:
    std::function<Boardcore::ADCData()> getVoltage;
    const float scale;
    const float offset;
};

}  // namespace Motor
