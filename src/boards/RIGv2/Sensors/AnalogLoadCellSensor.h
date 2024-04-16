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

class AnalogLoadCellSensor : public Boardcore::Sensor<Boardcore::LoadCellData>
{
public:
    AnalogLoadCellSensor(std::function<Boardcore::ADCData()> getVoltage,
                         float p0Voltage, float p0Mass, float p1Voltage,
                         float p1Mass)
        : getVoltage{getVoltage}, p0Voltage{p0Voltage}, p0Mass{p0Mass},
          p1Voltage{p1Voltage}, p1Mass{p1Mass}
    {
    }

    bool init() override { return true; }

    bool selfTest() override { return true; }

private:
    Boardcore::LoadCellData sampleImpl() override
    {
        auto voltage = getVoltage();
        return {voltage.voltageTimestamp, -voltageToMass(voltage.voltage)};
    }

    float voltageToMass(float voltage)
    {
        // Two point calibration
        // m = dmass/dvoltage
        float scale  = (p1Mass - p0Mass) / (p1Voltage - p0Voltage);
        float offset = p0Mass - scale * p0Voltage;  // Calculate offset
        return scale * voltage + offset;
    }

    std::function<Boardcore::ADCData()> getVoltage;

    float p0Voltage;
    float p0Mass;
    float p1Voltage;
    float p1Mass;
};

}  // namespace RIGv2