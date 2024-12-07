/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Giulia Facchi
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

#include <Motor/HIL/HIL.h>
#include <common/CanConfig.h>
#include <common/ReferenceConfig.h>
#include <drivers/timer/TimestampTimer.h>
#include <sensors/HILSensor.h>
#include <sensors/Sensor.h>

#include "Sensors.h"

namespace Motor
{

class HILSensors
    : public Boardcore::InjectableWithDeps<Boardcore::InjectableBase<Sensors>,
                                           MotorHIL>
{
public:
    explicit HILSensors(bool enableHw) : Super{}, enableHw{enableHw} {}

private:
    bool postSensorCreationHook() override
    {
        using namespace Boardcore;

        hillificator<>(ccPressure, enableHw,
                       [this]() { return updateChamberPressureSensorData(); });

        return true;
    };

    ~HILSensors() {};

    int getSampleCounter(int nData)
    {
        auto ts           = miosix::getTime();
        auto tsSensorData = getModule<MotorHIL>()->getTimestampSimulatorData();
        long long simulationPeriod =
            static_cast<long long>(
                getModule<MotorHIL>()->getSimulationPeriod()) *
            1e6;

        assert(ts >= tsSensorData &&
               "Actual timestamp is lesser then the packet timestamp");

        if (ts >= tsSensorData + simulationPeriod)
        {
            // TODO: Register this as an error
            return nData - 1;  // Return the last valid index
        }

        // Getting the index floored
        int sampleCounter = (ts - tsSensorData) * nData / simulationPeriod;

        if (sampleCounter < 0)
        {
            printf("sampleCounter: %d\n", sampleCounter);
            assert(sampleCounter < 0 && "Calculated a negative index");
            return 0;
        }

        return sampleCounter;
    }

    Boardcore::PressureData updateChamberPressureSensorData()
    {
        Boardcore::PressureData data;

        auto* sensorData = getModule<MotorHIL>()->getSensorData();

        int iCC = getSampleCounter(sensorData->pressureChamber.NDATA);

        data.pressureTimestamp = Boardcore::TimestampTimer::getTimestamp();
        data.pressure          = sensorData->pressureChamber.measures[iCC];

        return data;
    };

    bool enableHw;
};

}  // namespace Motor
