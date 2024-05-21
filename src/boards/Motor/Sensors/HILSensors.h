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

#include <Motor/Configs/HILSimulationConfig.h>
#include <Motor/Sensors/ChamberPressureSensor/ChamberPressureSensor.h>
#include <common/CanConfig.h>
#include <common/ReferenceConfig.h>
#include <sensors/Sensor.h>

#include "Sensors.h"

namespace Motor
{

class HILSensors : public Sensors
{
public:
    explicit HILSensors(Boardcore::TaskScheduler* sched, Motor::Buses* buses,
                        bool enableHw)
        : Sensors{sched, buses}, enableHw{enableHw}
    {
        using namespace HILConfig;
        using namespace Boardcore;

        chamberPressure = hillificator<>(chamberPressure, enableHw,
                                         updateChamberPressureSensorData);
    }

    ~HILSensors(){};

private:
    static int getSampleCounter(int nData)
    {
        auto ts           = miosix::getTime();
        auto tsSensorData = Boardcore::ModuleManager::getInstance()
                                .get<Boardcore::HILTransceiverBase>()
                                ->getTimestampSimulatorData();
        auto simulationPeriod = Boardcore::ModuleManager::getInstance()
                                    .get<HILConfig::MotorHIL>()
                                    ->getSimulationPeriod();

        assert(ts >= tsSensorData &&
               "Actual timestamp is lesser then the packet timestamp");

        // Getting the index floored
        int sampleCounter = (ts - tsSensorData) * nData / simulationPeriod;

        if (sampleCounter >= nData)
        {
            // TODO: Register this as an error
            return nData - 1;  // Return the last valid index
        }

        if (sampleCounter < 0)
        {
            assert(false && "Calculated a negative index");
            return 0;
        }

        return sampleCounter;
    }

    std::function<Boardcore::ChamberPressureSensorData(void)>
        updateChamberPressureSensorData = []()
    {
        Boardcore::ChamberPressureSensorData data;

        auto* hilTransceiver = static_cast<HILConfig::MotorHILTransceiver*>(
            Boardcore::ModuleManager::getInstance()
                .get<Boardcore::HILTransceiverBase>());
        auto* sensorData = hilTransceiver->getSensorData();

        int iCC = getSampleCounter(sensorData->pressureChamber.NDATA);

        data.pressureTimestamp = miosix::getTime();
        data.pressure = sensorData->pressureChamber.measures[iCC];

        return data;
    };

    bool enableHw;
};

}  // namespace Motor