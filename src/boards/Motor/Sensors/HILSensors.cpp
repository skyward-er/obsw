/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Matteo Pignataro
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

#include "HILSensors.h"

#include <Motor/Buses.h>
#include <Motor/Configs/SensorsConfig.h>
#include <drivers/interrupt/external_interrupts.h>
#include <interfaces-impl/hwmapping.h>
#include <logger/Logger.h>

using namespace std;
using namespace miosix;
using namespace HILConfig;
using namespace Boardcore;
using namespace Motor::SensorsConfig;

namespace Motor
{

Boardcore::ChamberPressureSensorData HILSensors::getChamberPressureSensorData()
{
    miosix::PauseKernelLock lock;
    if (chamberPressure == nullptr)
    {
        return ChamberPressureSensorData{};
    }

    auto sample =
        static_cast<Boardcore::PressureData>(chamberPressure->getLastSample());
    ChamberPressureSensorData convertedSample;
    convertedSample.pressureTimestamp = sample.pressureTimestamp;
    convertedSample.pressure          = sample.pressure;

    return convertedSample;
};

HILSensors::HILSensors(Boardcore::TaskScheduler* sched) : Sensors(sched){};

void HILSensors::chamberPressureInit()
{
    chamberPressure =
        new MotorHILChamberBarometer(&ModuleManager::getInstance()
                                          .get<MotorHIL>()
                                          ->hilTransceiver->getSensorData()
                                          ->pressureChamber);

    // Emplace the sensor inside the map
    SensorInfo info("CHAMBER_PRESSURE", SensorsConfig::SAMPLE_PERIOD_ADS131,
                    bind(&HILSensors::chamberPressureCallback, this));

    sensorsMap.emplace(make_pair(chamberPressure, info));
};

void HILSensors::chamberPressureCallback()
{
    Logger::getInstance().log(getChamberPressureSensorData());
}

}  // namespace Motor