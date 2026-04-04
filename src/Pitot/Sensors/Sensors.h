/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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

#include <Pitot/Buses.h>
#include <Pitot/Configs/SensorsConfig.h>
#include <Pitot/Sensors/SensorData.h>
#include <sensors/ND015X/ND015A.h>
#include <sensors/ND030D/ND030D.h>
#include <sensors/SensorManager.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <mutex>

#include "SensorData.h"

namespace Pitot
{
class BoardScheduler;
class Buses;

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    Sensors() {}

    bool isStarted();

    [[nodiscard]] bool start();

    void calibrate();

    Boardcore::InternalADCData getinternalADCLastSample();

    Boardcore::ND015XData getND015ADataLastSample();
    Boardcore::ND030XData getND030DDataLastSample();

    StaticPressureData getStaticPressureLastSample();
    DynamicPressureData getDynamicPressureLastSample();

    Boardcore::VoltageData getHeatingPadNTCVoltageLastSample();
    Boardcore::TemperatureData getHeatingPadNTCTemperatureLastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

protected:
    virtual bool postSensorCreationHook() { return true; }

    Boardcore::TaskScheduler& getSensorsScheduler();

    // Digital sensors
    std::unique_ptr<Boardcore::ND015A> nd015a;
    std::unique_ptr<Boardcore::ND030D> nd030d;
    std::unique_ptr<Boardcore::InternalADC> internalADC;

    std::unique_ptr<Boardcore::SensorManager> manager;

private:

    void internalADCInit();
    void internalADCCallback();

    void nd015aInit();
    void nd015aCallback();

    void nd030dInit();
    void nd030dCallback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};
};

}  // namespace Pitot
