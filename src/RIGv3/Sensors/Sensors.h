/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus
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

#include <RIGv3/BoardScheduler.h>
#include <RIGv3/Buses.h>
#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/SensorManager.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>
#include <functional>
#include <memory>

namespace RIGv3
{

class Sensors : public Boardcore::InjectableWithDeps<Buses, BoardScheduler>
{
public:
    [[nodiscard]] bool start();
    bool isStarted();

    Boardcore::InternalADCData getInternalADCLastSample();
    Boardcore::ADS131M08Data getADC0LastSample();
    Boardcore::ADS131M08Data getADC1LastSample();
    Boardcore::ADS131M08Data getADC2LastSample();
    Boardcore::ADS131M08Data getADC3LastSample();

    std::vector<Boardcore::SensorInfo> getSensorInfos();

private:
    void internalAdcInit();
    void internalAdcCallback();

    void adc0Init();
    void adc0Callback();

    void adc1Init();
    void adc1Callback();

    void adc2Init();
    void adc2Callback();

    void adc3Init();
    void adc3Callback();

    bool sensorManagerInit();

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");

    std::atomic<bool> started{false};

    std::unique_ptr<Boardcore::InternalADC> internalAdc;
    std::unique_ptr<Boardcore::ADS131M08> adc0;
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::ADS131M08> adc2;
    std::unique_ptr<Boardcore::ADS131M08> adc3;

    std::unique_ptr<Boardcore::SensorManager> SPI2Manager;
    std::unique_ptr<Boardcore::SensorManager> SPI3Manager;
};

}  // namespace RIGv3
