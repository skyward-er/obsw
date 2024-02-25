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

#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/SensorManager.h>

#include <memory>
#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIGv2
{

class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler &scheduler) : scheduler{scheduler} {}

    [[nodiscard]] bool start();

    void stop();

    bool isStarted();

    Boardcore::ADS131M08Data getADC1LastSample();

private:
    void adc1Init(Boardcore::SensorManager::SensorMap_t &map);
    void adc1Callback();

    Boardcore::Logger &sdLogger = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
    Boardcore::TaskScheduler &scheduler;

    std::atomic<bool> started{false};
    std::unique_ptr<Boardcore::ADS131M08> adc1;
    std::unique_ptr<Boardcore::SensorManager> manager;
};

}  // namespace RIGv2