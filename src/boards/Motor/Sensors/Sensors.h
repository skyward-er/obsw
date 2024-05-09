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

#include <drivers/adc/InternalADC.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/SensorManager.h>

#include <memory>
#include <utils/ModuleManager/ModuleManager.hpp>
#include <vector>

namespace Motor
{

class Sensors : public Boardcore::Module
{
public:
    explicit Sensors(Boardcore::TaskScheduler &scheduler) : scheduler{scheduler}
    {
    }

    bool isStarted();

    [[nodiscard]] bool start();

    std::vector<Boardcore::SensorInfo> getSensorInfo();

private:
    void lps22dfInit(Boardcore::SensorManager::SensorMap_t &map);
    void lps22dfCallback();

    void h3lis331dlInit(Boardcore::SensorManager::SensorMap_t &map);
    void h3lis331dlCallback();

    void lis2mdlInit(Boardcore::SensorManager::SensorMap_t &map);
    void lis2mdlCallback();

    void lsm6dsrxInit(Boardcore::SensorManager::SensorMap_t &map);
    void lsm6dsrxCallback();

    void ads131m08Init(Boardcore::SensorManager::SensorMap_t &map);
    void ads131m08Callback();

    void internalAdcInit(Boardcore::SensorManager::SensorMap_t &map);
    void internalAdcCallback();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");

    std::atomic<bool> started{false};

    std::unique_ptr<Boardcore::LPS22DF> lps22df;
    std::unique_ptr<Boardcore::H3LIS331DL> h3lis331dl;
    std::unique_ptr<Boardcore::LIS2MDL> lis2mdl;
    std::unique_ptr<Boardcore::LSM6DSRX> lsm6dsrx;
    std::unique_ptr<Boardcore::ADS131M08> ads131m08;
    std::unique_ptr<Boardcore::InternalADC> internalAdc;

    std::unique_ptr<Boardcore::SensorManager> manager;

    Boardcore::TaskScheduler &scheduler;
};

}  // namespace Motor