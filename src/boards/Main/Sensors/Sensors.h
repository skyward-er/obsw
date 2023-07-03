/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/SensorManager.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class Sensors : public Boardcore::Module
{
public:
    Sensors(Boardcore::TaskScheduler* sched);

    [[nodiscard]] bool start();

    /**
     * @brief Stops the sensor manager
     * @warning Stops the passed scheduler
     */
    void stop();

    /**
     * @brief Returns if all the sensors are started successfully
     */
    bool isStarted();

    /**
     * @brief Calibrates the sensors with an offset
     */
    void calibrate();

    // Sensor getters
    Boardcore::LPS22DFData getLPS22DFLastSample();
    Boardcore::LPS28DFWData getLPS28DFW_1LastSample();
    Boardcore::LPS28DFWData getLPS28DFW_2LastSample();
    Boardcore::H3LIS331DLData getH3LIS331DLLastSample();
    Boardcore::LIS2MDLData getLIS2MDLLastSample();
    Boardcore::UBXGPSData getGPSLastSample();

private:
    // Init and callbacks methods
    void lps22dfInit();
    void lps22dfCallback();

    void lps28dfw_1Init();
    void lps28dfw_1Callback();

    void lps28dfw_2Init();
    void lps28dfw_2Callback();

    void h3lis331dlInit();
    void h3lis331dlCallback();

    void lis2mdlInit();
    void lis2mdlCallback();

    void ubxgpsInit();
    void ubxgpsCallback();

    // Sensors instances
    Boardcore::LPS22DF* lps22df       = nullptr;
    Boardcore::LPS28DFW* lps28dfw_1   = nullptr;
    Boardcore::LPS28DFW* lps28dfw_2   = nullptr;
    Boardcore::H3LIS331DL* h3lis331dl = nullptr;
    Boardcore::LIS2MDL* lis2mdl       = nullptr;
    Boardcore::UBXGPSSpi* ubxgps      = nullptr;

    // Sensor manager
    Boardcore::SensorManager* manager = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorMap;
    Boardcore::TaskScheduler* scheduler = nullptr;

    // SD logger
    Boardcore::Logger& SDlogger = Boardcore::Logger::getInstance();

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Sensors");
};
}  // namespace Main