/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include <Groundstation/Automated/BoardScheduler.h>
#include <drivers/timer/TimestampTimer.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

#include "Groundstation/LyraGS/Buses.h"
#include "sensors/SensorManager.h"
#include "sensors/Vectornav/VN300/VN300.h"

namespace Antennas
{

static constexpr uint8_t VN300_CAL_CONVERGENCE =
    4;  ///< Calibration convergence parameter for VN300 soft and hard iron
        ///< calibration. 5: converge in 60-90sec, 1: converge in 15-20sec
static constexpr std::chrono::seconds VN300_CAL_TIME = std::chrono::seconds(30);

class Sensors
    : public Boardcore::InjectableWithDeps<LyraGS::Buses, BoardScheduler>
{
public:
    Sensors();

    /**
     * @brief Starts the SensorManager.
     */
    bool start();

    /**
     * @brief Returns the last sample of the VN300.
     */
    Boardcore::VN300Data getVN300LastSample();

    /**
     * @brief Trigger the calibration process for soft-hard iron in the VN300
     */
    bool calibrate();

    /**
     * @brief Returns the status of the calibration
     */
    bool isCalibrating();

private:
    bool vn300Init();
    void vn300Callback();

    std::atomic<bool> calibrating{false};

    Boardcore::VN300* vn300 = nullptr;

    std::unique_ptr<Boardcore::SensorManager> sm = nullptr;
    Boardcore::SensorManager::SensorMap_t sensorsMap;
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("sensors");
    std::chrono::nanoseconds calibrationStart;
};
}  // namespace Antennas
