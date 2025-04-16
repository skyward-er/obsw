/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <RIGv2/Configs/TARS3Config.h>
#include <common/MedianFilter.h>
#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

#include "TARS3Data.h"

namespace RIGv2
{

class Sensors;
class Actuators;
class BoardScheduler;

class TARS3
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors, Actuators>,
      public Boardcore::HSM<TARS3>
{
public:
    TARS3();

    [[nodiscard]] bool start();

private:
    void sample();

    // HSM states

    /**
     * @brief TARS3 is ready and waiting for the user to start cold refueling.
     */
    Boardcore::State Ready(const Boardcore::Event& event);

    /**
     * @brief Super state for when TARS3 is refueling.
     */
    Boardcore::State Refueling(const Boardcore::Event& event);

    /**
     * @brief TARS3 is waiting for the pressure to stabilize after completing a
     * filling-venting cycle.
     * Super state: Refueling
     */
    Boardcore::State RefuelingWaitAfterCycle(const Boardcore::Event& event);

    /**
     * @brief TARS3 is filling the OX tank.
     * Super state: Refueling
     */
    Boardcore::State RefuelingFilling(const Boardcore::Event& event);

    /**
     * @brief TARS3 is waiting for the pressure to stabilize after filling.
     * Super state: Refueling
     */
    Boardcore::State RefuelingWaitAfterFilling(const Boardcore::Event& event);

    /**
     * @brief TARS3 is venting the OX tank.
     * Super state: Refueling
     */
    Boardcore::State RefuelingVenting(const Boardcore::Event& event);

    void logAction(Tars3Action action, float data = 0);
    void logSample(float pressure, float mass);

    std::chrono::milliseconds fillingTime{0};
    std::chrono::milliseconds ventingTime{0};

    float previousPressure = 0;
    float currentPressure  = 0;

    int medianSamples = 0;
    MedianFilter<float, Config::TARS3::MEDIAN_SAMPLE_NUMBER> massFilter;
    MedianFilter<float, Config::TARS3::MEDIAN_SAMPLE_NUMBER> pressureFilter;

    miosix::FastMutex sampleMutex;
    float massSample     = 0;
    float pressureSample = 0;

    uint16_t delayedEventId = 0;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("tars3");
};
}  // namespace RIGv2
