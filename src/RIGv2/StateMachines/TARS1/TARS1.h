/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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

#include <RIGv2/Configs/TARS1Config.h>
#include <common/MedianFilter.h>
#include <events/HSM.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <chrono>

#include "TARS1Data.h"

namespace RIGv2
{

class Sensors;
class Actuators;
class BoardScheduler;

class TARS1
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors, Actuators>,
      public Boardcore::HSM<TARS1>
{
public:
    TARS1();

    [[nodiscard]] bool start();

    Tars1Action getLastAction() const { return lastAction; }

private:
    void sample();

    // HSM states

    /**
     * @brief TARS1 is ready and waiting for the user to start refueling.
     */
    Boardcore::State Ready(const Boardcore::Event& event);

    /**
     * @brief Super state for when TARS1 is refueling.
     */
    Boardcore::State Refueling(const Boardcore::Event& event);

    /**
     * @brief TARS1 is washing the OX tank.
     * Super state: Refueling
     *
     * Opens both the venting and filling valves to wash the OX tank.
     */
    Boardcore::State RefuelingWashing(const Boardcore::Event& event);

    /**
     * @brief TARS1 is filling the OX tank and waiting for the system to
     * stabilize.
     * Super state: Refueling
     */
    Boardcore::State RefuelingFilling(const Boardcore::Event& event);

    /**
     * @brief TARS1 is venting the OX tank.
     * Super state: Refueling
     */
    Boardcore::State RefuelingVenting(const Boardcore::Event& event);

    void updateAndLogAction(Tars1Action action);
    void logSample(float pressure, float mass);

    std::atomic<Tars1Action> lastAction{Tars1Action::READY};

    std::chrono::milliseconds ventingTime{0};

    float previousMass = 0;
    float currentMass  = 0;

    float previousPressure = 0;
    float currentPressure  = 0;

    int massStableCounter = 0;

    int medianSamples = 0;
    MedianFilter<float, Config::TARS1::MEDIAN_SAMPLE_NUMBER> massFilter;
    MedianFilter<float, Config::TARS1::MEDIAN_SAMPLE_NUMBER> pressureFilter;

    miosix::FastMutex sampleMutex;
    float massSample     = 0;
    float pressureSample = 0;

    uint16_t delayedEventId = 0;

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("tars1");
};

}  // namespace RIGv2
