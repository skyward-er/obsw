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

#include <RIGv2/Actuators/Actuators.h>
#include <RIGv2/BoardScheduler.h>
#include <RIGv2/Configs/TARS1Config.h>
#include <RIGv2/Sensors/Sensors.h>
#include <RIGv2/StateMachines/TARS1/TARS1Data.h>
#include <common/MedianFilter.h>
#include <events/FSM.h>
#include <miosix.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv2
{

class TARS1
    : public Boardcore::InjectableWithDeps<BoardScheduler, Sensors, Actuators>,
      public Boardcore::FSM<TARS1>
{
public:
    TARS1();

    [[nodiscard]] bool start();

    bool isRefueling();

private:
    void sample();

    void state_ready(const Boardcore::Event& event);
    void state_refueling(const Boardcore::Event& event);

    void logAction(Tars1ActionType action);
    void logSample(float pressure, float mass);

    Boardcore::Logger& sdLogger   = Boardcore::Logger::getInstance();
    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("tars1");

    float previousMass = 0;
    float currentMass  = 0;

    float previousPressure = 0;
    float currentPressure  = 0;

    int medianSamples = 0;
    MedianFilter<float, Config::TARS1::MEDIAN_SAMPLE_NUMBER> massFilter;
    MedianFilter<float, Config::TARS1::MEDIAN_SAMPLE_NUMBER> pressureFilter;

    miosix::FastMutex sampleMutex;
    float massSample     = 0;
    float pressureSample = 0;

    int massStableCounter       = 0;
    uint16_t nextDelayedEventId = 0;
};

}  // namespace RIGv2
