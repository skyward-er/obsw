/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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

#include <RIG/Configs/TARSConfig.h>
#include <RIG/StateMachines/TARS1/MedianFilter.h>
#include <RIG/StateMachines/TARS1/TARS1Data.h>
#include <common/Events.h>
#include <common/Topics.h>
#include <events/EventBroker.h>
#include <events/FSM.h>
#include <scheduler/TaskScheduler.h>
#include <utils/Stats/Stats.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
class TARS1 : public Boardcore::Module, public Boardcore::FSM<TARS1>
{
public:
    TARS1(Boardcore::TaskScheduler* sched);

    /**
     * @brief Overrides Active Object's method to add also the task to the task
     * scheduler to sample the sensors and filter them
     */
    [[nodiscard]] bool start() override;

    // FSM states
    void state_idle(const Boardcore::Event& event);
    void state_ready(const Boardcore::Event& event);
    void state_washing(const Boardcore::Event& event);
    void state_refueling(const Boardcore::Event& event);

    // Status getter
    TARS1Status getStatus();

private:
    /**
     * @brief Logs the current FSM status on SD
     */
    void logStatus(TARS1State state);

    /**
     * @brief Samples the sensors and computes an average
     */
    void sample();

    /**
     * @brief Evaluates if the algorithm reached its maximum capabilities in
     * filling the tank
     */
    bool isMaximumMass();

    /**
     * @brief Evaluates if the pressure inside the tank is at equilibrium
     */
    bool isPressureStable();

    // Data structure output of sampling
    struct
    {
        float pressure = 0;
        float mass     = 0;
    } samples;

    // Data structure for filtering purposes
    struct
    {
        MedianFilter<float, Config::TARS::TARS_FILTER_SAMPLE_NUMBER> pressure;
        MedianFilter<float, Config::TARS::TARS_FILTER_SAMPLE_NUMBER> mass;
    } filteredData;

    // Registered mass after venting
    float massAtVenting         = 0;
    float previousMassAtVenting = 0;

    // Registered pressure before checking
    float previousPressure = 0;

    // Number of times the mass was stable
    uint16_t massStableCounter = 0;

    // Counts the number of samples done for the average measure
    uint16_t sampleCounter = 0;

    // Current status
    TARS1Status status;

    // Dedicated scheduler for sampling sensors periodically
    Boardcore::TaskScheduler* scheduler = nullptr;

    // Mutex to synchronize sampling and reading
    miosix::FastMutex mutex;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("TARS1");
};
}  // namespace RIG