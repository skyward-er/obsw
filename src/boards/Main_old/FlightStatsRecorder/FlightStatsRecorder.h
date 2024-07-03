/* Copyright (c) 2019-2023 Skyward Experimental Rocketry
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

#include <Main/StateMachines/FlightModeManager/FlightModeManager.h>
#include <algorithms/ADA/ADAData.h>
#include <algorithms/NAS/NASState.h>
#include <common/MavlinkGemini.h>
#include <scheduler/TaskScheduler.h>
#include <sensors/SensorData.h>
#include <sensors/analog/Pitot/PitotData.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class FlightStatsRecorder : public Boardcore::Module
{
public:
    FlightStatsRecorder(Boardcore::TaskScheduler* sched);

    /**
     * @brief Adds a task to the scheduler to update the stats
     */
    bool start();

    /**
     * @brief Update method that gathers all the info to update the data
     * structure
     */
    void update();

    /**
     * @brief Gets the packet already populated
     */
    mavlink_rocket_stats_tm_t getStats();

private:
    // Update methods
    void updateAcc(FlightModeManagerState flightState,
                   Boardcore::AccelerometerData data);
    void updateBaro(FlightModeManagerState flightState,
                    Boardcore::PressureData data);
    void updateDPLPressure(FlightModeManagerState flightState,
                           Boardcore::PressureData data);
    void updatePitot(FlightModeManagerState flightState,
                     Boardcore::PitotData data);
    void updateNAS(FlightModeManagerState flightState,
                   Boardcore::NASState data);
    void updateADA(FlightModeManagerState flightState,
                   Boardcore::ADAState data);

    // Update scheduler to update all the data
    Boardcore::TaskScheduler* scheduler = nullptr;

    // Data structure
    mavlink_rocket_stats_tm_t stats;

    // Update mutex
    miosix::FastMutex mutex;
};
}  // namespace Main