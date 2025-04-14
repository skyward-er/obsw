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

#include <RIGv2/Configs/SchedulerConfig.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace RIGv2
{

class BoardScheduler : public Boardcore::Injectable
{
public:
    BoardScheduler()
        : tars(Config::Scheduler::TARS_PRIORITY),
          sensors(Config::Scheduler::SENSORS_PRIORITY)
    {
    }

    [[nodiscard]] bool start()
    {
        if (!tars.start())
        {
            LOG_ERR(logger, "Failed to start TARS scheduler");
            return false;
        }

        if (!sensors.start())
        {
            LOG_ERR(logger, "Failed to start sensors scheduler");
            return false;
        }

        started = true;
        return true;
    }

    bool isStarted() { return started; }

    Boardcore::TaskScheduler& getTars1Scheduler() { return tars; }

    Boardcore::TaskScheduler& getTars3Scheduler() { return tars; }

    Boardcore::TaskScheduler& getSensorsScheduler() { return sensors; }

    Boardcore::TaskScheduler& getActuatorsScheduler() { return sensors; }

    Boardcore::TaskScheduler& getCanBusScheduler() { return sensors; }

private:
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("boardscheduler");

    std::atomic<bool> started{false};

    Boardcore::TaskScheduler tars;
    Boardcore::TaskScheduler sensors;
};

}  // namespace RIGv2
