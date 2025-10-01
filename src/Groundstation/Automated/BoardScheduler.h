/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Nicolò Caruso
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

#include <Groundstation/Automated/Config/SchedulerConfig.h>
#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Antennas
{

class BoardScheduler : public Boardcore::Injectable
{
public:
    BoardScheduler()
        : sma{Antennas::Config::Scheduler::SMA_PRIORITY},
          sensors{Antennas::Config::Scheduler::SENSORS_PRIORIY},
          others{Antennas::Config::Scheduler::OTHER_PRIORIY}
    {
    }

    [[nodiscard]] bool start()
    {
        if (!sma.start())
        {
            LOG_ERR(logger, "Failed to start SMA scheduler");
            return false;
        }

        if (!sensors.start())
        {
            LOG_ERR(logger, "Failed to start Sensors scheduler");
            return false;
        }

        if (!others.start())
        {
            LOG_ERR(logger, "Failed to start others scheduler");
            return false;
        }

        started = true;
        return true;
    }

    bool isStarted() { return started; }

    Boardcore::TaskScheduler& getSMAScheduler() { return sma; }

    Boardcore::TaskScheduler& getSensorsScheduler() { return sensors; }

    Boardcore::TaskScheduler& getOthersScheduler() { return others; }

private:
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("boardscheduler");

    std::atomic<bool> started{false};

    Boardcore::TaskScheduler sma;
    Boardcore::TaskScheduler sensors;
    Boardcore::TaskScheduler others;
};

}  // namespace Antennas
