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
        : tars1(Config::Scheduler::TARS1_PRIORITY),
          sensors(Config::Scheduler::SENSORS_PRIORITY)
    {
    }

    [[nodiscard]] bool start() { return tars1.start() && sensors.start(); }

    Boardcore::TaskScheduler &getTars1Scheduler() { return tars1; }

    Boardcore::TaskScheduler &getSensorsScheduler() { return sensors; }

    Boardcore::TaskScheduler &getActuatorsScheduler() { return sensors; }

private:
    Boardcore::TaskScheduler tars1;
    Boardcore::TaskScheduler sensors;
};

}  // namespace RIGv2