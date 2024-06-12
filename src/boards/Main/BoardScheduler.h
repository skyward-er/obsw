/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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

#include <Main/Configs/SchedulerConfig.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{

class BoardScheduler : public Boardcore::Module
{
public:
    BoardScheduler()
        : nas(Main::Config::Scheduler::NAS_PRIORITY),
          ada(Main::Config::Scheduler::ADA_PRIORITY),
          sensors(Main::Config::Scheduler::SENSORS_PRIORITY),
          others(Main::Config::Scheduler::OTHERS_PRIORITY)
    {
    }

    [[nodiscard]] bool start()
    {
        return nas.start() && ada.start() && sensors.start() && others.start();
    }

    Boardcore::TaskScheduler &getNasScheduler() { return nas; }

    Boardcore::TaskScheduler &getAdaScheduler() { return ada; }

    Boardcore::TaskScheduler &getSensorsScheduler() { return sensors; }

    Boardcore::TaskScheduler &getPinObserverScheduler()
    {
        // TODO(davide.mor): Does this make sense?
        return sensors;
    }

    Boardcore::TaskScheduler &getRadioScheduler() { return others; }

    Boardcore::TaskScheduler &getCanBusScheduler() { return others; }

    Boardcore::TaskScheduler &getLowPriorityActuatorsScheduler()
    {
        return others;
    }

private:
    Boardcore::TaskScheduler nas;
    Boardcore::TaskScheduler ada;
    Boardcore::TaskScheduler sensors;
    Boardcore::TaskScheduler others;
};

}  // namespace Main