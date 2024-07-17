/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <scheduler/TaskScheduler.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{

/**
 * @brief The class that manages the task schedulers of the board.
 * It takes care of handing out task schedulers to modules.
 */
class BoardScheduler : public Boardcore::Injectable
{
public:
    Boardcore::TaskScheduler& nasController() { return critical; }
    Boardcore::TaskScheduler& sensors() { return high; }
    Boardcore::TaskScheduler& pinHandler() { return high; }
    Boardcore::TaskScheduler& radio() { return medium; }
    Boardcore::TaskScheduler& canHandler() { return medium; }
    Boardcore::TaskScheduler& altitudeTrigger() { return medium; }
    Boardcore::TaskScheduler& wingController() { return medium; }
    Boardcore::TaskScheduler& verticalVelocityTrigger() { return medium; }
    Boardcore::TaskScheduler& windEstimation() { return medium; }
    Boardcore::TaskScheduler& actuators() { return low; }
    Boardcore::TaskScheduler& flightStatsRecorder() { return low; }

    /**
     * @brief Starts all the schedulers
     */
    [[nodiscard]] bool start()
    {
        return critical.start() && high.start() && medium.start() &&
               low.start();
    }

    /**
     * @brief Returns if all the schedulers are up and running
     */
    bool isStarted()
    {
        return critical.isRunning() && high.isRunning() && medium.isRunning() &&
               low.isRunning();
    }

private:
    Boardcore::TaskScheduler critical{miosix::PRIORITY_MAX - 1};
    Boardcore::TaskScheduler high{miosix::PRIORITY_MAX - 2};
    Boardcore::TaskScheduler medium{miosix::PRIORITY_MAX - 3};
    Boardcore::TaskScheduler low{miosix::PRIORITY_MAX - 4};
};

}  // namespace Payload
