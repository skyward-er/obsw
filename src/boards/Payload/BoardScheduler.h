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

#include <diagnostic/PrintLogger.h>
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
    /**
     * @brief Enclosing struct to avoid polluting the BoardScheduler namespace.
     */
    struct Priority
    {
        /**
         * @brief Priority levels for the board schedulers.
         */
        enum PriorityLevel
        {
            LOW      = miosix::PRIORITY_MAX - 4,
            MEDIUM   = miosix::PRIORITY_MAX - 3,
            HIGH     = miosix::PRIORITY_MAX - 2,
            CRITICAL = miosix::PRIORITY_MAX - 1,
        };
    };

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

    static Priority::PriorityLevel flightModeManagerPriority()
    {
        return Priority::MEDIUM;
    }

    static Priority::PriorityLevel nasControllerPriority()
    {
        return Priority::MEDIUM;
    }

    static Priority::PriorityLevel wingControllerPriority()
    {
        return Priority::MEDIUM;
    }

    /**
     * @brief Starts all the schedulers
     */
    [[nodiscard]] bool start()
    {
        if (!critical.start())
        {
            LOG_ERR(logger, "Critical priority scheduler failed to start");
            return false;
        }

        if (!high.start())
        {
            LOG_ERR(logger, "High priority scheduler failed to start");
            return false;
        }

        if (!medium.start())
        {
            LOG_ERR(logger, "Medium priority scheduler failed to start");
            return false;
        }

        if (!low.start())
        {
            LOG_ERR(logger, "Low priority scheduler failed to start");
            return false;
        }

        started = true;
        return true;
    }

    /**
     * @brief Returns if all the schedulers are up and running
     */
    bool isStarted() { return started; }

private:
    Boardcore::TaskScheduler critical{Priority::CRITICAL};
    Boardcore::TaskScheduler high{Priority::HIGH};
    Boardcore::TaskScheduler medium{Priority::MEDIUM};
    Boardcore::TaskScheduler low{Priority::LOW};

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("BoardScheduler");
};

}  // namespace Payload
