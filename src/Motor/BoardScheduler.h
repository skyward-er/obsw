/* Copyright (c) 2025 Skyward Experimental Rocketry
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

namespace Motor
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

    Boardcore::TaskScheduler& sensors() { return high; }
    Boardcore::TaskScheduler& canHandler() { return medium; }

    static Priority::PriorityLevel actuatorsPriority()
    {
        return Priority::CRITICAL;  // Max priority for valve control
    }

    static Priority::PriorityLevel canHandlerPriority()
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

    std::atomic<bool> started{false};

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("BoardScheduler");
};

}  // namespace Motor
