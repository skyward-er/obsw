/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Ettore Pane
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

namespace ConRIGv2
{

/**
 * @brief Class that wraps the main task schedulers of the entire OBSW.
 */
class BoardScheduler : public Boardcore::Injectable
{
public:
    BoardScheduler()
        : radio{miosix::PRIORITY_MAX - 1}, buttons{miosix::PRIORITY_MAX - 2}
    {
    }

    [[nodiscard]] bool start()
    {
        if (!radio.start())
        {
            LOG_ERR(logger, "Failed to start radio scheduler");
            return false;
        }

        if (!buttons.start())
        {
            LOG_ERR(logger, "Failed to start buttons scheduler");
            return false;
        }

        return true;
    }

    Boardcore::TaskScheduler& getRadioScheduler() { return radio; }

    Boardcore::TaskScheduler& getButtonsScheduler() { return buttons; }

private:
    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("boardscheduler");

    Boardcore::TaskScheduler radio;
    Boardcore::TaskScheduler buttons;
};
}  // namespace ConRIGv2
