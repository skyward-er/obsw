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
#include "BoardScheduler.h"

using namespace Boardcore;

namespace con_RIG
{
// TODO: UPDATE THE SCHEDULER PRIORITY PARAMETER ONCE MERGED NEW TASK SCHEDULER
BoardScheduler::BoardScheduler()
{
    scheduler1 = new TaskScheduler();
    scheduler2 = new TaskScheduler();
    scheduler3 = new TaskScheduler();
    scheduler4 = new TaskScheduler();
}

TaskScheduler* BoardScheduler::getScheduler(miosix::Priority priority)
{
    switch (priority.get())
    {
        case miosix::PRIORITY_MAX:
            return scheduler4;
        case miosix::PRIORITY_MAX - 1:
            return scheduler3;
        case miosix::PRIORITY_MAX - 2:
            return scheduler2;
        case miosix::MAIN_PRIORITY:
            return scheduler1;
        default:
            return scheduler1;
    }
}

bool BoardScheduler::start()
{
    return scheduler1->start() && scheduler2->start() && scheduler3->start() &&
           scheduler4->start();
}

bool BoardScheduler::isStarted()
{
    return scheduler1->isRunning() && scheduler2->isRunning() &&
           scheduler3->isRunning() && scheduler4->isRunning();
}
}