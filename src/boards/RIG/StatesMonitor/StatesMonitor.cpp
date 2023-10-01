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

#include <RIG/StatesMonitor/StatesMonitor.h>

using namespace Boardcore;
using namespace miosix;

namespace RIG
{
StatesMonitor::StatesMonitor(TaskScheduler* sched) : scheduler(sched)
{
    // Init the arrays
    for (int i = 0; i < Config::StatesMonitor::BOARDS_NUMBER; i++)
    {
        updateTimestamps[i] = 0;
        boardStatuses[i]    = 0;
    }
}

bool StatesMonitor::start()
{
    // Add the task to the scheduler
    uint8_t result = scheduler->addTask([=]() { this->update(); },
                                        Config::StatesMonitor::UPDATE_PERIOD);
    return result != 0;
}

void StatesMonitor::update()
{
    Lock<FastMutex> lock(mutex);

    for (int i = 0; i < Config::StatesMonitor::BOARDS_NUMBER; i++)
    {
        // Check if the time since the last config expires
        if (getTick() >
            updateTimestamps[i] + Config::StatesMonitor::MAX_TIMEOUT)
        {
            boardStatuses[i] = 0;
        }
    }
}

void StatesMonitor::setBoardStatus(Common::CanConfig::Board board,
                                   uint8_t status)
{
    Lock<FastMutex> lock(mutex);
    uint8_t index = static_cast<uint8_t>(board);

    // Check the validity of the board index
    if (index < Config::StatesMonitor::BOARDS_NUMBER)
    {
        boardStatuses[index]    = status;
        updateTimestamps[index] = getTick();
    }
}

uint8_t StatesMonitor::getStatus(Common::CanConfig::Board board)
{
    Lock<FastMutex> lock(mutex);
    uint8_t index = static_cast<uint8_t>(board);

    if (index < Config::StatesMonitor::BOARDS_NUMBER)
    {
        return boardStatuses[index];
    }
    return 0;
}

}  // namespace RIG