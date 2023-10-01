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

#pragma once

#include <RIG/Configs/StatesMonitorConfig.h>
#include <common/CanConfig.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace RIG
{
/**
 * @brief The class monitors and updates the states of every board.
 * The CAN module when receives a state update, sets the current state into this
 * module. Every N seconds, the module checks if the states have been updated,
 * and if not resets them.
 */
class StatesMonitor : public Boardcore::Module
{
public:
    StatesMonitor(Boardcore::TaskScheduler* sched);

    /**
     * @brief Starts the update function
     */
    bool start();

    /**
     * @brief Update function which checks periodically the last time in which
     * the single states where updated. Eventually if the last update is greater
     * than N seconds ago the state is set to 0.
     */
    void update();

    /**
     * @brief Sets the passed Board to a specific status
     */
    void setBoardStatus(Common::CanConfig::Board board, uint8_t status);

    /**
     * @brief Get the status of a specific Board
     */
    uint8_t getStatus(Common::CanConfig::Board board);

private:
    // Board info
    long long int updateTimestamps[Config::StatesMonitor::BOARDS_NUMBER];
    uint8_t boardStatuses[Config::StatesMonitor::BOARDS_NUMBER];

    // Sync mutex
    miosix::FastMutex mutex;

    Boardcore::TaskScheduler* scheduler = nullptr;
};
}  // namespace RIG