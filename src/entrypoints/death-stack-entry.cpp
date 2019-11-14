/*
 * Copyright (c) 2019 Skyward Experimental Rocketry
 * Authors: Alvise de' Faveri Tron
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
#include <Common.h>
#include <DeathStack/DeathStack.h>
#include <DeathStack/TMTCManager/TMBuilder.h>
#include <DeathStack/TMTCManager/XbeeInterrupt.h>
#include <diagnostic/CpuMeter.h>
#include <math/Stats.h>
#include "DeathStack/System/StackLogger.h"
#include "DeathStack/System/SystemData.h"
#include "DeathStack/events/EventInjector.h"

using namespace DeathStackBoard;
using namespace miosix;

DeathStack* board;

StatsResult cpu_stat_res;


D(EventInjector debug_console;)

int main()
{
    D(debug_console.start());

    Stats cpu_stat;
    board = DeathStack::getInstance();

    // Log CPU Usage
    SystemData system_data;
    while (1)
    {
        StackLogger::getInstance()->updateStack(THID_ENTRYPOINT);

        system_data.timestamp = miosix::getTick();
        system_data.cpu_usage = averageCpuUtilization();
        cpu_stat.add(system_data.cpu_usage);

        cpu_stat_res               = cpu_stat.getStats();
        system_data.cpu_usage_min  = cpu_stat_res.minValue;
        system_data.cpu_usage_max  = cpu_stat_res.maxValue;
        system_data.cpu_usage_mean = cpu_stat_res.mean;

        system_data.min_free_heap = MemoryProfiling::getAbsoluteFreeHeap();
        system_data.free_heap     = MemoryProfiling::getCurrentFreeHeap();

        board->logger->log(system_data);

        // Log logger stats
        LogStats stats  = Logger::instance().getLogStats();
        stats.timestamp = miosix::getTick();
        board->logger->log(stats);

        // Log threads stack data
        StackLogger::getInstance()->log();
        // printf("CPU: %.2f\n", system_data.cpu_usage);
        Thread::sleep(1000);
    }
}