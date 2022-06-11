/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Erbetta
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

#include <DeathStack.h>
#include <FlightStatsRecorder/FSRController.h>
#include <diagnostic/CpuMeter.h>
#include <math/Stats.h>
#include <miosix.h>

using namespace miosix;
using namespace Boardcore;
using namespace DeathStackBoard;
// using namespace GlobalBuffers;

int main()
{
    PrintLogger log = Logging::getLogger("main");
#ifndef DEBUG  // if not debugging, output to file only INFO level or higher
    unique_ptr<LogSink> log_sink = std::make_unique<FileLogSinkBuffered>();
    log_sink->setLevel(LOGL_INFO);
    Logging::addLogSink(log_sink);
#endif

    Stats cpu_stat;
    StatsResult cpu_stat_res;
    SystemData system_data;

    // Instantiate the stack
    DeathStack::getInstance().start();
    LOG_INFO(log, "Death stack started");

    LoggerService& logger_service = LoggerService::getInstance();

    for (;;)
    {
        Thread::sleep(1000);
        logger_service.log(logger_service.getLogger().getLoggerStats());

        StackLogger::getInstance().updateStack(THID_ENTRYPOINT);

        system_data.timestamp = getTick();
        system_data.cpu_usage = getCpuStats();
        cpu_stat.add(system_data.cpu_usage);

        cpu_stat_res               = cpu_stat.getStats();
        system_data.cpu_usage_min  = cpu_stat_res.minValue;
        system_data.cpu_usage_max  = cpu_stat_res.maxValue;
        system_data.cpu_usage_mean = cpu_stat_res.mean;

        system_data.min_free_heap = MemoryProfiling::getAbsoluteFreeHeap();
        system_data.free_heap     = MemoryProfiling::getCurrentFreeHeap();

        logger_service.log(system_data);

        StackLogger::getInstance().log();
    }
}
