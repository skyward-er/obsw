/**
 * Copyright (c) 2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta (luca.erbetta@skywarder.eu)
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

#include <miosix.h>

#include "DeathStack.h"
// #include <diagnostic/PrintLogger.h>
#include <Debug.h>

#include "math/Stats.h"
#include <diagnostic/CpuMeter.h>

using namespace miosix;
using namespace DeathStackBoard;
// using namespace GlobalBuffers;

int main()
{
    // Logging::startAsyncLogger();
    // PrintLogger log = Logging::getLogger("main");

    Stats cpu_stat;

    // LOG_INFO(log, "Starting death stack...");
    TRACE("Starting death stack...\n");
    // Instantiate the stack
    Thread::sleep(1000);
    DeathStack::getInstance()->start();
    // LOG_INFO(log, "Death stack started");
    TRACE("Death stack started\n");

    for (;;)
    {
        Thread::sleep(1000);
        LoggerService::getInstance()->log(
            LoggerService::getInstance()->getLogger().getLogStats());
            
        cpu_stat.add(averageCpuUtilization());

        /*printf("CPU : avg: %.2f   max: %.2f   min: %.2f \n",
               cpu_stat.getStats().mean, cpu_stat.getStats().maxValue,
               cpu_stat.getStats().minValue);*/

        Thread::sleep(200);
    }
}