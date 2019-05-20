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
#include "DeathStack/CpuData.h"

using namespace DeathStackBoard;
using namespace miosix;

DeathStack* board;

int main()
{
    board = new DeathStack();


    // Log CPU Usage
    CpuUsageData cpu_data;   
    while(1)
    {
        cpu_data.timestamp = miosix::getTick();
        cpu_data.cpu_usage = averageCpuUtilization();

        board->logger->log(cpu_data);

        Thread::sleep(1000);
    }
}