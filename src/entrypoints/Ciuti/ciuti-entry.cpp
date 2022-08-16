/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Alberto Nidasio
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

#include <Ciuti/BoardScheduler.h>
#include <Ciuti/Sensors/Sensors.h>
#include <diagnostic/CpuMeter/CpuMeter.h>

#include <thread>

using namespace miosix;
using namespace Boardcore;
using namespace Ciuti;

void print()
{
    auto ch0 =
        Sensors::getInstance().getInternalADCLastSample(InternalADC::CH0);
    auto ch1 =
        Sensors::getInstance().getInternalADCLastSample(InternalADC::CH1);

    printf("[%.2f] CH0: %.6f, CH1: %.6f, log number: %d\n",
           ch0.voltageTimestamp / 1e6, ch0.voltage, ch1.voltage,
           Logger::getInstance().getCurrentLogNumber());
}

int main()
{
    Logger::getInstance().start();
    Sensors::getInstance().start();

    BoardScheduler::getInstance().getScheduler().addTask(print, 100);
    BoardScheduler::getInstance().getScheduler().start();

    // Periodical statistics
    while (true)
    {
        Thread::sleep(1000);
        Logger::getInstance().log(CpuMeter::getCpuStats());
        CpuMeter::resetCpuStats();
        Logger::getInstance().logStats();
    }
}
