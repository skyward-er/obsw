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
// #include "Main/GlobalBuffers.h"

using namespace miosix;
using namespace DeathStackBoard;
// using namespace GlobalBuffers;

int main()
{
    TRACE("Starting windtunnel test....\n");
    // Instantiate the stack
    DeathStack::getInstance()->start();
    TRACE("Running\n");

    // bool printed = false;
    for (;;)
    {
        Thread::sleep(1000);
        LoggerService::getInstance()->log(
            LoggerService::getInstance()->getLogger().getLogStats());
        // if(buf_adc_pitot.size() >= GLOBAL_BUF_LEN && !printed)
        // {
        //     printed = true;
        //     printf("timestamp,voltage_pitot\n");
        //     for(auto v : buf_adc_pitot)
        //     {
        //         printf("%llu,%f\n", v.adc_timestamp, v.voltage);
        //     }
        // }
    }
}
