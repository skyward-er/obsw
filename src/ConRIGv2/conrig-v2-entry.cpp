/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Authors: Ettore Pane, Niccolò Betto
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

#include <ConRIGv2/BoardScheduler.h>
#include <ConRIGv2/Buses.h>
#include <ConRIGv2/Buttons/Buttons.h>
#include <ConRIGv2/Hub/Hub.h>
#include <ConRIGv2/Radio/Radio.h>
#include <interfaces-impl/hwmapping.h>
#include <miosix.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace ConRIGv2;

int main()
{
    bool initResult = true;

    DependencyManager manager;

    auto buses     = new Buses();
    auto scheduler = new BoardScheduler();
    auto radio     = new Radio();
    auto buttons   = new Buttons();
    auto hub       = new Hub();

    initResult &= manager.insert<BoardScheduler>(scheduler) &&
                  manager.insert<Buses>(buses) &&
                  manager.insert<Radio>(radio) && manager.insert<Hub>(hub) &&
                  manager.insert<Buttons>(buttons) && manager.inject();

    if (!initResult)
    {
        std::cerr << "*** Failed to inject dependencies ***" << std::endl;
        return -1;
    }

    // Status led indicators
    // led1: Buttons init/error
    // led2: Radio init/error
    // led3: Hub init/error
    // led4: Everything ok

    std::cout << "Starting Buttons" << std::endl;
    led1On();
    if (!buttons->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Buttons ***" << std::endl;
    }
    else
    {
        led1Off();
    }

    std::cout << "Starting Radio" << std::endl;
    led2On();
    if (!radio->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Radio ***" << std::endl;
    }
    else
    {
        led2Off();
    }

    std::cout << "Starting Hub" << std::endl;
    led3On();
    if (!hub->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start Hub ***" << std::endl;
    }
    else
    {
        led3Off();
    }

    std::cout << "Starting BoardScheduler" << std::endl;
    if (!scheduler->start())
    {
        initResult = false;
        std::cerr << "*** Failed to start BoardScheduler ***" << std::endl;
    }

    if (initResult)
    {
        std::cout << "*** All good! ***" << std::endl;
        led4On();
    }
    else
    {
        std::cerr << "*** Init failure ***" << std::endl;
    }

    while (true)
        Thread::wait();
}
