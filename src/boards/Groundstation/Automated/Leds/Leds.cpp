/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Federico Lolli
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

#include "Leds.h"

#include <miosix.h>

using namespace miosix;
using namespace Boardcore;

namespace Antennas
{

bool Leds::start()
{
    // turn off all leds
    ledOff();

    if (!ActiveObject::start())
    {
        return false;
    }

    return true;
}

void Leds::run()
{
    while (!shouldStop())
    {
        // we avoid using mutex to avoid waiting for the release of it
        // data races here are not a problem
        if (blinking_red)
        {
            userLed4::high();
            Thread::sleep(100);
            userLed4::low();
            Thread::sleep(100);
        }
        else if (yellow_blink)
        {
            led2On();
            Thread::sleep(50);
            led2Off();
            Thread::sleep(50);
        }
        else if (orange_blinking)
        {
            led3On();
            Thread::sleep(50);
            led3Off();
            Thread::sleep(50);
        }
    }
}

void Leds::errorLoop()
{
    while (true)
    {
        userLed4::high();
        Thread::sleep(100);
        userLed4::low();
        Thread::sleep(100);
    }
}

}  // namespace Antennas
