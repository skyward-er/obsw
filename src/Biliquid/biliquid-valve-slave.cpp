/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <Biliquid/Actuators/Actuators.h>
#include <Biliquid/Control/SequenceManager.h>
#include <Biliquid/hwmapping.h>

#include <iostream>

using namespace miosix;
using namespace Boardcore;
using namespace Biliquid;

int main()
{
    std::cout << "Initializing hardware" << std::endl;
    // Since this is a temporary board, we didn't bother creating a proper BSP
    hwmapping::init();

    std::cout << "Initializing actuators" << std::endl;
    Actuators* actuators = new Actuators();
    actuators->start();

    std::cout << "Initializing sequence manager" << std::endl;
    SequenceManager* manager = new SequenceManager(*actuators);
    (void)manager;  // Prevent unused variable warning

    std::cout << "Initialization OK!" << std::endl;

    while (true)
    {
        hwmapping::StatusLed::high();
        miosix::Thread::sleep(1000);
        hwmapping::StatusLed::low();
        miosix::Thread::sleep(1000);
    }

    return 0;
}
