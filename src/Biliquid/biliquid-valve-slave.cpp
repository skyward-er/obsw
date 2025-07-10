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
#include <Biliquid/PinObserver/PinObserver.h>
#include <Biliquid/hwmapping.h>
#include <events/EventBroker.h>
#include <fmt/format.h>
#include <scheduler/TaskScheduler.h>
#include <utils/PinObserver/PinObserver.h>

using namespace miosix;
using namespace Boardcore;
using namespace Biliquid;

int main()
{
    fmt::print("Initializing hardware...\n");
    // Since this is a temporary board, we didn't bother creating a proper BSP
    hwmapping::init();

    fmt::print("Initializing Event Broker...\n");
    auto& broker = EventBroker::getInstance();
    broker.start();

    fmt::print("Initializing Actuators...\n");
    auto actuators = new Actuators();
    actuators->start();

    fmt::print("Initializing Sequence Manager...\n");
    auto manager = new SequenceManager(*actuators);
    manager->start();

    fmt::print("Initializing Pin Observer...\n");
    auto pinScheduler = new TaskScheduler(PRIORITY_MAX - 3);
    auto pinObserver  = new PinObserver(*pinScheduler, 20);
    Pins::registerPins(*pinObserver);
    pinScheduler->start();

    fmt::print("Initialization OK!\n");

    while (true)
    {
        hwmapping::StatusLed::high();
        miosix::Thread::sleep(1000);
        hwmapping::StatusLed::low();
        miosix::Thread::sleep(1000);
    }

    return 0;
}
