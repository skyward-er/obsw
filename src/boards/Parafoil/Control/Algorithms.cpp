/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Parafoil/Control/Algorithms.h>
#include <Parafoil/ParafoilTest.h>

using namespace Boardcore;
using namespace std;

namespace Parafoil
{
Algorithms::Algorithms(TaskScheduler* scheduler)
{
    this->scheduler = scheduler;

    // Init the algorithms
    NASInit();
}

Algorithms::~Algorithms()
{
    // Destroy all the algorithms
    delete nas;
}

bool Algorithms::start()
{
    // Start the scheduler
    return nas->start();
}

void Algorithms::NASInit()
{
    // Create the nas
    nas = new NASController(scheduler);

    // Init
    nas->init();
}

/**
 * GETTERS
 */
NASState Algorithms::getNASLastSample()
{
    // Pause the kernel for sync purposes
    miosix::PauseKernelLock lock;
    return nas->getLastSample();
}
}  // namespace Parafoil