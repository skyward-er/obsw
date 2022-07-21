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
#pragma once

#include <Payload/Control/Algorithms.h>
#include <Payload/Payload.h>

using namespace Boardcore;
using namespace std;

namespace Payload
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
    return scheduler->start();
}

void Algorithms::NASInit()
{
    nas = new NASController<BMX160Data, UBXGPSData>(
        bind(&Sensors::getImuBMX160LastSample,
             Payload::Payload::getInstance().sensors),
        bind(&Sensors::getGPSLastSample,
             Payload::Payload::getInstance().sensors),
        scheduler);

    // TODO should we set the initial orientation and position?
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
}  // namespace Payload