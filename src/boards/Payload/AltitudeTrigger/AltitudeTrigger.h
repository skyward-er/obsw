/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Payload
{

class AltitudeTrigger : public Boardcore::Module
{
public:
    explicit AltitudeTrigger(TaskScheduler *sched);

    bool start();

    void enable();

    void disable();

    bool isActive();

    void setDeploymentAltitude(float altitude);

private:
    // Update method that posts a FLIGHT_WING_ALT_PASSED when the correct
    // altitude is reached
    void update();

    std::atomic<bool> running;

    // Number of times that the algorithm detects to be below the fixed
    // altitude
    int confidence;

    float deploymentAltitude;

    miosix::FastMutex mutex;

    Boardcore::TaskScheduler *scheduler = nullptr;
};

}  // namespace Payload
