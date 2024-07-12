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

#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{
class BoardScheduler;
class NASController;

class AltitudeTrigger
    : public Boardcore::InjectableWithDeps<BoardScheduler, NASController>
{
public:
    explicit AltitudeTrigger();

    /**
     * @brief Adds the update() task to the task scheduler.
     */
    bool start();

    /**
     * @brief Enable the AltitudeTrigger.
     */
    void enable();

    /**
     * @brief Disable the AltitudeTrigger.
     */
    void disable();

    /**
     * @return The status of the AltitudeTrigger
     */
    bool isActive();

    /**
     * @return Set the altitude of the AltitudeTrigger
     */
    void setDeploymentAltitude(float altitude);

private:
    /**
     * @brief Update method that posts a FLIGHT_WING_ALT_PASSED when the correct
     * altitude is reached
     */
    void update();

    bool running;
    int confidence;  ///< Number of times that the algorithm detects to be below
                     ///< the fixed altitude
    float deploymentAltitude;

    miosix::FastMutex mutex;
};

}  // namespace Payload
