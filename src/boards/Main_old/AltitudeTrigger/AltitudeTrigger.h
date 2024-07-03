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

#include <diagnostic/PrintLogger.h>
#include <scheduler/TaskScheduler.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Main
{
class AltitudeTrigger : public Boardcore::Module
{
public:
    AltitudeTrigger(Boardcore::TaskScheduler* sched);

    /**
     * @brief Adds to the task scheduler the periodic update task
     */
    bool start();

    /**
     * @brief Set the Deployment Altitude that must be reached to open the Main
     * parachute
     */
    void setDeploymentAltitude(float alt);

    /**
     * @brief The update function that is called every period to check the FMM
     * state and the altitude reached
     */
    void update();

private:
    /**
     * @brief Reference that must be reached
     */
    std::atomic<float> altitude;

    /**
     * @brief Counts how many times the altitude has been reached consequently.
     */
    uint32_t confidence;

    /**
     * @brief Scheduler to which add the periodic update task
     */
    Boardcore::TaskScheduler* scheduler = nullptr;

    Boardcore::PrintLogger logger =
        Boardcore::Logging::getLogger("AltitudeTrigger");
};
}  // namespace Main