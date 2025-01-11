/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Matteo Pignataro, Davide Basso
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

#include <Parafoil/Configs/WingConfig.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <atomic>
#include <utils/ModuleManager/ModuleManager.hpp>

namespace Parafoil
{
class BoardScheduler;
class NASController;

class AltitudeTrigger
    : public Boardcore::InjectableWithDeps<BoardScheduler, NASController>
{
public:
    /**
     * @brief Adds the update() task to the task scheduler.
     */
    bool start();

    /**
     * @return If the AltitudeTrigger is started.
     */
    bool isStarted();

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
    bool isEnabled();

    /**
     * @return Set the altitude of the AltitudeTrigger
     */
    void setDeploymentAltitude(Boardcore::Units::Length::Meter altitude);

private:
    // Update method that posts a FLIGHT_WING_ALT_PASSED when the correct
    // altitude is reached
    void update();

    std::atomic<bool> started{false};
    std::atomic<bool> running{false};

    // Number of times that the algorithm detects to be below the fixed
    // altitude
    int confidence = 0;

    std::atomic<float> targetAltitude{
        Config::AltitudeTrigger::DEPLOYMENT_ALTITUDE
            .value<Boardcore::Units::Length::Meter>()};
};

}  // namespace Parafoil
