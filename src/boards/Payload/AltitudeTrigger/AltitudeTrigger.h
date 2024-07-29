/* Copyright (c) 2023-2024 Skyward Experimental Rocketry
 * Authors: Federico Mandelli, Nicclò Betto
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

#include <Payload/Configs/WingConfig.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{
class BoardScheduler;
class NASController;

class AltitudeTrigger
    : public Boardcore::InjectableWithDeps<BoardScheduler, NASController>
{
public:
    bool start();
    bool isStarted();

    void enable();
    void disable();
    bool isEnabled();

    /**
     * @return Set the deployment altitude.
     */
    void setDeploymentAltitude(float altitude);

private:
    /**
     * @brief Update method that posts a FLIGHT_WING_ALT_PASSED when the correct
     * altitude is reached
     */
    void update();

    std::atomic<bool> started{false};
    std::atomic<bool> running{false};

    std::atomic<float> targetAltitude{
        Config::AltitudeTrigger::DEPLOYMENT_ALTITUDE};

    int confidence = 0;  ///< Number of consecutive readings that are below the
                         ///< target altitude
};

}  // namespace Payload
