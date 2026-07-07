/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Raul Radu
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

#include <Parafoil/Wing/WingAlgorithm.h>
#include <algorithms/ReferenceValues.h>
#include <algorithms/WingController/WingControllerData.h>
#include <algorithms/WingController/wingController.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <Eigen/Core>

namespace Parafoil
{
class Sensors;
class NASController;
class Actuators;

class AutocodedWingAlgorithm : public Boardcore::InjectableWithDeps<
                                   Boardcore::InjectableBase<WingAlgorithm>,
                                   Sensors, NASController, Actuators>
{
public:
    /**
     * @brief Construct a new Autocoded Automatic Wing Algorithm object
     *
     * @param servoLeft The left servo
     * @param servoRight The right servo
     */
    AutocodedWingAlgorithm(ServosList servoLeft, ServosList servoRight);

protected:
    /**
     * @brief This method updates the autocoded algoritm and sets the servos
     * position accordingly
     */
    void step() override;

    bool init() override;

    wingController controller;  ///< The autocoded wing controller

    // Logging structure
    WingAlgorithmData data;

    /**
     * @brief Mutex
     */
    miosix::FastMutex mutex;
};
}  // namespace Parafoil
