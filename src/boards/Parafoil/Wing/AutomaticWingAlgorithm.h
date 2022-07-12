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

#include <Parafoil/Wing/WingAlgorithm.h>
#include <algorithms/PIController.h>

#include <Eigen/Core>

namespace Parafoil
{
class AutomaticWingAlgorithm : public WingAlgorithm
{
public:
    /**
     * @brief Construct a new Automatic Wing Algorithm object
     *
     * @param Kp Proportional value for PI controller
     * @param Ki Integral value for PI controller
     * @param servo1 The first servo
     * @param servo2 The second servo
     */
    AutomaticWingAlgorithm(float Kp, float Ki, ServoInterface* servo1,
                           ServoInterface* servo2);

    /**
     * @brief Construct a new Automatic Wing Algorithm object
     *
     * @param Kp Proportional value for PI controller
     * @param Ki Integral value for PI controller
     */
    AutomaticWingAlgorithm(float Kp, float Ki);

    /**
     * @brief Destroy the Automatic Wing Algorithm object and the PI
     */
    ~AutomaticWingAlgorithm();

protected:
    // PI controller tuned on the Kp and Ki passed through constructor
    Boardcore::PIController* controller;

    /**
     * @brief This method implements the automatic algorithm that will steer the
     * parafoil according to its position and velocity. IN THIS METHOD THE
     * GUIDANCE IS TRANSLATED
     */
    void step() override;
};
}  // namespace Parafoil