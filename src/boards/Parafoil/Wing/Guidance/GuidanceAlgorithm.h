/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Radu Raul
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

#include <Eigen/Core>

namespace Parafoil
{

/**
 * This class acts as an interface for a generic guidance algorithm that is used
 * by the Automatic Wing Algorithm.
 */
class GuidanceAlgorithm
{
public:
    /**
     * @brief This method must calculate the yaw angle of the parafoil knowing
     * the current position and the target position without changing the vectors
     * passed as arguments.
     *
     * @note the args are const references to reduce access time by avoiding
     * copying objects that will be read-only.
     *
     * @param[in] position the current NED position of the parafoil in [m]
     * @param[in] target NED position of the target in [m]
     * @param[out] heading The current heading, it is used for logging purposes
     *
     * @returns the yaw angle of the parafoil in [rad]
     */
    virtual float calculateTargetAngle(
        const Eigen::Vector3f& currentPositionNED,
        Eigen::Vector2f& heading) = 0;

    Eigen::Vector2f getTargetNED() { return targetNED; }

protected:
    Eigen::Vector2f targetNED{0, 0};  // NED
};

}  // namespace Parafoil
