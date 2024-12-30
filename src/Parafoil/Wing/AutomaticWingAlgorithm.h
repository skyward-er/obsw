/* Copyright (c) 2024 Skyward Experimental Rocketry
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

#include <Parafoil/Wing/Guidance/GuidanceAlgorithm.h>
#include <Parafoil/Wing/WingAlgorithm.h>
#include <algorithms/NAS/NASState.h>
#include <algorithms/PIController.h>
#include <algorithms/ReferenceValues.h>
#include <units/Angle.h>
#include <utils/DependencyManager/DependencyManager.h>

#include <Eigen/Core>

namespace Parafoil
{

class Sensors;
class NASController;
class Actuators;

class AutomaticWingAlgorithm : public Boardcore::InjectableWithDeps<
                                   Boardcore::InjectableBase<WingAlgorithm>,
                                   Sensors, NASController, Actuators>
{
public:
    /**
     * @brief Construct a new Automatic Wing Algorithm object
     *
     * @param Kp Proportional value for PI controller
     * @param Ki Integral value for PI controller
     * @param servo1 The first servo
     * @param servo2 The second servo
     * @param guidance The algorithm used to compute the target yaw and the
     * heading
     */
    AutomaticWingAlgorithm(float Kp, float Ki, ServosList servo1,
                           ServosList servo2, GuidanceAlgorithm& guidance);

protected:
    // Guidance algorithm that sets the yaw.
    GuidanceAlgorithm& guidance;

    // PI controller tuned on the Kp and Ki passed through constructor
    std::unique_ptr<Boardcore::PIController> controller;

    /**
     * @brief Actual algorithm implementation, all parameters should be in NED
     *
     * @return The angle to set to the servo, positive is right and negative is
     * left
     */
    Boardcore::Units::Angle::Degree algorithmStep(
        const Boardcore::NASState& state);

    /**
     * @brief This method implements the automatic algorithm that will steer the
     * parafoil according to its position and velocity. IN THIS METHOD THE
     * GUIDANCE IS TRANSLATED
     */
    void step() override;

    /**
     * @brief Computes the difference between two angles
     *
     * @param a The first angle
     * @param b The second angle
     *
     * @returns angle(a) - angle(b)
     */
    Boardcore::Units::Angle::Radian angleDiff(
        Boardcore::Units::Angle::Radian a, Boardcore::Units::Angle::Radian b);

    // Logging structure
    WingAlgorithmData data;

    /**
     * @brief Mutex
     */
    miosix::FastMutex mutex;
};
}  // namespace Parafoil
