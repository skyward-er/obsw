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

#include <Payload/Wing/Guidance/GuidanceAlgorithm.h>

#include <Eigen/Core>

namespace Payload
{

struct EarlyManeuversPoints
{
    float targetN;
    float targetE;
    float emcN;
    float emcE;
    float m1N;
    float m1E;
    float m2N;
    float m2E;
};

/**
 * This class is the implementation of the Simple Closed Loop guidance.
 * It calculates the yaw between the current position and the target position by
 * calculating the difference between the two vectors and the angle formed by
 * the diff vector
 *
 * requires: WingControllers
 */
class EarlyManeuversGuidanceAlgorithm : public GuidanceAlgorithm
{

public:
    /**
     * @brief Constructor for the EM Algorithm
     *
     */
    EarlyManeuversGuidanceAlgorithm();

    virtual ~EarlyManeuversGuidanceAlgorithm();

    /**
     * @brief This method calculates the yaw angle of the parafoil knowing
     * the current position and the target position.
     *
     * @param[in] currentPositionNED the current NED position of the parafoil in
     * m
     * @param[in] targetPositionNED NED position of the target in m
     * @param[out] heading Saves the heading vector for logging purposes
     *
     * @returns the yaw angle of the parafoil in rad
     *
     */
    float calculateTargetAngle(const Eigen::Vector3f& currentPositionNED,
                               Eigen::Vector2f& heading) override;

    /**
     * @brief Set Early Maneuvers points
     *
     * @param[in] EMC NED
     * @param[in] M1 NED
     * @param[in] M2 NED
     *
     */
    void setPoints(Eigen::Vector2f targetNED, Eigen::Vector2f EMC,
                   Eigen::Vector2f M1, Eigen::Vector2f M2);

    /**
     * @brief Get Early Maneuvers points.
     */
    EarlyManeuversPoints getPoints();

private:
    /** @brief Updates the class target
     *
     */
    void computeActiveTarget(float altitude);

    /**
     * @brief Enumerates all the possible targets of the EM algorithm
     */
    enum class Target
    {
        EMC = 0,
        M1,
        M2,
        FINAL
    };

    // Point we are currently poinying to
    Target activeTarget;

    // Eigen::Vector2f targetNED;  // NED

    Eigen::Vector2f EMC;  // NED

    Eigen::Vector2f M1;  // NED

    Eigen::Vector2f M2;  // NED

    unsigned int targetAltitudeConfidence;
    unsigned int m2AltitudeConfidence;
    unsigned int m1AltitudeConfidence;
    unsigned int emcAltitudeConfidence;
};

}  // namespace Payload
