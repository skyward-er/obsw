/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Davide Basso
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
#include <diagnostic/PrintLogger.h>
#include <units/Angle.h>
#include <units/Length.h>
#include <units/Speed.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "CASCAConfig.h"

namespace Parafoil
{
class NASController;

/**
 * Implementation of the Controlled Altitude System for Canopy Adjustments,
 * using terminal trajectory generation, Dubins path and minimum control,
 * based on the current position.
 */
class CASCA : public Boardcore::InjectableWithDeps<NASController>
{
public:
    constexpr static uint16_t MANUEVER_TRAJECTORY_POINTS = 30;
    constexpr static uint16_t TRAJECTORY_POINTS =
        MANUEVER_TRAJECTORY_POINTS * 3;

    /**
     * @brief Position structure representing the state of the parafoil.
     */
    struct Position
    {
        Boardcore::Units::Length::Meter x;
        Boardcore::Units::Length::Meter y;
        Boardcore::Units::Angle::Radian orientation;
        Boardcore::Units::Length::Meter altitude;
    };

    using ManueverTrajectory = std::array<Position, MANUEVER_TRAJECTORY_POINTS>;
    using Trajectory         = std::array<Position, TRAJECTORY_POINTS>;

    CASCA(CASCAConfig config, Position target) : config(config), target(target)
    {
    }

    /**
     * @brief Compute the trajectory for the parafoil using the CASCA algorithm.
     * @return The computed trajectory.
     */
    Trajectory computeTrajectory();

    /**
     * @brief Set the target position for the algorithm.
     * @param newTarget New target position.
     */
    void setTargetPosition(Position newTarget);

private:
    /**
     *  @brief Current state of the parafoil.
     */
    struct State
    {
        // Vertical velocity of the parafoil in NED z axis
        Boardcore::Units::Speed::MeterPerSecond verticalVelocity;
        // Module of the velocity vector in the NED plan (xy)
        Boardcore::Units::Speed::MeterPerSecond inPlaneVelocity;
        // Glide ratio computed as vertical velocity / in-plane velocity
        float glideRatio;
    };

    /**
     * @brief Boundary conditions for the CASCA algorithm.
     */
    struct BoundaryConditions
    {
        Position currentPosition;
        Position targetPosition;
    };

    /**
     * @brief Dubins object for computing Dubins maneuvers.
     */
    struct Dubins
    {
        const State& state;  ///< Current state of the parafoil
        const BoundaryConditions&
            boundary;  ///< Boundary conditions for the algorithm

        enum class SequenceMode
        {
            RSL,  // Right - Straight - Left
            LSR,  // Left  - Straight - Right
            RSR,  // Right - Straight - Right
            LSL   // Left  - Straight - Left
        };

        enum class Maneuver
        {
            Straight,
            LeftTurn,
            RightTurn
        };

        using ManeuversSequence = std::array<Maneuver, 3>;

        struct Parameters
        {
            ManueverTrajectory firstManeuverTrajectory;
            ManueverTrajectory secondManeuverTrajectory;
            ManueverTrajectory thirdManeuverTrajectory;

            ManeuversSequence maneuvers;  ///< Sequence of maneuvers to perform
            Boardcore::Units::Length::Meter radius;  ///< Trajectory radius
        };

        struct Margins
        {
            Boardcore::Units::Length::Meter
                tau;    ///< Total Trajectory Required Altitude [m]
            float eta;  ///< Altitude Margin Parameter [-]
        };

        // Dubins_Trajectory_OBS
        Parameters computeManuevers(Boardcore::Units::Length::Meter radius);

        // Dubins_Optimizer
        SequenceMode findOptimalMode();

        Margins computeMargins(Parameters& parameters);

        // Dubins_Maneuvers
        Trajectory computeStraightManueverTrajectory(
            const Position& start, Boardcore::Units::Length::Meter distance);
        Trajectory computeCurvedManueverTrajectory(
            const Position& start, Boardcore::Units::Angle::Radian angle);
    };

    Trajectory computeMinimumControlTrajectory(
        const BoundaryConditions& boundary, const State& state);
    Trajectory computeDubinsTrajectory(const BoundaryConditions& boundary,
                                       const State& state);
    Trajectory computeTerminalTrajectory(const BoundaryConditions& boundary,
                                         const State& state);

    Position target;     ///< Target position for the parafoil
    CASCAConfig config;  ///< Algorithm configuration parameters
    PrintLogger logger = Boardcore::Logging::getLogger("CASCA");
};

}  // namespace Parafoil
