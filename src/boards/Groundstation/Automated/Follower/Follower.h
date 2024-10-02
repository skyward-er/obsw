/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano, Niccolò Betto, Federico Lolli
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

#include <Groundstation/Automated/Config/FollowerConfig.h>
#include <algorithms/Algorithm.h>
#include <algorithms/NAS/NASState.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <sensors/SensorData.h>

#include <utils/ModuleManager/ModuleManager.hpp>

#include "FollowerData.h"

namespace Antennas
{

class Follower : public Boardcore::Algorithm
{
public:
    Follower();

    bool init() override;

    // Setters for the GPS coordinates of the antenna
    void setAntennaCoordinates(const Boardcore::GPSData& gpsData);
    // Setters for the GPS coordinates of the rocket while in ramp
    void setInitialRocketCoordinates(const Boardcore::GPSData& gpsData);
    // Setters for the NAS state of the rocket
    void setLastRocketNasState(const Boardcore::NASState nasState);

    /**
     * @brief Getter for the target antenna position computed by the algorithm.
     * @returns The target antenna positions.
     */
    AntennaAngles getTargetAngles() { return targetAngles; }

private:
    void step() override;

    /**
     * @brief Synchronized getter that returns a copy of the last NAS state
     * of the rocket
     */
    Boardcore::NASState Follower::getLastRocketNasState();

    /**
     * @brief Calculates the target angles from the given NED coordinates that
     * the antenna should point to.
     */
    AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned);

    // max number of retries for GPS data acquisition
    const uint8_t maxInitRetries = 120;

    bool antennaCoordinatesSet = false;
    bool rocketCoordinatesSet  = false;

    Boardcore::NASState lastRocketNasState;
    miosix::FastMutex lastRocketNasStateMutex;

    // GPS coordinates of the antenna [lat, lon, alt] [deg, deg, m]
    Eigen::Vector3f antennaCoordinates;
    // GPS coordinates of the rocket while in ramp [lat, lon, alt] [deg, deg, m]
    Eigen::Vector3f initialRocketCoordinates;
    // Initial distance between the antenna and the rocket while in ramp [lat,
    // lon, alt] [deg, deg, m]
    Eigen::Vector2f initialAntennaRocketDistance;

    // Target yaw and pitch of the system [deg, deg]
    AntennaAngles targetAngles;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Follower");
};

}  // namespace Antennas
