/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

#include "Follower.h"

#include <Groundstation/Automated/Actuators/Actuators.h>
#include <Groundstation/Automated/Hub.h>
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <common/ReferenceConfig.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <Eigen/Dense>
#include <cmath>

using namespace Boardcore;

namespace Antennas
{

Follower::Follower()
    : antennaCoordinates(
          Common::ReferenceConfig::defaultReferenceValues.refLatitude,
          Common::ReferenceConfig::defaultReferenceValues.refLongitude,
          Common::ReferenceConfig::defaultReferenceValues.refAltitude)
{
}

bool Follower::init() { return true; }

void Follower::step()
{
    GPSData lastRocketGpsState =
        static_cast<Hub*>(
            ModuleManager::getInstance().get<Groundstation::HubBase>())
            ->getLastRocketGpsState();

    // Antennas Coordinates
    Eigen::Vector2f antennaCoord{antennaCoordinates[0], antennaCoordinates[1]};

    // Calculating the NED coordinates in Automated Antennas reference system
    Eigen::Vector2f rocketCoord{lastRocketGpsState.latitude,
                                lastRocketGpsState.longitude};
    Eigen::Vector2f neRocket =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition;
    rocketPosition.n = neRocket[0];
    rocketPosition.e = neRocket[1];
    rocketPosition.d = (-antennaCoordinates[2]);

    // Calculating absolute angles from the NED frame
    AntennaAngles absoluteAngles =
        rocketPositionToAntennaAngles(rocketPosition);

    // Calculation actuation angles wrt the current position
    VN300Data vn300 =
        ModuleManager::getInstance().get<Sensors>()->getVN300LastSample();
    AntennaAngles stepperAngles{absoluteAngles.theta1 - vn300.yaw,
                                absoluteAngles.theta2 - vn300.pitch};

    // Propagator step

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed =
        (stepperAngles.theta1 * 1000) / (360 * FollowerConfig::FOLLOWER_PERIOD);
    float verticalSpeed =
        (stepperAngles.theta2 * 1000) / (360 * FollowerConfig::FOLLOWER_PERIOD);

    ModuleManager::getInstance().get<Actuators>()->setSpeed(
        Actuators::StepperList::HORIZONTAL, horizontalSpeed);
    ModuleManager::getInstance().get<Actuators>()->setSpeed(
        Actuators::StepperList::VERTICAL, verticalSpeed);

    // Actuating steppers
    ModuleManager::getInstance().get<Actuators>()->moveDeg(
        Actuators::StepperList::HORIZONTAL, stepperAngles.theta1);
    ModuleManager::getInstance().get<Actuators>()->moveDeg(
        Actuators::StepperList::VERTICAL, stepperAngles.theta2);
}

AntennaAngles Follower::rocketPositionToAntennaAngles(const NEDCoords& ned)
{
    AntennaAngles angles;
    // Calculating the horizontal angle in absolute ned frame
    angles.theta1 = std::atan2(ned.n, ned.e);
    // Calculating the horizontal distance of the rocket
    float ground_dist = std::sqrt(ned.n * ned.n + ned.e * ned.e);
    // Calculating the vertical angle in absolute ned frame
    angles.theta2 = std::atan2(-ned.d, ground_dist);
    return angles;
}

}  // namespace Antennas
