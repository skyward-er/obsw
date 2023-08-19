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
#include <Groundstation/Automated/Sensors/Sensors.h>
#include <utils/AeroUtils/AeroUtils.h>

#include <Eigen/Dense>
#include <cmath>

using namespace Boardcore;

namespace Antennas
{

Follower::Follower() {}

bool Follower::init()
{
    for (int i = 0; i < maxInitRetries; i++)
    {
        VN300Data vn300Data = ModuleManager::getInstance()
                                  .get<Antennas::Sensors>()
                                  ->getVN300LastSample();

        if (vn300Data.fix_gps > 0)
        {
            antennaPosition.gpsTimestamp  = vn300Data.insTimestamp;
            antennaPosition.latitude      = vn300Data.latitude;
            antennaPosition.latitude      = vn300Data.latitude;
            antennaPosition.height        = vn300Data.altitude;
            antennaPosition.velocityNorth = vn300Data.nedVelX;
            antennaPosition.velocityEast  = vn300Data.nedVelY;
            antennaPosition.velocityDown  = vn300Data.nedVelZ;
            antennaPosition.satellites    = vn300Data.fix_gps;
            antennaPosition.fix           = (vn300Data.fix_gps > 0);
            LOG_INFO(logger, "Initialization Succeeded");
            return true;
        }
        else
        {
            // Waiting for GPS fix for another second
            LOG_WARN(
                logger,
                "Initialization Failed for the {} time, retrying in one second",
                i);
            miosix::Thread::sleep(1000);
        }
    }

    return false;
}

/**
 * @param get rocket NAS state.
 */
NASState Follower::getLastRocketNasState() { return lastRocketNasState; }

GPSData Follower::getLastRocketGpsState() { return lastRocketGpsState; }

/**
 * @param set rocket NAS state.
 */
void Follower::setLastRocketState(const NASState& nas, const GPSData& gps)
{
    lastRocketNasState = nas;
    lastRocketGpsState = gps;
}

void Follower::step()
{
    // Calculating the NED coordinates in Automated Antennas reference system
    Eigen::Vector2f antennasCoord{antennaPosition.latitude,
                                  antennaPosition.longitude};
    Eigen::Vector2f rocketCoord{lastRocketGpsState.latitude,
                                lastRocketGpsState.longitude};
    Eigen::Vector2f neRocket =
        Aeroutils::geodetic2NED(rocketCoord, antennasCoord);

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition;
    rocketPosition.n = neRocket[0];
    rocketPosition.e = neRocket[1];
    rocketPosition.d = (-antennaPosition.height);

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
