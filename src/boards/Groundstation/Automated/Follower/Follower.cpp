/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Niccolò Betto
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
#include <iostream>

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

bool Follower::init()
{
    // Antenna Coordinates
    Eigen::Vector2f antennaCoord{antennaCoordinates.head<2>()};
    // Rocket coordinates
    Eigen::Vector2f rocketCoord{initialRocketCoordinates.head<2>()};

    // Calculate the distance between the antenna and the rocket while in ramp
    initialAntennaRocketDistance =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    // Store the initial orientation of the antenna
    VN300Data vn300 =
        ModuleManager::getInstance().get<Sensors>()->getVN300LastSample();
    initialOrientation = {vn300.yaw, vn300.pitch};

    return true;
}

void Follower::step()
{
    Hub* hub = static_cast<Hub*>(
        ModuleManager::getInstance().get<Groundstation::HubBase>());
    NASState lastRocketNasState = hub->getLastRocketNasState();

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition = {lastRocketNasState.n, lastRocketNasState.e,
                                lastRocketNasState.d};

    // Calculating absolute angles from the NED frame
    AntennaAngles absoluteAngles =
        rocketPositionToAntennaAngles(rocketPosition);

    // Calculation actuation angles wrt the current position
    VN300Data vn300 =
        ModuleManager::getInstance().get<Sensors>()->getVN300LastSample();
    Eigen::Vector2f relativeOrientation = {
        vn300.yaw - initialOrientation.x(),
        vn300.pitch - initialOrientation.y()};
    AntennaAngles stepperAngles{
        absoluteAngles.theta1 - relativeOrientation.x(),
        absoluteAngles.theta2 - relativeOrientation.y()};

    // Propagator step

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed = std::abs((stepperAngles.theta1 * 1000) /
                                     (360 * FollowerConfig::FOLLOWER_PERIOD));
    float verticalSpeed   = std::abs((stepperAngles.theta2 * 1000) /
                                     (360 * FollowerConfig::FOLLOWER_PERIOD));

#ifndef NDEBUG
    std::cout << "[FOLLOWER] STEPPER "
              << "Angles: [" << stepperAngles.theta1 << ", "
              << stepperAngles.theta2 << "] "
              << "Speed: [" << horizontalSpeed << ", " << verticalSpeed
              << "]   VN300 Relative: [" << relativeOrientation.x() << ", "
              << relativeOrientation.y() << "]\n";
#endif

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

AntennaAngles Follower::rocketPositionToAntennaAngles(
    const NEDCoords& rocketNed)
{
    // NED of the antenna wrt current rocket position
    NEDCoords antennaNed = {rocketNed.n + initialAntennaRocketDistance.x(),
                            rocketNed.e + initialAntennaRocketDistance.y(),
                            rocketNed.d};
    AntennaAngles angles;
    // Calculating the horizontal angle in absolute ned frame
    // std::atan2 outputs angles in radians, we need to convert it to degrees
    angles.theta1 = std::atan2(antennaNed.n, antennaNed.e) / EIGEN_PI * 180;
    // Calculating the horizontal distance of the rocket
    float ground_dist =
        std::sqrt(antennaNed.n * antennaNed.n + antennaNed.e * antennaNed.e);
    // Calculating the vertical angle in absolute ned frame
    angles.theta2 = std::atan2(-antennaNed.d, ground_dist) / EIGEN_PI * 180;
    return angles;
}

}  // namespace Antennas
