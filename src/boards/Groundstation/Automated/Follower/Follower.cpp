/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Niccolò Betto, Federico Lolli, Nicolò Caruso
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
#include <Groundstation/Automated/SMController/SMController.h>
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
          Common::ReferenceConfig::defaultReferenceValues.refAltitude),
      targetAngles({0, 0})
{
}

void Follower::setAntennaCoordinates(const Boardcore::GPSData& gpsData)
{
    antennaCoordinates = {gpsData.latitude, gpsData.longitude, gpsData.height};
    Boardcore::Logger::getInstance().log(
        static_cast<LogAntennasCoordinates>(gpsData));
    antennaCoordinatesSet = true;
}

void Follower::setInitialRocketCoordinates(const Boardcore::GPSData& gpsData)
{
    if (rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Rocket coordinates already set");
        return;
    }

    initialRocketCoordinates = {gpsData.latitude, gpsData.longitude,
                                gpsData.height};
    Boardcore::Logger::getInstance().log(
        static_cast<LogRocketCoordinates>(gpsData));
    rocketCoordinatesSet = true;
}

void Follower::setLastRocketNasState(const NASState nasState)
{
    Lock<FastMutex> lock(lastRocketNasStateMutex);
    lastRocketNasState = nasState;
}

NASState Follower::getLastRocketNasState()
{
    Lock<FastMutex> lock(lastRocketNasStateMutex);
    return lastRocketNasState;
}

bool Follower::init()
{
    if (!antennaCoordinatesSet || !rocketCoordinatesSet)
    {
        LOG_ERR(logger, "Antenna or rocket coordinates not set");
        return false;
    }

    // Antenna Coordinates
    Eigen::Vector2f antennaCoord{antennaCoordinates.head<2>()};
    // Rocket coordinates
    Eigen::Vector2f rocketCoord{initialRocketCoordinates.head<2>()};

    initialAntennaRocketDistance =
        Aeroutils::geodetic2NED(rocketCoord, antennaCoord);

    LOG_INFO(logger, "Initial antenna - rocket distance: [{}, {}] [m]\n",
             initialAntennaRocketDistance[0], initialAntennaRocketDistance[1]);

    return true;
}

void Follower::step()
{
    NASState lastRocketNasState = getLastRocketNasState();

    // Getting the position of the rocket wrt the antennas in NED frame
    NEDCoords rocketPosition = {lastRocketNasState.n, lastRocketNasState.e,
                                lastRocketNasState.d};

    // Calculate the antenna target angles from the NED rocket coordinates
    targetAngles = rocketPositionToAntennaAngles(rocketPosition);

    VN300Data vn300 =
        ModuleManager::getInstance().get<Sensors>()->getVN300LastSample();

    // Calculate the amount to move from the current position
    AntennaAngles stepperAngles{targetAngles.yaw - vn300.yaw,
                                targetAngles.pitch - vn300.pitch};

    // Rotate in the shortest direction
    if (stepperAngles.yaw > 180)
    {
        stepperAngles.yaw -= 360;
    }
    else if (stepperAngles.yaw < -180)
    {
        stepperAngles.yaw += 360;
    }

    // Calculate angular velocity for moving the antennas toward position
    float horizontalSpeed = std::abs((stepperAngles.yaw * 1000) /
                                     (360 * FollowerConfig::FOLLOWER_PERIOD));
    float verticalSpeed   = std::abs((stepperAngles.pitch * 1000) /
                                     (360 * FollowerConfig::FOLLOWER_PERIOD));

#ifndef NDEBUG
    std::cout << "[FOLLOWER] STEPPER "
              << "Angles: [" << stepperAngles.yaw << ", " << stepperAngles.pitch
              << "] "
              << "Speed: [" << horizontalSpeed << ", " << verticalSpeed
              << "]   VN300 measure: [" << vn300.yaw << ", " << vn300.pitch
              << "]\n";
#endif

    ModuleManager::getInstance().get<Actuators>()->setSpeed(
        StepperList::STEPPER_X, horizontalSpeed);
    ModuleManager::getInstance().get<Actuators>()->setSpeed(
        StepperList::STEPPER_Y, verticalSpeed);

    // Actuating steppers
    bool actuated = true;
    actuated &= ModuleManager::getInstance().get<Actuators>()->moveDeg(
        StepperList::STEPPER_X, stepperAngles.yaw);
    actuated &= ModuleManager::getInstance().get<Actuators>()->moveDeg(
        StepperList::STEPPER_Y, stepperAngles.pitch);
    if (!actuated)
        LOG_ERR(logger, "Step antenna - At least one stepper did not moved\n");
}

AntennaAngles Follower::rocketPositionToAntennaAngles(
    const NEDCoords& rocketNed)
{
    // NED coordinates of the rocket in the NED antenna frame
    NEDCoords ned = {rocketNed.n + initialAntennaRocketDistance.x(),
                     rocketNed.e + initialAntennaRocketDistance.y(),
                     rocketNed.d};

    AntennaAngles angles;
    // Calculate the horizontal angle relative to the NED frame
    // std::atan2 outputs angles in radians, convert to degrees
    angles.yaw = std::atan2(ned.e, ned.n) / EIGEN_PI * 180;

    float distance = std::sqrt(ned.n * ned.n + ned.e * ned.e);
    // Calculate the vertical angle relative to the NED frame
    angles.pitch = std::atan2(-ned.d, distance) / EIGEN_PI * 180;

    return angles;
}

}  // namespace Antennas
