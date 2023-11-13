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

#pragma once

#include <Groundstation/Automated/Config/FollowerConfig.h>
#include <algorithms/Algorithm.h>
#include <algorithms/NAS/NASState.h>
#include <diagnostic/PrintLogger.h>
#include <logger/Logger.h>
#include <sensors/SensorData.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Antennas
{

struct NEDCoords
{
    float n = 0;
    float e = 0;
    float d = 0;
};

/**
 * @brief A structure for storing angles repesenting antenna positions
 */
struct AntennaAngles
{
    float yaw;    //!< Angle from the X axis on the horizontal XY plane,
                  //!< anti-clockwise positive [deg]
    float pitch;  //!< Angle between the XY plane and the target position [deg]
};

class Follower : public Boardcore::Algorithm, public Boardcore::Module
{
public:
    Follower();

    bool init() override;

    void setAntennaCoordinates(const Boardcore::GPSData& gpsData)
    {
        antennaCoordinates = {gpsData.latitude, gpsData.longitude,
                              gpsData.height};
    };

    void setInitialRocketCoordinates(const Boardcore::GPSData& gpsData)
    {
        initialRocketCoordinates = {gpsData.latitude, gpsData.longitude,
                                    gpsData.height};
    }

    Eigen::Vector2f getInitialAntennaRocketDistance()
    {
        return initialAntennaRocketDistance;
    }

private:
    void step() override;

    /**
     * @brief Calculates the target angles from the given NED coordinates that
     * the antenna should point to.
     */
    AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned);

    const uint8_t maxInitRetries =
        120;  ///< max number of retries for GPS data acquisition

    Eigen::Vector3f antennaCoordinates;  ///< GPS coordinates of the antenna
                                         ///< [lat, lon, alt] [deg, deg, m]
    Eigen::Vector3f initialRocketCoordinates;  ///< GPS coordinates of the
                                               ///< rocket while in ramp [lat,
                                               ///< lon, alt] [deg, deg, m]
    Eigen::Vector2f
        initialAntennaRocketDistance;  ///< Distance between the antenna and
                                       ///< the rocket while in ramp [m]

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Follower");
};

}  // namespace Antennas
