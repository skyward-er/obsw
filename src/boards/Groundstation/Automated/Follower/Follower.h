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

struct AntennaAngles
{
    float theta1 = 0;
    float theta2 = 0;
};

class Follower : public Boardcore::Algorithm, public Boardcore::Module
{
public:
    Follower();

    bool init() override;

    void setAntennaPosition(const Boardcore::GPSData& gpsData)
    {
        antennaCoordinates = {gpsData.latitude, gpsData.longitude, gpsData.height};
    };

private:
    void step() override;

    AntennaAngles rocketPositionToAntennaAngles(const NEDCoords& ned);

    const uint8_t maxInitRetries =
        120;  ///< max number of retries for GPS data acquisition
    Eigen::Vector3f antennaCoordinates;

    Boardcore::PrintLogger logger = Boardcore::Logging::getLogger("Follower");
};

}  // namespace Antennas
