/* Copyright (c) 2025-2026 Skyward Experimental Rocketry
 * Author: Davide Basso, Leonardo Montecchi
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

#include <Parafoil/Configs/WingConfig.h>
#include <Parafoil/Sensors/Sensors.h>
#include <units/Length.h>
#include <utils/AeroUtils/AeroUtils.h>
#include <utils/AltitudeMap/AltitudeMap.h>
#include <utils/DependencyManager/DependencyManager.h>

#include "AltitudeTrigger.h"
#include "LandingFlareData.h"

namespace Parafoil
{

class BoardScheduler;
class NASController;
class Sensors;

class LandingFlare : public Boardcore::InjectableWithDeps<
                         Boardcore::InjectableBase<AltitudeTrigger>,
                         BoardScheduler, NASController, Sensors>
{
public:
    LandingFlare()
        : Super({
              .threshold  = Config::Wing::LandingFlare::ALTITUDE,
              .confidence = Config::Wing::LandingFlare::CONFIDENCE,
              .updateRate = Config::Wing::LandingFlare::UPDATE_RATE,
          }),
          map(Config::Wing::LandingFlare::ALTITUDE_MAP_ADDRESS) {};

    bool start();

    /**
     * @brief Set the landing target position in NED coordinates.
     */
    void setTargetGEO(Eigen::Vector2f targetGEO);

private:
    void update();

    Boardcore::AltitudeMap map{nullptr};

    /**
     * @brief Calculate the altitude above ground level (using the ground
     * altitude variation) based on the current gps position.
     */
    float calculateAboveGroundAltitude(LandingFlareData &data);

    /**
     * @brief Find the current position in NED coordinates using gps.
     */
    Eigen::Vector2f findCurrentPositionNED();

    Eigen::Vector2f
        targetGEO;  ///< Target position in gps GEO coordinates, used to
                    ///< calculate the local coordinates for the altitude map
};

}  // namespace Parafoil
