/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Davide Basso
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

#include <algorithms/NAS/NASState.h>
#include <miosix.h>
#include <sensors/SensorData.h>
#include <units/Acceleration.h>
#include <units/Length.h>
#include <units/Pressure.h>
#include <units/Speed.h>
#include <utils/DependencyManager/DependencyManager.h>

namespace MockupMain
{

class FlightModeManager;

class FlightStatsRecorder
    : public Boardcore::InjectableWithDeps<FlightModeManager>
{
public:
    struct Stats
    {
        // Drop
        uint64_t dropTs = 0;

        // Maximum acceleration after drop, before deployment
        uint64_t dropMaxAccTs = 0;
        Boardcore::Units::Acceleration::MeterPerSecondSquared dropMaxAcc{0};

        // Maximum vertical speed
        uint64_t maxSpeedTs = 0;
        Boardcore::Units::Speed::MeterPerSecond maxSpeed{0};
        Boardcore::Units::Length::Meter maxSpeedAlt{0};

        // Deployment
        uint64_t dplTs = 0;
        Boardcore::Units::Length::Meter dplAlt{0};

        // Maximum acceleration after deployment
        uint64_t dplMaxAccTs = 0;
        Boardcore::Units::Acceleration::MeterPerSecondSquared dplMaxAcc{0};
    };

    void reset();

    Stats getStats();

    void dropDetected(uint64_t ts);
    void deploymentDetected(uint64_t ts, Boardcore::Units::Length::Meter alt);

    void updateAcc(const Boardcore::AccelerometerData& data);
    void updateNas(const Boardcore::NASState& data, float refTemperature);
    void updatePressure(const Boardcore::PressureData& data);

private:
    miosix::FastMutex statsMutex;
    Stats stats;
};

}  // namespace MockupMain
