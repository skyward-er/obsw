/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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
#include <utils/DependencyManager/DependencyManager.h>

namespace Payload
{

class FlightModeManager;

class FlightStatsRecorder
    : public Boardcore::InjectableWithDeps<FlightModeManager>
{
public:
    struct Stats
    {
        // Liftoff
        uint64_t liftoffTs = 0;

        // Maximum acceleration during liftoff
        uint64_t liftoffMaxAccTs = 0;
        float liftoffMaxAcc      = 0.0f;

        // Shutdown
        uint64_t shutdownTs = 0;
        float shutdownAlt   = 0.0f;

        // Maximum vertical speed
        uint64_t maxSpeedTs = 0;
        float maxSpeed      = 0.0f;
        float maxSpeedAlt   = 0.0f;

        // Max mach
        uint64_t maxMachTs = 0;
        float maxMach      = 0.0f;

        // Apogee
        uint64_t apogeeTs = 0;
        float apogeeLat   = 0.0f;
        float apogeeLon   = 0.0f;
        float apogeeAlt   = 0.0f;

        // Maximum acceleration after apogee
        uint64_t apogeeMaxAccTs = 0;
        float apogeeMaxAcc      = 0.0f;

        // Deployment
        uint64_t dplTs = 0;
        float dplAlt   = 0.0f;

        // Maximum acceleration after deployment
        uint64_t dplMaxAccTs = 0;
        float dplMaxAcc      = 0.0f;

        // Minimum pressure (apogee pressure)
        float minPressure = 0.0f;
    };

    void reset();

    Stats getStats();

    void liftoffDetected(uint64_t ts);
    void shutdownDetected(uint64_t ts, float alt);
    void apogeeDetected(uint64_t ts, float lat, float lon, float alt);
    void deploymentDetected(uint64_t ts, float alt);

    void updateAcc(const Boardcore::AccelerometerData &data);
    void updateNas(const Boardcore::NASState &data);
    void updatePressure(const Boardcore::PressureData &data);

private:
    miosix::FastMutex statsMutex;
    Stats stats;
};

}  // namespace Payload
