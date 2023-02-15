/* Copyright (c) 2022 Skyward Experimental Rocketry
 * Author: Matteo Pignataro
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

#include <Singleton.h>
#include <algorithms/NAS/NASState.h>
#include <common/Mavlink.h>
#include <sensors/SensorData.h>
#include <sensors/analog/Pitot/PitotData.h>

namespace Payload
{

/**
 * @brief This class records some valuable data that we need to send to quick
 * analyze the flight in the mean time.
 */
class FlightStatsRecorder : public Boardcore::Singleton<FlightStatsRecorder>
{
    friend class Boardcore::Singleton<FlightStatsRecorder>;

public:
    void update(Boardcore::AccelerometerData data);
    void update(Boardcore::NASState state);
    void update(Boardcore::PitotData data);
    void update(Boardcore::PressureData data);
    void setApogee(Boardcore::GPSData data);

    mavlink_payload_stats_tm_t getStats();

private:
    mavlink_payload_stats_tm_t stats;

    FlightStatsRecorder();
};

}  // namespace Payload
