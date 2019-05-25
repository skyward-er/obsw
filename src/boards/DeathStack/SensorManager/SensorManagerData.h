/*
 * Copyright (c) 2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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

#include <math/Stats.h>
#include <cstdint>
#include <cstring>
#include <ostream>

namespace DeathStackBoard
{

enum class SensorSamplerId : uint8_t
{
    STATS        = 0,
    GPS          = 10,
    MAGN         = 100,
    SIMPLE_20HZ  = 20,
    SIMPLE_250HZ = 250
};

enum class TempSensorId : uint8_t
{
    LM75B_ANALOG = 0,
    LM75B_IMU    = 1
};

enum class SensorManagerState : uint8_t
{
    IDLE,
    LOGGING
};

// Nominal value returned by SensorStatus.toNumeric() when every sensor was
// initialized successfully.
// 127 = 1 in every bit of the SensorStatus struct
static constexpr uint16_t NOMINAL_SENSOR_INIT_VALUE = 127;

struct SensorStatus
{
    // Imus
    uint16_t mpu9250 : 1;

    // Temperature sensors
    uint16_t lm75b_imu : 1;
    uint16_t lm75b_analog : 1;

    // GPS
    uint16_t piksi : 1;

    // Internal ADC
    uint16_t current_sensor : 1;
    uint16_t battery_sensor : 1;

    // External ADC
    uint16_t ad7994 : 1;

    /**
     * Converts data in the struct to a single uint16_t value
     * @return uint16_t representing the struct
     */
    uint16_t toNumeric()
    {
        uint16_t out;
        memcpy(&out, this, sizeof(out));
        return out;
    }

    static std::string header()
    {
        return "mpu9250,lm75b_imu,lm75b_analog,piksi,current_sensor,battery_"
               "sensor,ad7994";
    }

    void print(std::ostream& os) const
    {
        os << mpu9250 << "," << lm75b_imu << "," << lm75b_analog << "," << piksi
           << "," << current_sensor << "," << battery_sensor << "," << ad7994;
    }
};

struct LM75BData
{
    TempSensorId id;
    long long timestamp;
    float temp;

    static std::string header() { return "id,timestamp,temp\n"; }

    void print(std::ostream& os) const
    {
        os << (int)id << "," << timestamp << "," << temp << "\n";
    }
};

struct SensorManagerStatus
{
    uint64_t timestamp;
    SensorManagerState state;

    uint16_t sensor_status;
    static std::string header() { return "timestamp,state,sensor_status\n"; }

    void print(std::ostream& os) const
    {
        os << timestamp << "," << (int)state << "," << sensor_status << "\n";
    }
};

}  // namespace DeathStackBoard
