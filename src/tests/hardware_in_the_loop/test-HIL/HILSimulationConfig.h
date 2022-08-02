/* Copyright (c) 2020 Skyward Experimental Rocketry
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

#include <utils/Stats/Stats.h>

#include <list>

#include "algorithms/ADA/ADAData.h"
#include "algorithms/NAS/NASState.h"
#include "old_examples/shared/math/Vec3.h"
#include "sensors/SensorInfo.h"

struct SensorConfig : public Boardcore::SensorInfo
{
    SensorConfig(const std::string s, const uint32_t period)
        : Boardcore::SensorInfo{s, period, []() {}, true}
    {
    }
};

/** baudrate of connection */
const int SIM_BAUDRATE = 115200;

/** Period of simulation in milliseconds */
const int SIMULATION_PERIOD = 100;

/** sample frequency of sensor (samples/second) */
const int ACCEL_FREQ = 100;
const int GYRO_FREQ  = 100;
const int MAGN_FREQ  = 100;
const int IMU_FREQ   = 100;
const int BARO_FREQ  = 20;
const int GPS_FREQ   = 10;

/** update frequency of the Navigation System */
const int KALM_FREQ = 10;

/** update frequency of airbrakes control algorithm */
const int CONTROL_FREQ = 10;

/** min and max values in radiants of the actuator */
const float MinAlphaDegree = 0.0;
const float MaxAlphaDegree = 0.84;

/** sensors configuration */
const SensorConfig accelConfig("accel", ACCEL_FREQ);
const SensorConfig gyroConfig("gyro", GYRO_FREQ);
const SensorConfig magnConfig("magn", MAGN_FREQ);
const SensorConfig imuConfig("imu", IMU_FREQ);
const SensorConfig baroConfig("baro", BARO_FREQ);
const SensorConfig gpsConfig("gps", GPS_FREQ);
const SensorConfig kalmConfig("kalm", KALM_FREQ);

/** Number of samples per sensor at each simulator iteration */
const int N_DATA_ACCEL = (ACCEL_FREQ * SIMULATION_PERIOD) / 1000;  // 10
const int N_DATA_GYRO  = (GYRO_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_MAGN  = (MAGN_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_BARO  = (BARO_FREQ * SIMULATION_PERIOD) / 1000;   // 2
const int N_DATA_GPS   = (GPS_FREQ * SIMULATION_PERIOD) / 1000;    // 1
const int N_DATA_KALM  = (KALM_FREQ * SIMULATION_PERIOD) / 1000;   // 1

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
public:
    struct Accelerometer
    {
        Boardcore::Vec3 measures[N_DATA_ACCEL];
    } accelerometer;

    struct Gyro
    {
        Boardcore::Vec3 measures[N_DATA_GYRO];
    } gyro;

    struct Magnetometer
    {
        Boardcore::Vec3 measures[N_DATA_MAGN];
    } magnetometer;

    struct Gps
    {
        Boardcore::Vec3 positionMeasures[N_DATA_GPS];
        Boardcore::Vec3 velocityMeasures[N_DATA_GPS];
        float fix;
        float num_satellites;
    } gps;

    struct Barometer
    {
        float measures[N_DATA_BARO];
    } barometer;

    struct Kalman
    {
        float z;
        float vz;
        float vMod;
    } kalman;

    struct Flags
    {
        float flag_flight;
        float flag_ascent;
        float flag_burning;
        float flag_airbrakes;
        float flag_para1;
        float flag_para2;
    } flags;
};

/**
 * @brief ADA data sent to the simulator
 */
struct ADAdataHIL
{
    uint64_t ada_timestamp;
    float mslAltitude;    // Altitude at mean sea level [m].
    float verticalSpeed;  // Vertical speed [m/s].

    ADAdataHIL& operator+=(const ADAdataHIL& x)
    {
        this->ada_timestamp += x.ada_timestamp;
        this->mslAltitude += x.mslAltitude;
        this->verticalSpeed += x.verticalSpeed;

        return *this;  // return the result by reference
    }

    ADAdataHIL operator/(int x)
    {
        return ADAdataHIL{this->ada_timestamp / x, this->mslAltitude / x,
                          this->verticalSpeed / x};
    }

    ADAdataHIL operator*(int x)
    {
        return ADAdataHIL{this->ada_timestamp * x, this->mslAltitude * x,
                          this->verticalSpeed * x};
    }

    static std::string header()
    {
        return "timestamp,mslAltitude,verticalSpeed\n";
    }

    void print(std::ostream& os) const
    {
        os << ada_timestamp << "," << mslAltitude << "," << verticalSpeed
           << "\n";
    }
};

/**
 * @brief Data structure expected by the simulator
 */
typedef struct
{
    // Airbrakes opening (percentage)
    float airbrakes_opening;

    // NAS
    Boardcore::NASState nasState;

    // ADA
    ADAdataHIL adaState;

    void print() const
    {
        printf(
            "abk:%f\nned:%f,%f,%f\nbody:%f,%f,%f\nq:%f,%f,%f,%f\nada:%f,%f\n\n",
            airbrakes_opening, nasState.n, nasState.e, nasState.d, nasState.bx,
            nasState.by, nasState.bz, nasState.qx, nasState.qy, nasState.qz,
            nasState.qw, adaState.mslAltitude, adaState.verticalSpeed);
    }
} ActuatorData;

/**
 * @brief Data structure in order to store elaborated data by the algorithms
 */
typedef struct
{
    // Airbrakes opening (percentage)
    float airbrakes_opening;

    // NAS
    std::list<Boardcore::NASState> nasState;

    // ADA
    std::list<ADAdataHIL> adaState;

    void setAirBrakesOpening(float airbrakes_opening)
    {
        this->airbrakes_opening = airbrakes_opening;
        TRACE("setted abk opening\n");
    };

    void addNASState(Boardcore::NASState nasState)
    {
        this->nasState.push_back(nasState);
        TRACE("added nas state\n");
    };

    void addADAState(Boardcore::ADAState adaState)
    {
        ADAdataHIL data{adaState.timestamp, adaState.mslAltitude,
                        adaState.verticalSpeed};

        this->adaState.push_back(data);
        TRACE("added ada state\n");
    };

    ActuatorData getAvgActuatorData()
    {
        // NAS
        int n                 = nasState.size();
        float nasTimestampAvg = 0;
        auto nasXAvg          = this->nasState.front().getX().setZero();

        for (auto it = this->nasState.begin(); it != this->nasState.end(); ++it)
        {
            // Eigen::Matrix<float, 13, 1>
            nasTimestampAvg += (it->timestamp / n);
            nasXAvg += (it->getX() / n);
        }
        Boardcore::NASState nasStateAvg(nasTimestampAvg, nasXAvg);

        // ADA
        auto adaStateAvg = this->adaState.front() * 0;
        n                = adaState.size();

        for (auto x : this->adaState)
        {
            adaStateAvg += (x / n);
        }

        return ActuatorData{airbrakes_opening, nasStateAvg, adaStateAvg};
    }
} ElaboratedData;
