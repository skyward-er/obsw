/* Copyright (c) 2020-2023 Skyward Experimental Rocketry
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

#include <drivers/timer/TimestampTimer.h>
#include <drivers/usart/USART.h>
#include <utils/Debug.h>
#include <utils/Stats/Stats.h>

#include <list>
#include <utils/ModuleManager/ModuleManager.hpp>

#include "algorithms/ADA/ADAData.h"
#include "algorithms/NAS/NAS.h"
#include "algorithms/NAS/NASState.h"
#include "sensors/SensorInfo.h"

namespace HILConfig
{

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
const int BARO_FREQ  = 50;
const int GPS_FREQ   = 10;

/** sensors configuration */
const SensorConfig accelConfig("accel", ACCEL_FREQ);
const SensorConfig gyroConfig("gyro", GYRO_FREQ);
const SensorConfig magnConfig("magn", MAGN_FREQ);
const SensorConfig imuConfig("imu", IMU_FREQ);
const SensorConfig baroConfig("baro", BARO_FREQ);
const SensorConfig gpsConfig("gps", GPS_FREQ);

/** Number of samples per sensor at each simulator iteration */
const int N_DATA_ACCEL = (ACCEL_FREQ * SIMULATION_PERIOD) / 1000;  // 10
const int N_DATA_GYRO  = (GYRO_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_MAGN  = (MAGN_FREQ * SIMULATION_PERIOD) / 1000;   // 10
const int N_DATA_IMU   = (IMU_FREQ * SIMULATION_PERIOD) / 1000;    // 10
const int N_DATA_BARO  = (BARO_FREQ * SIMULATION_PERIOD) / 1000;   // 2
const int N_DATA_GPS   = (GPS_FREQ * SIMULATION_PERIOD) / 1000;    // 1

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 */
struct SimulatorData
{
    struct Accelerometer
    {
        float measures[N_DATA_ACCEL][3];
    } accelerometer;

    struct Gyro
    {
        float measures[N_DATA_GYRO][3];
    } gyro;

    struct Magnetometer
    {
        float measures[N_DATA_MAGN][3];
    } magnetometer;

    struct Barometer
    {
        float measures[N_DATA_BARO];
    } barometer;

    struct Gps
    {
        float positionMeasures[N_DATA_GPS][3];
        float velocityMeasures[N_DATA_GPS][3];
        float fix;
        float num_satellites;
    } gps;

    struct Wing
    {
        float wind[2];
        float state;
    } wing;

    struct Flags
    {
        float flag_flight;
        float flag_ascent;
        float flag_burning;
        float flag_airbrakes;
        float flag_para1;
        float flag_para2;
    } flags;

    void printAccelerometer()
    {
        TRACE("accel\n");
        for (int i = 0; i < N_DATA_ACCEL; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", accelerometer.measures[i][0],
                  accelerometer.measures[i][1], accelerometer.measures[i][2]);
    }

    void printGyro()
    {
        TRACE("gyro\n");
        for (int i = 0; i < N_DATA_GYRO; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gyro.measures[i][0],
                  gyro.measures[i][1], gyro.measures[i][2]);
    }

    void printMagnetometer()
    {
        TRACE("magneto\n");
        for (int i = 0; i < N_DATA_MAGN; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", magnetometer.measures[i][0],
                  magnetometer.measures[i][1], magnetometer.measures[i][2]);
    }

    void printGPS()
    {
        TRACE("gps\n");
        TRACE("pos\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.positionMeasures[i][0],
                  gps.positionMeasures[i][1], gps.positionMeasures[i][2]);

        TRACE("vel\n");
        for (int i = 0; i < N_DATA_GPS; i++)
            TRACE("%+.3f\t%+.3f\t%+.3f\n", gps.velocityMeasures[i][0],
                  gps.velocityMeasures[i][1], gps.velocityMeasures[i][2]);
        TRACE("fix:%+.3f\tnsat:%+.3f\n", gps.fix, gps.num_satellites);
    }

    void printBarometer()
    {
        TRACE("press1\n");
        for (int i = 0; i < N_DATA_BARO; i++)
            TRACE("%+.3f\n", barometer.measures[i]);
    }

    void printFlags()
    {
        TRACE("flags\n");
        TRACE(
            "flight:\t%+.3f\n"
            "ascent:\t%+.3f\n"
            "burning:\t%+.3f\n"
            "airbrakes:\t%+.3f\n"
            "para1:\t%+.3f\n"
            "para2:\t%+.3f\n",
            flags.flag_flight, flags.flag_ascent, flags.flag_burning,
            flags.flag_airbrakes, flags.flag_para1, flags.flag_para2);
    }

    void print()
    {
        printAccelerometer();
        printGyro();
        printMagnetometer();
        printGPS();
        printBarometer();
        printFlags();
    }
};

/**
 * @brief Data structure expected by the simulator
 */
struct ActuatorData
{
    float state;
    float windX;
    float windY;
    float psiRef;
    float deltaA;

    void print() const
    {
        TRACE(
            "state: %+.3f, wind: [%+.3f,%+.3f], psiRef: %+.3f, deltaA: %+.3f\n",
            state, windX, windY, psiRef, deltaA);
    }
};

}  // namespace HILConfig
