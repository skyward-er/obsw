/* Copyright (c) 2020 Skyward Experimental Rocketry
 * Authors: Emilio Corigliano
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include "math/Vec3.h"
#include "sensors/SensorInfo.h"

struct SensorConfig : public SensorInfo
{
    SensorConfig(const uint32_t period) : SensorInfo{period, []() {}, false, false}
    {
    }
};

/** serial port number */
const int SIM_SERIAL_PORT_NUM = 3;

/** baudrate of connection */
const int SIM_BAUDRATE = 115200;

/** Period of simulation in milliseconds */
const int SIMULATION_PERIOD = 100;

/** sample frequency of sensor (samples/second) */
const int IMU_FREQ   = 100;
const int ACCEL_FREQ = 100;
const int GYRO_FREQ  = 100;
const int MAGN_FREQ  = 100;
const int BARO_FREQ  = 20;
const int GPS_FREQ   = 10;

/** update frequency of the Navigation System */
const int KALMAN_FREQ = 100;

/** update frequency of airbrakes control algorithm */
const int CONTROL_FREQ = 10;

/** update frequency of airbrakes control algorithm */
const int ADA_FREQ = 20;

/** ADA configs
 * [TODO?] Prendi valori da config ADA
 */
const float deploymentAltitude   = 450;
const float referenceAltitude    = 109;  // launchpad Altitude for Pont De Sor?
const float referenceTemperature = 15;

/** sensors configuration */
const SensorConfig imuConfig(1000 / IMU_FREQ);
const SensorConfig baroConfig(1000 / BARO_FREQ);
const SensorConfig gpsConfig(1000 / GPS_FREQ);

/** Number of samples per sensor at each simulator iteration */
const int N_DATA_IMU    = (IMU_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_ACCEL  = (ACCEL_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_GYRO   = (GYRO_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_MAGN   = (MAGN_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_BARO   = (BARO_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_GPS    = (GPS_FREQ * SIMULATION_PERIOD) / 1000;
const int N_DATA_KALMAN = (KALMAN_FREQ * SIMULATION_PERIOD) / 1000;

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 *
 * NOTE : all the fields contained in the struct are floats because
 *        it is the data type sent over serial by MATLAB
 */
struct SimulatorData
{
public:
    struct Accelerometer
    {
        Vec3 measures[N_DATA_ACCEL];
    } accelerometer;

    struct Gyro
    {
        Vec3 measures[N_DATA_GYRO];
    } gyro;

    struct Magnetometer
    {
        Vec3 measures[N_DATA_MAGN];
    } magnetometer;

    struct Gps
    {
        Vec3 positionMeasures[N_DATA_GPS];
        Vec3 velocityMeasures[N_DATA_GPS];
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
        float flag_aerobrakes;
        float flag_para1;
        float flag_para2;
    } flags;
};

/**
 * @brief Data structure expected by the simulator
 */
using ActuatorData = float;