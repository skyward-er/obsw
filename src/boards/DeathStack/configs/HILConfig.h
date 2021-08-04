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

// serial port number
static constexpr int HIL_SERIAL_PORT_NUM = 3;

// baudrate of connection
static constexpr int HIL_BAUDRATE = 115200;

// Period of simulation in milliseconds
static constexpr int HIL_SIMULATION_PERIOD = 100;  // 10 hz

// sample frequency of sensor (samples/second)
static constexpr int HIL_IMU_PERIOD   = 10;   // 100 hz
static constexpr int HIL_ACCEL_PERIOD = 10;   // 100 hz
static constexpr int HIL_GYRO_PERIOD  = 10;   // 100 hz
static constexpr int HIL_MAGN_PERIOD  = 10;   // 100 hz
static constexpr int HIL_BARO_PERIOD  = 50;   // 20 hz
static constexpr int HIL_GPS_PERIOD   = 100;  // 10 hz;

// // update frequency of the Navigation System
// const int KALMAN_FREQ = 100;
// // update frequency of airbrakes control algorithm
// const int CONTROL_FREQ = 10;
// // update frequency of airbrakes control algorithm
// const int ADA_FREQ = 20;

// ADA configs
// [TODO?] Prendi valori da config ADA
// const float deploymentAltitude   = 450;
// const float referenceAltitude    = 109;  // launchpad Altitude for Pont De
// Sor? const float referenceTemperature = 15;

// Number of samples per sensor at each simulator iteration
static constexpr int N_DATA_IMU   = (HIL_SIMULATION_PERIOD / HIL_IMU_PERIOD);
static constexpr int N_DATA_ACCEL = (HIL_SIMULATION_PERIOD / HIL_ACCEL_PERIOD);
static constexpr int N_DATA_GYRO  = (HIL_SIMULATION_PERIOD) / HIL_GYRO_PERIOD;
static constexpr int N_DATA_MAGN  = (HIL_SIMULATION_PERIOD / HIL_MAGN_PERIOD);
static constexpr int N_DATA_BARO  = (HIL_SIMULATION_PERIOD / HIL_BARO_PERIOD);
static constexpr int N_DATA_GPS   = (HIL_SIMULATION_PERIOD / HIL_GPS_PERIOD);
// const int N_DATA_KALMAN = (HIL_KALMAN_FREQ * HIL_SIMULATION_PERIOD);

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received.
 *
 * This structure then is accessed by sensors and other components in order to
 * get the data they need
 *
 * NOTE: all the fields contained in the struct are floats because it is the
 * data type sent over serial by MATLAB
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
