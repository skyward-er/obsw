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

#include "math/Vec3.h"

const float SIM_BAUDRATE = 256000; /**< baudrate of connection */
const float SIMULATION_PERIOD =
    100; /**< Period of simulation in milliseconds */

const int N_DATA_ACCEL = 10; /**< Number of simulations */
const int N_DATA_GYRO  = 10; /**< Number of simulations */
const int N_DATA_MAGN  = 10; /**< Number of simulations */
const int N_DATA_BARO  = 2;  /**< Number of simulations */
const int N_DATA_GPS   = 1;  /**< Number of simulations */
const int N_DATA_KALM  = 1;  /**< Number of simulations */

/**
 * @brief Data structure used by the simulator in order to directly deserialize
 * the data received
 *
 * This structure then is accessed by every mock sensor in order to get the data
 * they need
 */
struct SensorData
{
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
};

/**
 * @brief Data structure expected by the simulator
 */
using ActuatorData = float;