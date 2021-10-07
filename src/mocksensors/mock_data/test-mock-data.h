/* Copyright (c) 2019 Skyward Experimental Rocketry
 * Author: Luca Mozzarelli
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

static const unsigned PRESSURE_DATA_SIZE = 2570;
extern const float SIMULATED_PRESSURE[PRESSURE_DATA_SIZE];

static const unsigned GPS_DATA_SIZE = 2570;
extern const float SIMULATED_LAT[GPS_DATA_SIZE];
extern const float SIMULATED_LON[GPS_DATA_SIZE];
extern const float SIMULATED_VNORD[GPS_DATA_SIZE];
extern const float SIMULATED_VEAST[GPS_DATA_SIZE];

static const unsigned IMU_DATA_SIZE          = 2570;
static const unsigned MOTION_SENSOR_AXIS_NUM = 3;
extern const float ACCELEROMETER_DATA[MOTION_SENSOR_AXIS_NUM][IMU_DATA_SIZE];
extern const float GYROSCOPE_DATA[MOTION_SENSOR_AXIS_NUM][IMU_DATA_SIZE];
extern const float MAGNETOMETER_DATA[MOTION_SENSOR_AXIS_NUM][IMU_DATA_SIZE];