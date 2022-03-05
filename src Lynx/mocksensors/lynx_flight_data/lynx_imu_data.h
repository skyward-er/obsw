/* Copyright (c) 2021 Skyward Experimental Rocketry
 * Author: Luca Conterio
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

/**
 * BMX160 IMU data from Lynx flight test in Roccaraso.
 * Sampled at 50 Hz (20 ms period).
 * Each sample here is the result of an average over an entire FIFO, corrected
 * using calibration parameters and rotated in body frame.
 */
static constexpr unsigned IMU_DATA_SIZE      = 6463;
static const unsigned MOTION_SENSOR_AXIS_NUM = 3;
extern const float ACCEL_DATA[IMU_DATA_SIZE][MOTION_SENSOR_AXIS_NUM];
extern const float GYRO_DATA[IMU_DATA_SIZE][MOTION_SENSOR_AXIS_NUM];
extern const float MAG_DATA[IMU_DATA_SIZE][MOTION_SENSOR_AXIS_NUM];
