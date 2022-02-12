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

/**
 * This class specifies the sensors constants that the sensor manager
 * needs to know about every device. For example the sample time is
 * essential to understand how much time a sensor should wait before
 * another sample.
 */

#pragma once

#include <sensors/UbloxGPS/UbloxGPS.h>
#include <sensors/MPU9250/MPU9250.h>
#include <sensors/BME280/BME280.h>

using namespace Boardcore;

namespace ParafoilTestDev
{
    //GPS settings TODO SPI
    static miosix::GpioPin GPS_CS(GPIOA_BASE, 4);
    static constexpr unsigned int GPS_SAMPLE_RATE       = 25;
    static constexpr unsigned int GPS_SAMPLE_PERIOD     = 1000 / GPS_SAMPLE_RATE;
    static constexpr unsigned int GPS_BAUD_RATE         = 460800;

    //IMU MPU9250 settings
    static miosix::GpioPin IMU_CS(GPIOA_BASE, 5);
    static MPU9250::MPU9250GyroFSR IMU_GYRO_SCALE       = MPU9250::GYRO_FSR_250DPS;
    static MPU9250::MPU9250AccelFSR IMU_ACCEL_SCALE     = MPU9250::ACCEL_FSR_2G;
    static constexpr unsigned short IMU_SAMPLE_RATE   = 100;
    static constexpr unsigned int   IMU_SAMPLE_PERIOD = 1000 / IMU_SAMPLE_RATE;

    //Barometer BME280 settings
    static miosix::GpioPin PRESS_CS(GPIOA_BASE, 6);   
    static BME280::StandbyTime PRESS_SAMPLE_PERIOD = BME280::STB_TIME_0_5; 
}