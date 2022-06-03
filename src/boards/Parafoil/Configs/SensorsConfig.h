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

#include <sensors/BME280/BME280.h>
#include <sensors/MPU9250/MPU9250.h>

namespace Parafoil
{

// SPI pinouts
static miosix::GpioPin SCK(GPIOA_BASE, 5);
static miosix::GpioPin MISO(GPIOB_BASE, 4);
static miosix::GpioPin MOSI(GPIOA_BASE, 7);

// GPS settings
// static miosix::GpioPin GPS_CS(GPIOG_BASE, 3);
static miosix::GpioPin GPS_TX(GPIOD_BASE, 5);
static miosix::GpioPin GPS_RX(GPIOA_BASE, 3);
static constexpr unsigned int GPS_SAMPLE_RATE   = 10;
static constexpr unsigned int GPS_SAMPLE_PERIOD = 1000 / GPS_SAMPLE_RATE;

// IMU MPU9250 settings
static miosix::GpioPin IMU_CS(GPIOB_BASE, 2);
static const Boardcore::MPU9250::MPU9250GyroFSR IMU_GYRO_SCALE =
    Boardcore::MPU9250::GYRO_FSR_500DPS;
static const Boardcore::MPU9250::MPU9250AccelFSR IMU_ACCEL_SCALE =
    Boardcore::MPU9250::ACCEL_FSR_16G;
static constexpr unsigned short IMU_SAMPLE_RATE = 500;
static constexpr unsigned int IMU_SAMPLE_PERIOD = 1000 / IMU_SAMPLE_RATE;

// Barometer BME280 settings
static miosix::GpioPin PRESS_CS(GPIOC_BASE, 11);
static const Boardcore::BME280::StandbyTime PRESS_SAMPLE_RATE =
    Boardcore::BME280::STB_TIME_0_5;
static constexpr unsigned int PRESS_SAMPLE_PERIOD = 20;

}  // namespace Parafoil
