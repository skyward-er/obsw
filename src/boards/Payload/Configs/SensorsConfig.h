/* Copyright (c) 2023 Skyward Experimental Rocketry
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
#pragma once

#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/UBXGPS/UBXGPSSpi.h>

namespace Payload
{
namespace SensorsConfig
{
constexpr unsigned int NUMBER_OF_SENSORS  = 8;
constexpr uint32_t MAG_CALIBRATION_PERIOD = 100;  // [ms]

// BMX160
constexpr unsigned int BMX160_CALIBRATION_DURATION = 2000;
constexpr Boardcore::BMX160Config::AccelerometerRange BMX160_ACC_FSR_ENUM =
    Boardcore::BMX160Config::AccelerometerRange::G_16;
constexpr Boardcore::BMX160Config::GyroscopeRange BMX160_GYRO_FSR_ENUM =
    Boardcore::BMX160Config::GyroscopeRange::DEG_1000;
constexpr unsigned int BMX160_ACC_GYRO_ODR = 400;
constexpr Boardcore::BMX160Config::OutputDataRate BMX160_ACC_GYRO_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_400;
constexpr unsigned int BMX160_MAG_ODR = 100;
constexpr Boardcore::BMX160Config::OutputDataRate BMX160_MAG_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_100;

constexpr unsigned int BMX160_TEMP_DIVIDER = 1;

constexpr unsigned int BMX160_FIFO_WATERMARK = 40;

// UNUSED CONSTANTS
constexpr unsigned int BMX160_FIFO_HEADER_SIZE = 1;
constexpr unsigned int BMX160_ACC_DATA_SIZE    = 6;
constexpr unsigned int BMX160_GYRO_DATA_SIZE   = 6;
constexpr unsigned int BMX160_MAG_DATA_SIZE    = 8;

// UNUSED - How many bytes go into the fifo each second
constexpr unsigned int BMX160_FIFO_FILL_RATE =
    BMX160_ACC_GYRO_ODR * (BMX160_FIFO_HEADER_SIZE + BMX160_ACC_DATA_SIZE +
                           BMX160_GYRO_DATA_SIZE) +
    BMX160_MAG_ODR * (BMX160_MAG_DATA_SIZE + BMX160_FIFO_HEADER_SIZE);

// UNUSED - How long does it take for the bmx fifo to fill up
constexpr unsigned int BMX160_FIFO_FILL_TIME =
    1024 * 1000 / BMX160_FIFO_FILL_RATE;

// Axis rotation
static const Boardcore::AxisOrthoOrientation BMX160_AXIS_ROTATION = {
    Boardcore::Direction::NEGATIVE_Y, Boardcore::Direction::NEGATIVE_Z};

// LIS magnetometer
constexpr Boardcore::LIS3MDL::ODR MAG_LIS_ODR = Boardcore::LIS3MDL::ODR_80_HZ;
constexpr Boardcore::LIS3MDL::FullScale MAG_LIS_FULLSCALE =
    Boardcore::LIS3MDL::FS_4_GAUSS;

// ADS1118 and connected sensors
constexpr Boardcore::ADS1118::ADS1118Mux ADS1118_CH_STATIC_PORT =
    Boardcore::ADS1118::MUX_AIN0_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADS1118_DR_STATIC_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADS1118_PGA_STATIC_PORT =
    Boardcore::ADS1118::FSR_6_144;

constexpr Boardcore::ADS1118::ADS1118Mux ADS1118_CH_PITOT_PORT =
    Boardcore::ADS1118::MUX_AIN1_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADS1118_DR_PITOT_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADS1118_PGA_PITOT_PORT =
    Boardcore::ADS1118::FSR_6_144;

constexpr Boardcore::ADS1118::ADS1118Mux ADS1118_CH_DPL_PORT =
    Boardcore::ADS1118::MUX_AIN2_GND;
constexpr Boardcore::ADS1118::ADS1118DataRate ADS1118_DR_DPL_PORT =
    Boardcore::ADS1118::DR_860;
constexpr Boardcore::ADS1118::ADS1118Pga ADS1118_PGA_DPL_PORT =
    Boardcore::ADS1118::FSR_6_144;

// MS5803 barometer
constexpr unsigned int MS5803_TEMP_DIVIDER = 5;

// GPS
static constexpr int UBXGPS_BAUD_RATE            = 460800;
static constexpr unsigned int UBXGPS_SAMPLE_RATE = 10;

// Internal ADC & Battery Voltage
constexpr Boardcore::InternalADC::Channel ADC_BATTERY_VOLTAGE =
    Boardcore::InternalADC::Channel::CH5;
constexpr float BATTERY_VOLTAGE_COEFF = 5.98;

// Sampling periods [ms]

// BMX160_SAMPLE_PERIOD: Sample before the fifo is full, but slightly after the
// watermark level (watermark + 30) ---> high slack due to scheduler
// imprecision, avoid clearing the fifo before the interrupt
constexpr unsigned int BMX160_SAMPLE_PERIOD =
    BMX160_FIFO_FILL_TIME * (BMX160_FIFO_WATERMARK + 30) * 4 / 1024;  // [ms]
constexpr unsigned int LIS3MDL_SAMPLE_PERIOD = 15;                    // [ms]
constexpr unsigned int MS5803_SAMPLE_PERIOD  = 15;                    // [ms]
constexpr unsigned int UBXGPS_SAMPLE_PERIOD =
    1000 / UBXGPS_SAMPLE_RATE;                              // [ms]
constexpr unsigned int ADS1118_SAMPLE_PERIOD       = 6;     // [ms]
constexpr unsigned int INTERNAL_ADC_SAMPLE_PERIOD  = 1000;  // [ms]
constexpr unsigned int INTERNAL_TEMP_SAMPLE_PERIOD = 2000;  // [ms]

}  // namespace SensorsConfig
}  // namespace Payload
