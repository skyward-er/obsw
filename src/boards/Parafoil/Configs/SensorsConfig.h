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

#include <drivers/adc/InternalADC.h>
#include <sensors/ADS131M08/ADS131M08Defs.h>
#include <sensors/BMX160/BMX160Config.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
// TODO change with H3LIS331DLdef when merge request is merged
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/calibration/AxisOrientation.h>

namespace Parafoil
{
namespace SensorsConfig
{
constexpr unsigned int NUMBER_OF_SENSORS  = 9;
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

constexpr unsigned int BMX160_TEMP_DIVIDER = 1000;

constexpr unsigned int BMX160_FIFO_WATERMARK = 40;

// UNUSED CONSTANTS
constexpr unsigned int BMX160_FIFO_HEADER_SIZE = 1;
constexpr unsigned int BMX160_ACC_DATA_SIZE    = 6;
constexpr unsigned int BMX160_GYRO_DATA_SIZE   = 6;
constexpr unsigned int BMX160_MAG_DATA_SIZE    = 8;

// H3LIS331DL
constexpr Boardcore::H3LIS331DLDefs::OutputDataRate H3LIS331DL_ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr Boardcore::H3LIS331DLDefs::BlockDataUpdate H3LIS331DL_BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange H3LIS331DL_FSR =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
constexpr uint32_t H3LIS331DL_PERIOD = 10;  // [ms] 100Hz

// LPS22DF
constexpr Boardcore::LPS22DF::AVG LPS22DF_AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR LPS22DF_ODR = Boardcore::LPS22DF::ODR_100;
constexpr uint32_t LPS22DF_PERIOD             = 20;  // [ms] 50Hz

// UBXGPS
constexpr uint8_t UBXGPS_SAMPLE_RATE = 10;
// The +5 is needed because GPS data must be read faster than it is produced (to
// not cause delays)
constexpr uint32_t UBXGPS_PERIOD = 1000 / (UBXGPS_SAMPLE_RATE + 5);  // [ms]

// ADS
constexpr Boardcore::ADS131M08Defs::OversamplingRatio
    ADS131M08_OVERSAMPLING_RATIO =
        Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool ADS131M08_GLOBAL_CHOP_MODE = true;
constexpr uint32_t ADS131M08_PERIOD       = 10;  // [ms] 100Hz

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

// Internal ADC & Battery Voltage
constexpr Boardcore::InternalADC::Channel ADC_BATTERY_VOLTAGE =
    Boardcore::InternalADC::Channel::CH5;
// Internal ADC voltage divider
constexpr float BATTERY_VOLTAGE_COEFF = (150 + 40.2) / 40.2;

// Sampling periods [ms]

// BMX160_SAMPLE_PERIOD: Sample before the fifo is full, but slightly after the
// watermark level (watermark + 30) ---> high slack due to scheduler
// imprecision, avoid clearing the fifo before the interrupt
constexpr unsigned int BMX160_SAMPLE_PERIOD =
    BMX160_FIFO_FILL_TIME * (BMX160_FIFO_WATERMARK + 30) * 4 / 1024;  // [ms]
constexpr unsigned int LIS3MDL_SAMPLE_PERIOD       = 15;
constexpr unsigned int INTERNAL_ADC_SAMPLE_PERIOD  = 1000;  // [ms]
constexpr unsigned int INTERNAL_TEMP_SAMPLE_PERIOD = 2000;  // [ms]

}  // namespace SensorsConfig
}  // namespace Parafoil
