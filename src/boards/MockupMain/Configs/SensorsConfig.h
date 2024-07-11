/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Angelo Prete
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
#include <units/Frequency.h>

#include <chrono>

namespace MockupMain
{
namespace SensorsConfig
{

// clang-format off
// Indent to avoid the linter complaining about using namespace
  using namespace Boardcore::Units::Frequency;
  using namespace std::chrono_literals;
// clang-format on

constexpr unsigned int NUMBER_OF_SENSORS  = 7;
constexpr uint32_t MAG_CALIBRATION_PERIOD = 100;  // [ms]

// BMX160
constexpr unsigned int BMX160_CALIBRATION_DURATION = 2000;
constexpr Boardcore::BMX160Config::AccelerometerRange BMX160_ACC_FSR_ENUM =
    Boardcore::BMX160Config::AccelerometerRange::G_16;
constexpr Boardcore::BMX160Config::GyroscopeRange BMX160_GYRO_FSR_ENUM =
    Boardcore::BMX160Config::GyroscopeRange::DEG_1000;
constexpr unsigned int BMX160_ACC_GYRO_ODR = 100;
constexpr Boardcore::BMX160Config::OutputDataRate BMX160_ACC_GYRO_ODR_ENUM =
    Boardcore::BMX160Config::OutputDataRate::HZ_100;
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
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_1000;
constexpr Boardcore::H3LIS331DLDefs::BlockDataUpdate H3LIS331DL_BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange H3LIS331DL_FSR =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
constexpr auto H3LIS331DL_PERIOD = 800_hz;

// LPS22DF
constexpr Boardcore::LPS22DF::AVG LPS22DF_AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR LPS22DF_ODR = Boardcore::LPS22DF::ODR_50;
constexpr auto LPS22DF_PERIOD                 = 20_hz;

// UBXGPS
constexpr auto UBXGPS_SAMPLE_RATE = 5;
// The +5 is needed because GPS data must be read faster than it is produced (to
// not cause delays)
constexpr auto UBXGPS_PERIOD = 10_hz;

// ADS
constexpr Boardcore::ADS131M08Defs::OversamplingRatio
    ADS131M08_OVERSAMPLING_RATIO =
        Boardcore::ADS131M08Defs::OversamplingRatio::OSR_4096;
constexpr bool ADS131M08_GLOBAL_CHOP_MODE = true;
constexpr auto ADS131M08_PERIOD           = 1000_hz;

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

// BMX160_SAMPLE_PERIOD: Sample before the fifo is full, but slightly after the
// watermark level (watermark + 30) ---> high slack due to scheduler
// imprecision, avoid clearing the fifo before the interrupt
constexpr auto BMX160_SAMPLE_PERIOD = std::chrono::milliseconds(
    BMX160_FIFO_FILL_TIME * (BMX160_FIFO_WATERMARK + 30) * 4 / 1024);  // [ms]
constexpr auto LIS3MDL_SAMPLE_PERIOD       = 6_hz;
constexpr auto INTERNAL_ADC_SAMPLE_PERIOD  = 1_hz;
constexpr auto INTERNAL_TEMP_SAMPLE_PERIOD = 2_hz;

// LoadCell
constexpr auto LOAD_CELL_SAMPLE_PERIOD = 1000_hz;
constexpr Boardcore::ADS131M08Defs::Channel LOAD_CELL_ADC_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_0;
// TODO set calibration
static constexpr float LOAD_CELL_P0_VOLTAGE = -488.47e-6;
static constexpr float LOAD_CELL_P0_MASS    = 3.272;
static constexpr float LOAD_CELL_P1_VOLTAGE = -2.24e-6;
static constexpr float LOAD_CELL_P1_MASS    = 30.233;

}  // namespace SensorsConfig
}  // namespace MockupMain
