/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <units/Frequency.h>

#include <string>

namespace Main
{

namespace Config
{

namespace Sensors
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr int CALIBRATION_SAMPLES_COUNT       = 20;
constexpr unsigned int CALIBRATION_SLEEP_TIME = 100;  // [ms]

// Minimum lps28dfw in order for the measure to be considered valid and
// compensate other sensors
constexpr float ATMOS_CALIBRATION_THRESH = 50000.0;  // [Pa]

constexpr Hertz MAG_CALIBRATION_RATE = 50_hz;
static const std::string MAG_CALIBRATION_FILENAME{"/sd/magCalibration.csv"};

namespace LPS22DF
{
constexpr Boardcore::LPS22DF::AVG AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR ODR = Boardcore::LPS22DF::ODR_100;

constexpr Hertz RATE   = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LPS22DF

namespace LPS28DFW
{
constexpr Boardcore::LPS28DFW::FullScaleRange FS = Boardcore::LPS28DFW::FS_1260;
constexpr Boardcore::LPS28DFW::AVG AVG           = Boardcore::LPS28DFW::AVG_4;
constexpr Boardcore::LPS28DFW::ODR ODR           = Boardcore::LPS28DFW::ODR_100;

constexpr Hertz RATE   = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LPS28DFW

namespace H3LIS331DL
{
constexpr Boardcore::H3LIS331DLDefs::OutputDataRate ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange FS =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace H3LIS331DL

namespace LIS2MDL
{
constexpr Boardcore::LIS2MDL::ODR ODR = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr unsigned int TEMP_DIVIDER   = 10;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LIS2MDL

namespace UBXGPS
{
constexpr Hertz RATE   = 10_hz;
constexpr bool ENABLED = true;
}  // namespace UBXGPS

namespace LSM6DSRX
{
constexpr Boardcore::LSM6DSRXConfig::ACC_FULLSCALE ACC_FS =
    Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr Boardcore::LSM6DSRXConfig::ACC_ODR ACC_ODR =
    Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Boardcore::LSM6DSRXConfig::GYR_FULLSCALE GYR_FS =
    Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000;
constexpr Boardcore::LSM6DSRXConfig::GYR_ODR GYR_ODR =
    Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LSM6DSRX

namespace VN100
{
constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace VN100

namespace ADS131M08
{
constexpr Boardcore::ADS131M08Defs::OversamplingRatio OSR =
    Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

// ADC channels definitions for various sensors
constexpr Boardcore::ADS131M08Defs::Channel STATIC_PRESSURE_1_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_0;
constexpr Boardcore::ADS131M08Defs::Channel STATIC_PRESSURE_2_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_1;
constexpr Boardcore::ADS131M08Defs::Channel DPL_BAY_PRESSURE_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_2;

// These values have been calibrated via vacuum chamber, by looking at LPS28DFW
// output and analog sensor output.
constexpr float CHANNEL_0_SCALE = 4.21662235677003f;
constexpr float CHANNEL_1_SCALE = 4.21662235677003f;
constexpr float CHANNEL_2_SCALE = 4.21662235677003f;

constexpr float STATIC_PRESSURE_1_SCALE = CHANNEL_0_SCALE;
constexpr float STATIC_PRESSURE_2_SCALE = CHANNEL_1_SCALE;
constexpr float DPL_BAY_PRESSURE_SCALE  = CHANNEL_2_SCALE;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADS131M08

namespace InternalADC
{
constexpr Boardcore::InternalADC::Channel VBAT_CH =
    Boardcore::InternalADC::Channel::CH8;
constexpr Boardcore::InternalADC::Channel CAM_VBAT_CH =
    Boardcore::InternalADC::Channel::CH9;

constexpr float VBAT_SCALE     = 7500.0f / 2400.0f;
constexpr float CAM_VBAT_SCALE = 7500.0f / 2400.0f;

constexpr Hertz RATE   = 10_hz;
constexpr bool ENABLED = true;
}  // namespace InternalADC

namespace IMU
{
constexpr bool USE_CALIBRATED_LIS2MDL  = true;
constexpr bool USE_CALIBRATED_LSM6DSRX = true;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace IMU

namespace Atmos
{

// Setting this to true changes the atmospheric pressure to be the static
// pressure port number 2
constexpr bool USE_PORT_2 = false;

}  // namespace Atmos

}  // namespace Sensors

}  // namespace Config

}  // namespace Main
