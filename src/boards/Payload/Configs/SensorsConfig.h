/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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
#include <sensors/UBXGPS/UBXGPSSpi.h>
#include <units/Frequency.h>

#include <chrono>

namespace Payload
{
namespace Config
{
namespace Sensors
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono_literals;

namespace LPS22DF
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 50_hz;
constexpr auto AVG           = Boardcore::LPS22DF::AVG_4;
constexpr auto ODR           = Boardcore::LPS22DF::ODR_100;
}  // namespace LPS22DF

namespace LPS28DFW
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto AVG           = Boardcore::LPS28DFW::AVG_4;
constexpr auto ODR           = Boardcore::LPS28DFW::ODR_200;
constexpr auto FSR           = Boardcore::LPS28DFW::FS_1260;
}  // namespace LPS28DFW

namespace H3LIS331DL
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto ODR = Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr auto BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
constexpr auto FSR = Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
}  // namespace H3LIS331DL

namespace LIS2MDL
{
constexpr auto ENABLED             = true;
constexpr auto SAMPLING_RATE       = 100_hz;
constexpr auto ODR                 = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr auto TEMPERATURE_DIVIDER = 10U;
}  // namespace LIS2MDL

namespace UBXGPS
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 10_hz;
}  // namespace UBXGPS

namespace LSM6DSRX
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;
constexpr auto ACC_FS  = Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr auto ACC_ODR = Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr auto GYR_FS  = Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000;
constexpr auto GYR_ODR = Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
}  // namespace LSM6DSRX

namespace ADS131M08
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto OVERSAMPLING_RATIO =
    Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE = true;
}  // namespace ADS131M08

namespace InternalADC
{
constexpr auto ENABLED        = true;
constexpr auto SAMPLING_RATE  = 10_hz;
constexpr auto VBAT_CH        = Boardcore::InternalADC::Channel::CH8;
constexpr auto CAM_VBAT_CH    = Boardcore::InternalADC::Channel::CH9;
constexpr auto VBAT_SCALE     = 7500.0f / 2400.0f;
constexpr auto CAM_VBAT_SCALE = 7500.0f / 2400.0f;
}  // namespace InternalADC

// Scale values for analog pressure sensors have been calibrated via vacuum
// chamber, by looking at LPS28DFW output and analog sensor output

namespace StaticPressure
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = ADS131M08::SAMPLING_RATE;
constexpr auto ADC_CH        = Boardcore::ADS131M08Defs::Channel::CHANNEL_0;
constexpr auto SCALE         = 4.21662235677003f;
}  // namespace StaticPressure

namespace DynamicPressure
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = ADS131M08::SAMPLING_RATE;
constexpr auto ADC_CH        = Boardcore::ADS131M08Defs::Channel::CHANNEL_1;
constexpr auto SCALE         = 4.21662235677003f;
}  // namespace DynamicPressure

namespace RotatedIMU
{
constexpr auto USE_CALIBRATED_LIS2MDL  = true;
constexpr auto USE_CALIBRATED_LSM6DSRX = true;
constexpr auto ENABLED                 = true;
constexpr auto SAMPLING_RATE           = 100_hz;
}  // namespace RotatedIMU

namespace Calibration
{
constexpr auto SAMPLE_COUNT  = 20;
constexpr auto SAMPLE_PERIOD = 100ms;
// Threshold value of the LP28DFW in order to consider the measure valid to use
// as reference for the calibration of the analog sensors
constexpr auto ATMOS_THRESHOLD = 50'000.0f;  // [Pa]
}  // namespace Calibration

namespace MagCalibration
{
constexpr auto FILE_ENABLED     = true;  ///< Load calibration data from SD card
constexpr auto SAMPLING_RATE    = 50_hz;
constexpr auto CALIBRATION_PATH = "/sd/magCalibration.csv";
}  // namespace MagCalibration

}  // namespace Sensors
}  // namespace Config
}  // namespace Payload
