/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Basso
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
#include <sensors/LIS3MDL/LIS3MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/calibration/AxisOrientation.h>
#include <units/Frequency.h>

#include <chrono>

namespace Parafoil
{
namespace Config
{
namespace Sensors
{

/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono_literals;

namespace BMX160
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;

constexpr auto CALIBRATION_DURATION = 2000ms;
constexpr auto ACC_FSR      = Boardcore::BMX160Config::AccelerometerRange::G_16;
constexpr auto GYRO_FSR     = Boardcore::BMX160Config::GyroscopeRange::DEG_1000;
constexpr auto ACC_GYRO_ODR = Boardcore::BMX160Config::OutputDataRate::HZ_200;
constexpr auto MAG_ODR      = Boardcore::BMX160Config::OutputDataRate::HZ_100;

static const Boardcore::AxisOrthoOrientation AXIS_ORIENTATION = {
    Boardcore::Direction::NEGATIVE_Z, Boardcore::Direction::POSITIVE_Y};

constexpr auto TEMP_DIVIDER   = 1000;
constexpr auto FIFO_WATERMARK = 40;

constexpr auto FIFO_HEADER_SIZE = 1;
constexpr auto ACC_DATA_SIZE    = 6;
constexpr auto GYRO_DATA_SIZE   = 6;
constexpr auto MAG_DATA_SIZE    = 8;
}  // namespace BMX160

namespace H3LIS331DL
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto ODR = Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr auto BDU =
    Boardcore::H3LIS331DLDefs::BlockDataUpdate::BDU_CONTINUOS_UPDATE;
constexpr auto FSR = Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
}  // namespace H3LIS331DL

namespace LPS22DF
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 50_hz;
constexpr auto AVG           = Boardcore::LPS22DF::AVG_4;
constexpr auto ODR           = Boardcore::LPS22DF::ODR_100;
}  // namespace LPS22DF

namespace UBXGPS
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 10_hz;
}  // namespace UBXGPS

namespace ADS131M08
{
constexpr auto ENABLED       = false;  // NOTICE: disabled due to broken sensor
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto OVERSAMPLING_RATIO =
    Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE = true;
}  // namespace ADS131M08

namespace LIS3MDL
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto ODR           = Boardcore::LIS3MDL::ODR_80_HZ;
constexpr auto FSR           = Boardcore::LIS3MDL::FS_4_GAUSS;
}  // namespace LIS3MDL

namespace InternalADC
{
constexpr auto ENABLED       = true;
constexpr auto SAMPLING_RATE = 100_hz;
constexpr auto VBAT_CH       = Boardcore::InternalADC::Channel::CH5;
constexpr auto VBAT_COEFF    = (150 + 40.2) / 40.2;
}  // namespace InternalADC

namespace MagCalibration
{
constexpr auto FILE_ENABLED     = true;
constexpr auto SAMPLING_RATE    = 10_hz;
constexpr auto CALIBRATION_PATH = "/sd/bmx160_magnetometer_calibration.csv";
}  // namespace MagCalibration

}  // namespace Sensors

}  // namespace Config

}  // namespace Parafoil
