/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Pietro Bortolus
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
 * Manual IMU sensor selection.
 *   default: uses the VN100 as main IMU;
 *  'DUAL_LSM6': uses the LSM6DSRX (double sensors) as IMU (backup)
 */
// #define DUAL_LSM6
#if defined(DUAL_LSM6)
#warning "DUAL_LSM6 is being used instead of VN100 :)"
#endif

#include <drivers/adc/InternalADC.h>
#include <sensors/AS5047D/AS5047DSPI.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <units/Frequency.h>

#include <chrono>
#include <string>

namespace Main
{

namespace Config
{

namespace Sensors
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto CALIBRATION_SAMPLES_COUNT = 20;
constexpr auto CALIBRATION_SLEEP_TIME    = 100ms;

constexpr auto MAG_CALIBRATION_RATE = 50_hz;
static const std::string MAG_CALIBRATION_FILENAME{"/sd/magCalibration.csv"};

namespace AS5047D_LEFT
{
constexpr auto DATA_SELECT = Boardcore::AS5047DDefs::DataSelect::DAECANG;
constexpr auto DAEC_EN     = Boardcore::AS5047DDefs::DAECStatus::DAEC_ON;
constexpr auto ROTATION_DIRECTION =
    Boardcore::AS5047DDefs::ABIRotationDirection::NORMAL;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace AS5047D_LEFT

namespace AS5047D_RIGHT
{
constexpr auto DATA_SELECT = Boardcore::AS5047DDefs::DataSelect::DAECANG;
constexpr auto DAEC_EN     = Boardcore::AS5047DDefs::DAECStatus::DAEC_ON;
constexpr auto ROTATION_DIRECTION =
    Boardcore::AS5047DDefs::ABIRotationDirection::NORMAL;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace AS5047D_RIGHT

namespace AS5047D_ABK
{
constexpr auto DATA_SELECT = Boardcore::AS5047DDefs::DataSelect::DAECANG;
constexpr auto DAEC_EN     = Boardcore::AS5047DDefs::DAECStatus::DAEC_ON;
constexpr auto ROTATION_DIRECTION =
    Boardcore::AS5047DDefs::ABIRotationDirection::NORMAL;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace AS5047D_ABK

namespace LIS2MDL_RCS
{
constexpr auto ODR          = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr auto TEMP_DIVIDER = 10U;

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace LIS2MDL_RCS
namespace LPS22DF
{
constexpr auto AVG = Boardcore::LPS22DF::AVG_4;
constexpr auto ODR = Boardcore::LPS22DF::ODR_100;

constexpr auto RATE    = 50_hz;
constexpr auto ENABLED = true;
}  // namespace LPS22DF

namespace H3LIS331DL
{
constexpr auto ODR = Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr auto FS  = Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace H3LIS331DL

namespace UBXGPS
{
constexpr auto RATE    = 10_hz;
constexpr auto ENABLED = true;
}  // namespace UBXGPS

namespace LIS2MDL_INT
{
constexpr auto ODR          = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr auto TEMP_DIVIDER = 10U;

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace LIS2MDL_INT

namespace LSM6DSRX_LOW
{
constexpr auto ACC_CALIBRATION_FILENAME  = "/sd/accCalibration0.csv";
constexpr auto GYRO_CALIBRATION_FILENAME = "/sd/gyroCalibration0.csv";

constexpr auto ACC_FS  = Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr auto ACC_ODR = Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr auto ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto GYR_FS  = Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000;
constexpr auto GYR_ODR = Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
constexpr auto GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace LSM6DSRX_LOW

namespace LSM6DSRX_HIGH
{
constexpr auto ACC_CALIBRATION_FILENAME  = "/sd/accCalibration1.csv";
constexpr auto GYRO_CALIBRATION_FILENAME = "/sd/gyroCalibration1.csv";

constexpr auto ACC_FS  = Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G4;
constexpr auto ACC_ODR = Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_104;
constexpr auto ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto GYR_FS  = Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_1000;
constexpr auto GYR_ODR = Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_104;
constexpr auto GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace LSM6DSRX_HIGH

namespace VN100
{
constexpr auto RATE    = 400_hz;
constexpr auto ENABLED = true;

// TODO
constexpr auto ACC_CALIBRATION_FILENAME  = "/sd/vn100AccCalibration.csv";
constexpr auto GYRO_CALIBRATION_FILENAME = "/sd/vn100GyroCalibration.csv";
}  // namespace VN100

namespace ND015A
{
constexpr auto IOW = Boardcore::ND015A::IOWatchdogEnable::DISABLED;
constexpr auto BWL = Boardcore::ND015A::BWLimitFilter::BWL_200;
constexpr auto NTC = Boardcore::ND015A::NotchEnable::DISABLED;

constexpr uint8_t ODR = 0x00;  // Auto select based on BWL

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace ND015A

namespace InternalADC
{
constexpr auto VBAT_CH     = Boardcore::InternalADC::Channel::CH8;
constexpr auto CAM_VBAT_CH = Boardcore::InternalADC::Channel::CH9;

constexpr auto VBAT_SCALE     = 7500.0f / 2400.0f;
constexpr auto CAM_VBAT_SCALE = 7500.0f / 2400.0f;

constexpr auto RATE    = 10_hz;
constexpr auto ENABLED = true;
}  // namespace InternalADC

namespace IMU
{
constexpr auto USE_CALIBRATED_LIS2MDL  = true;
constexpr auto USE_CALIBRATED_LSM6DSRX = true;
constexpr auto USE_CALIBRATED_VN100    = true;

constexpr auto RATE    = 200_hz;
constexpr auto ENABLED = true;
}  // namespace IMU

namespace Atmos
{
enum class AtmosSensor
{
    SENSOR_0 = 0,
    SENSOR_1 = 1,
    SENSOR_2 = 2
};

// The sensor used for the atmospheric pressure, one of the 3 ND015A sensors
constexpr auto ATMOS_SENSOR = AtmosSensor::SENSOR_0;

}  // namespace Atmos
}  // namespace Sensors
}  // namespace Config
}  // namespace Main
