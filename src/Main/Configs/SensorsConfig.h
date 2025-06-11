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

#include <drivers/adc/InternalADC.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <units/Frequency.h>

#include <string>

namespace Main
{

namespace Config
{

namespace Sensors
{
/* linter off */ using namespace Boardcore::Units::Frequency;

// Switches between LPS22DF + LIS2MDL IN/EXT configuration and the dual
// magnetometer configuration, LIS2MDL IN + LIS2MDL EXT.
// The dual mag configuration is used during testing to compare magnetometer
// positioning.
// NOTE: Ensure the configuration pins on the board are soldered accordingly.
constexpr bool USING_DUAL_MAGNETOMETER = false;

constexpr int CALIBRATION_SAMPLES_COUNT       = 20;
constexpr unsigned int CALIBRATION_SLEEP_TIME = 100;  // [ms]

constexpr Hertz MAG_CALIBRATION_RATE = 50_hz;
static const std::string MAG_CALIBRATION_FILENAME{"/sd/magCalibration.csv"};

namespace LPS22DF
{
constexpr Boardcore::LPS22DF::AVG AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR ODR = Boardcore::LPS22DF::ODR_100;

constexpr Hertz RATE   = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LPS22DF

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

namespace LSM6DSRX_0
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
}  // namespace LSM6DSRX_0

namespace LSM6DSRX_1
{
constexpr Boardcore::LSM6DSRXConfig::ACC_FULLSCALE ACC_FS =
    Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G4;
constexpr Boardcore::LSM6DSRXConfig::ACC_ODR ACC_ODR =
    Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_104;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Boardcore::LSM6DSRXConfig::GYR_FULLSCALE GYR_FS =
    Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_1000;
constexpr Boardcore::LSM6DSRXConfig::GYR_ODR GYR_ODR =
    Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_104;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LSM6DSRX_1

namespace VN100
{
constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace VN100

namespace ND015A
{
constexpr Boardcore::ND015A::IOWatchdogEnable IOW =
    Boardcore::ND015A::IOWatchdogEnable::DISABLED;
constexpr Boardcore::ND015A::BWLimitFilter BWL =
    Boardcore::ND015A::BWLimitFilter::BWL_200;
constexpr Boardcore::ND015A::NotchEnable NTC =
    Boardcore::ND015A::NotchEnable::DISABLED;

constexpr uint8_t ODR = 0x1C;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ND015A

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
enum class AtmosSensor
{
    SENSOR_0 = 0,
    SENSOR_1 = 1,
    SENSOR_2 = 2
};

// The sensor used for the atmospheric pressure, one of the 3 ND015A sensors
constexpr AtmosSensor ATMOS_SENSOR = AtmosSensor::SENSOR_0;

}  // namespace Atmos
}  // namespace Sensors
}  // namespace Config
}  // namespace Main
