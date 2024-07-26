/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor, Emilio Corigliano, Giuseppe Brentino
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
#include <units/Frequency.h>

namespace Main
{

namespace Config
{

namespace Sensors
{

/**
 * Since it isn't used for nas or calibration, the AVG is left the minimum and
 * ODR is left to 100Hz which is the double
 */
namespace LPS22DF
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Boardcore::LPS22DF::AVG AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR ODR = Boardcore::LPS22DF::ODR_100;

constexpr Hertz RATE   = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LPS22DF

/**
 * Higher resolution (lower FSR = 1260hPa)
 *
 * Since it is used for calibration purposes, the AVG is set to 64 but it is ok
 * to set it to 8. The current consumption in the AVG_64 472.8 uA, at AVG_8 is
 * 111.5 uA and AVG_4 is 87.8 uA.
 */
namespace LPS28DFW
{
constexpr Boardcore::LPS28DFW::FullScaleRange FS = Boardcore::LPS28DFW::FS_1260;
constexpr Boardcore::LPS28DFW::AVG AVG           = Boardcore::LPS28DFW::AVG_8;
constexpr Boardcore::LPS28DFW::ODR ODR           = Boardcore::LPS28DFW::ODR_100;

constexpr Hertz RATE   = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LPS28DFW

/**
 * Since the last year we reached a peak of 20g max, we leave the FS to 100 g
 *
 * TODO: discuss with Annalisa. Maybe considering to use up to 400Hz (ODR).
 * Check also the current consumption
 * 200 Hz with 400 ODR
 */
namespace H3LIS331DL
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Boardcore::H3LIS331DLDefs::OutputDataRate ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange FS =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace H3LIS331DL

/**
 * Warning: both ODR and Sampling rate is at 100Hz! Could lead to aliasing
 */
namespace LIS2MDL
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Boardcore::LIS2MDL::ODR ODR = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr unsigned int TEMP_DIVIDER   = 5;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LIS2MDL

/**
 * 10Hz now,
 */
namespace UBXGPS
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz RATE   = 10_hz;
constexpr bool ENABLED = true;
}  // namespace UBXGPS

/**
 * 416 ODR and 100 Hz frequency se serve a recovery (con fifo serve solo in
 * postprocess) 104 ODR and 50 Hz frequency basta per GNC
 *
 * 2000 DPS ma se a roccaraso gira poco possiamo metterlo a 1000 DPS.
 */
namespace LSM6DSRX
{
/* linter off */ using namespace Boardcore::Units::Frequency;

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

namespace ADS131M08
{
/* linter off */ using namespace Boardcore::Units::Frequency;

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

constexpr float CHANNEL_0_SCALE = (38300.0f + 13000.0f) / 13000.0f;
constexpr float CHANNEL_1_SCALE = (38300.0f + 13000.0f) / 13000.0f;
constexpr float CHANNEL_2_SCALE = (38300.0f + 13000.0f) / 13000.0f;

constexpr float STATIC_PRESSURE_1_SCALE = CHANNEL_0_SCALE;
constexpr float STATIC_PRESSURE_2_SCALE = CHANNEL_1_SCALE;
constexpr float DPL_BAY_PRESSURE_SCALE  = CHANNEL_2_SCALE;

constexpr Hertz RATE   = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADS131M08

namespace InternalADC
{
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Boardcore::InternalADC::Channel VBAT_CH =
    Boardcore::InternalADC::Channel::CH8;
constexpr Boardcore::InternalADC::Channel CAM_VBAT_CH =
    Boardcore::InternalADC::Channel::CH9;
constexpr Boardcore::InternalADC::Channel CUTTER_SENSE_CH =
    Boardcore::InternalADC::Channel::CH11;

constexpr float VBAT_SCALE     = 7500.0f / 2400.0f;
constexpr float CAM_VBAT_SCALE = 7500.0f / 2400.0f;

constexpr Hertz RATE   = 10_hz;
constexpr bool ENABLED = true;
}  // namespace InternalADC

/**
 * Same frequency of the accelerometer (lsm6dsrx)
 */
namespace RotatedIMU
{
constexpr uint32_t RATE = LSM6DSRX::RATE;
constexpr bool ENABLED  = true;
}  // namespace RotatedIMU

}  // namespace Sensors

}  // namespace Config

}  // namespace Main