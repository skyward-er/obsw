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

#include <sensors/ADS131M08/ADS131M08.h>
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LPS28DFW/LPS28DFW.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>

namespace Main
{

namespace Config
{

namespace Sensors
{

namespace LPS22DF
{
static constexpr Boardcore::LPS22DF::AVG AVG = Boardcore::LPS22DF::AVG_4;
static constexpr Boardcore::LPS22DF::ODR ODR = Boardcore::LPS22DF::ODR_100;
static constexpr uint32_t PERIOD             = 20;  // [ms] 50Hz
}  // namespace LPS22DF

namespace LPS28DFW
{
static constexpr Boardcore::LPS28DFW::FullScaleRange FS =
    Boardcore::LPS28DFW::FS_1260;
static constexpr Boardcore::LPS28DFW::AVG AVG = Boardcore::LPS28DFW::AVG_4;
static constexpr Boardcore::LPS28DFW::ODR ODR = Boardcore::LPS28DFW::ODR_100;
static constexpr uint32_t PERIOD              = 20;  // [ms] 50Hz
}  // namespace LPS28DFW

namespace H3LIS331DL
{
static constexpr Boardcore::H3LIS331DLDefs::OutputDataRate ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
static constexpr Boardcore::H3LIS331DLDefs::FullScaleRange FS =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
static constexpr uint32_t PERIOD = 10;  // [ms] 100Hz
}  // namespace H3LIS331DL

namespace LIS2MDL
{
static constexpr Boardcore::LIS2MDL::ODR ODR = Boardcore::LIS2MDL::ODR_100_HZ;
static constexpr unsigned int TEMP_DIVIDER   = 5;
static constexpr uint32_t PERIOD             = 10;  // [ms] 100Hz
}  // namespace LIS2MDL

namespace UBXGPS
{
static constexpr uint32_t PERIOD = 200;  // [ms] 5Hz
}

namespace LSM6DSRX
{
static constexpr Boardcore::LSM6DSRXConfig::ACC_FULLSCALE ACC_FS =
    Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
static constexpr Boardcore::LSM6DSRXConfig::ACC_ODR ACC_ODR =
    Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
static constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

static constexpr Boardcore::LSM6DSRXConfig::GYR_FULLSCALE GYR_FS =
    Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_4000;
static constexpr Boardcore::LSM6DSRXConfig::GYR_ODR GYR_ODR =
    Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
static constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

static constexpr uint32_t PERIOD = 20;  // [ms] 50Hz
}  // namespace LSM6DSRX

namespace ADS131M08
{
static constexpr Boardcore::ADS131M08Defs::OversamplingRatio OSR =
    Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
static constexpr bool GLOBAL_CHOP_MODE_EN = true;
static constexpr uint32_t PERIOD          = 10;  // [ms] 100Hz
}  // namespace ADS131M08

namespace InternalADC
{
static constexpr uint32_t PERIOD = 100;  // [ms] 10Hz

static constexpr Boardcore::InternalADC::Channel VBAT_CH =
    Boardcore::InternalADC::Channel::CH8;
static constexpr Boardcore::InternalADC::Channel CAM_VBAT_CH =
    Boardcore::InternalADC::Channel::CH9;
static constexpr Boardcore::InternalADC::Channel CUTTER_SENSE_CH =
    Boardcore::InternalADC::Channel::CH11;

static constexpr float VBAT_SCALE     = 7500.0f / 2400.0f;
static constexpr float CAM_VBAT_SCALE = 7500.0f / 2400.0f;
}  // namespace InternalADC

}  // namespace Sensors

}  // namespace Config

}  // namespace Main