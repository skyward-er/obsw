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
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <units/Units.h>

namespace Motor
{

namespace Config
{

namespace Sensors
{
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace LPS22DF
{
constexpr Boardcore::LPS22DF::AVG AVG = Boardcore::LPS22DF::AVG_4;
constexpr Boardcore::LPS22DF::ODR ODR = Boardcore::LPS22DF::ODR_100;

constexpr Hertz PERIOD = 50_hz;  // [ms] 50Hz
constexpr bool ENABLED = true;
}  // namespace LPS22DF

namespace H3LIS331DL
{
constexpr Boardcore::H3LIS331DLDefs::OutputDataRate ODR =
    Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr Boardcore::H3LIS331DLDefs::FullScaleRange FS =
    Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
constexpr Hertz PERIOD = 100_hz;
constexpr bool ENABLED = true;
}  // namespace H3LIS331DL

namespace LIS2MDL
{
constexpr Boardcore::LIS2MDL::ODR ODR = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr unsigned int TEMP_DIVIDER   = 5;
constexpr Hertz PERIOD                = 100_hz;
constexpr bool ENABLED                = true;
}  // namespace LIS2MDL

namespace LSM6DSRX
{
constexpr Boardcore::LSM6DSRXConfig::ACC_FULLSCALE ACC_FS =
    Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr Boardcore::LSM6DSRXConfig::ACC_ODR ACC_ODR =
    Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Boardcore::LSM6DSRXConfig::GYR_FULLSCALE GYR_FS =
    Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_4000;
constexpr Boardcore::LSM6DSRXConfig::GYR_ODR GYR_ODR =
    Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
constexpr Boardcore::LSM6DSRXConfig::OPERATING_MODE GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr Hertz PERIOD = 50_hz;
constexpr bool ENABLED = true;
}  // namespace LSM6DSRX

namespace ADS131M08
{
constexpr Boardcore::ADS131M08Defs::OversamplingRatio OSR =
    Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

/*
Calibration values for tank sensors board

Channel 6 - 29.79ohm
0.1185V 3.97mA  29.84ohm
0.3575V 12mA    29.79ohm
0.5952V 20.01mA 29.76ohm

Channel 5 - 29.73ohm
0.1192V 4.03mA  29.57ohm
0.3600V 12.01mA 29.97ohm
0.5930V 20mA    29.65ohm

Channel 4 - 29.65ohm
0.1194V 4.02mA  29.70ohm
0.3584V 12.06mA 29.72ohm
0.5911V 20mA    29.55ohm
*/
constexpr float CH4_SHUNT_RESISTANCE = 29.79;
constexpr float CH5_SHUNT_RESISTANCE = 29.7;
constexpr float CH6_SHUNT_RESISTANCE = 29.65;

constexpr Boardcore::ADS131M08Defs::Channel TANK_TC_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_3;
constexpr Boardcore::ADS131M08Defs::Channel TANK_TOP_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_5;
constexpr Boardcore::ADS131M08Defs::Channel TANK_BOTTOM_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_6;
constexpr Boardcore::ADS131M08Defs::Channel ENGINE_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_4;

constexpr Hertz PERIOD = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADS131M08

namespace Trafag
{
constexpr float TANK_TOP_SHUNT_RESISTANCE    = ADS131M08::CH5_SHUNT_RESISTANCE;
constexpr float TANK_BOTTOM_SHUNT_RESISTANCE = ADS131M08::CH6_SHUNT_RESISTANCE;
constexpr float ENGINE_SHUNT_RESISTANCE      = ADS131M08::CH4_SHUNT_RESISTANCE;

constexpr float MIN_CURRENT = 4;
constexpr float MAX_CURRENT = 20;

constexpr float TANK_TOP_MAX_PRESSURE    = 100;  // bar
constexpr float TANK_BOTTOM_MAX_PRESSURE = 100;  // bar
// TODO: THIS NEEDS TO CHANGE FOR A FLIGHT CONFIGURATION!
constexpr float ENGINE_MAX_PRESSURE = 100;  // bar
}  // namespace Trafag

namespace InternalADC
{
constexpr Boardcore::InternalADC::Channel VBAT_CH =
    Boardcore::InternalADC::Channel::CH14;

constexpr float VBAT_SCALE = 9040.0f / 3000.0f;

constexpr Hertz PERIOD = 10_hz;
constexpr bool ENABLED = true;
}  // namespace InternalADC

}  // namespace Sensors

}  // namespace Config

}  // namespace Motor
