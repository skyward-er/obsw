/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Fabrizio Monti
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
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace LPS22DF
{
constexpr auto AVG = Boardcore::LPS22DF::AVG_4;
constexpr auto ODR = Boardcore::LPS22DF::ODR_100;

constexpr auto RATE    = 50_hz;  // [ms] 50Hz
constexpr bool ENABLED = true;
}  // namespace LPS22DF

namespace H3LIS331DL
{
constexpr auto ODR     = Boardcore::H3LIS331DLDefs::OutputDataRate::ODR_400;
constexpr auto FS      = Boardcore::H3LIS331DLDefs::FullScaleRange::FS_100;
constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace H3LIS331DL

namespace LIS2MDL
{
constexpr auto ODR          = Boardcore::LIS2MDL::ODR_100_HZ;
constexpr auto TEMP_DIVIDER = 10;

constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LIS2MDL

namespace LSM6DSRX
{
constexpr auto ACC_FS  = Boardcore::LSM6DSRXConfig::ACC_FULLSCALE::G16;
constexpr auto ACC_ODR = Boardcore::LSM6DSRXConfig::ACC_ODR::HZ_416;
constexpr auto ACC_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto GYR_FS  = Boardcore::LSM6DSRXConfig::GYR_FULLSCALE::DPS_2000;
constexpr auto GYR_ODR = Boardcore::LSM6DSRXConfig::GYR_ODR::HZ_416;
constexpr auto GYR_OP_MODE =
    Boardcore::LSM6DSRXConfig::OPERATING_MODE::HIGH_PERFORMANCE;

constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace LSM6DSRX

namespace ADS131M08
{
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

using namespace Boardcore::ADS131M08Defs;

constexpr auto N2_TANK_PT_CHANNEL          = Channel::CHANNEL_4;
constexpr auto REGULATOR_OUT_PT_CHANNEL    = Channel::CHANNEL_5;
constexpr auto OX_TANK_TOP_PT_CHANNEL      = Channel::CHANNEL_2;
constexpr auto OX_TANK_BOTTOM_0_PT_CHANNEL = Channel::CHANNEL_1;
constexpr auto OX_TANK_BOTTOM_1_PT_CHANNEL = Channel::CHANNEL_0;
constexpr auto CC_PT_CHANNEL               = Channel::CHANNEL_3;

constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADS131M08

namespace Trafag
{
// Default shunt resistance, used before calibration or if it's out of bounds
constexpr float DEFAULT_SHUNT_RESISTANCE = 29.5;
// Bounds of the shunt resistance, outside of which the calibration is ignored
constexpr float SHUNT_RESISTANCE_LOWER_BOUND = 28.5;
constexpr float SHUNT_RESISTANCE_UPPER_BOUND = 30.0;

constexpr auto CALIBRATE_SAMPLE_COUNT         = 10;
constexpr auto CALIBRATE_WAIT_BETWEEN_SAMPLES = 100ms;

// Current at 0 bar relative to atmospheric pressure
constexpr float MIN_CURRENT = 4;  // [mA]
// Current at MAX_PRESSURE bar relative to atmospheric pressure
constexpr float MAX_CURRENT = 20;  // [mA]

constexpr float N2_TANK_MAX_PRESSURE          = 400;  // bar
constexpr float REGULATOR_OUT_MAX_PRESSURE    = 100;  // bar
constexpr float OX_TANK_TOP_MAX_PRESSURE      = 100;  // bar
constexpr float OX_TANK_BOTTOM_0_MAX_PRESSURE = 100;  // bar
constexpr float OX_TANK_BOTTOM_1_MAX_PRESSURE = 100;  // bar
constexpr float CC_MAX_PRESSURE               = 40;   // bar
}  // namespace Trafag

namespace Kulite
{
// TODO: This needs to be properly calibrated
// constexpr float TANK_P0_VOLTAGE = 0.3917f;
// constexpr float TANK_P0_TEMP    = 31.0f;
// constexpr float TANK_P1_VOLTAGE = 0.4043f;
// constexpr float TANK_P1_TEMP    = 38.0f;
constexpr float TANK_P0_VOLTAGE = 0.2834f;
constexpr float TANK_P0_TEMP    = 12.0f;
constexpr float TANK_P1_VOLTAGE = 0.4034f;
constexpr float TANK_P1_TEMP    = 34.0f;

}  // namespace Kulite

namespace MAX31856
{
constexpr auto PERIOD  = 10_hz;
constexpr bool ENABLED = true;
}  // namespace MAX31856

namespace InternalADC
{
constexpr auto CURRENT_CH = Boardcore::InternalADC::Channel::CH9;
constexpr auto VBAT_CH    = Boardcore::InternalADC::Channel::CH14;

// Voltage divider with an offset: (R1 + R2) / R1 * 5
constexpr float CURRENT_SCALE  = (7500.0 + 5100.0) / 7500.0 * 5.0;
constexpr float CURRENT_OFFSET = -0.5 * 5.0;

constexpr float VBAT_SCALE = 9040.0 / 3000.0;

constexpr auto RATE    = 10_hz;
constexpr bool ENABLED = true;
}  // namespace InternalADC

}  // namespace Sensors

}  // namespace Config

}  // namespace Motor
