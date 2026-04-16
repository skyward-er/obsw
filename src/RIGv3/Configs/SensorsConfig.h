/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Authors: Niccolò Betto, Pietro Bortolus
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
#include <units/Frequency.h>

#include <chrono>
#include <cstdint>

namespace RIGv3
{

namespace Config
{

namespace Sensors
{

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace ADS131M08_FAST
{
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_4096;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

constexpr Hertz PERIOD = 1000_hz;
}  // namespace ADS131M08_FAST

namespace ADS131M08_SLOW
{
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_16256;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

constexpr Hertz PERIOD = 100_hz;
}  // namespace ADS131M08_SLOW

namespace ADC_0
{
constexpr bool ENABLED = true;

using Channel = Boardcore::ADS131M08Defs::Channel;

constexpr auto PRZ_VESSEL_1_PT_CHANNEL = Channel::CHANNEL_0;
constexpr auto PRZ_VESSEL_2_PT_CHANNEL = Channel::CHANNEL_1;
constexpr auto PRZ_FILLING_PT_CHANNEL  = Channel::CHANNEL_2;
constexpr auto OX_VESSEL_PT_CHANNEL    = Channel::CHANNEL_3;
constexpr auto OX_FILLING_PT_CHANNEL   = Channel::CHANNEL_4;
constexpr auto OX_VESSEL_LC_CHANNEL    = Channel::CHANNEL_5;

constexpr float CH0_SHUNT_RESISTANCE = 29.283f;
constexpr float CH1_SHUNT_RESISTANCE = 29.233f;
constexpr float CH2_SHUNT_RESISTANCE = 29.268f;
constexpr float CH3_SHUNT_RESISTANCE = 29.645f;
constexpr float CH4_SHUNT_RESISTANCE = 29.708f;
constexpr float CH5_SHUNT_RESISTANCE = 29.708f;
}  // namespace ADC_0

namespace ADC_1
{
constexpr bool ENABLED = true;

}  // namespace ADC_1

namespace ADC_2
{
constexpr bool ENABLED = true;

using Channel = Boardcore::ADS131M08Defs::Channel;

constexpr auto PRZ_TANK_PT_CHANNEL        = Channel::CHANNEL_0;
constexpr auto OX_REG_OUT_PT_CHANNEL      = Channel::CHANNEL_1;
constexpr auto FUEL_REG_OUT_PT_CHANNEL    = Channel::CHANNEL_2;
constexpr auto OX_TANK_PT_CHANNEL         = Channel::CHANNEL_3;
constexpr auto FUEL_TANK_PT_CHANNEL       = Channel::CHANNEL_4;
constexpr auto IGNITER_CHAMBER_PT_CHANNEL = Channel::CHANNEL_5;
constexpr auto MAIN_CHAMBER_PT_CHANNEL    = Channel::CHANNEL_6;

constexpr float CH0_SHUNT_RESISTANCE = 29.283f;
constexpr float CH1_SHUNT_RESISTANCE = 29.233f;
constexpr float CH2_SHUNT_RESISTANCE = 29.268f;
constexpr float CH3_SHUNT_RESISTANCE = 29.645f;
constexpr float CH4_SHUNT_RESISTANCE = 29.708f;
constexpr float CH5_SHUNT_RESISTANCE = 29.708f;
constexpr float CH6_SHUNT_RESISTANCE = 29.708f;

}  // namespace ADC_2

namespace ADC_3
{
constexpr bool ENABLED = true;

using Channel = Boardcore::ADS131M08Defs::Channel;

constexpr auto MAIN_OX_ENCODER_CHANNEL   = Channel::CHANNEL_0;
constexpr auto MAIN_FUEL_ENCODER_CHANNEL = Channel::CHANNEL_1;
constexpr auto PRZ_OX_ENCODER_CHANNEL    = Channel::CHANNEL_2;
constexpr auto PRZ_FUEL_ENCODER_CHANNEL  = Channel::CHANNEL_3;

}  // namespace ADC_3

namespace InternalADC
{
constexpr bool ENABLED = true;
constexpr Hertz PERIOD = 10_hz;
}  // namespace InternalADC

namespace Trafag
{
// Default shunt resistance, used before calibration or if it's out of bounds
constexpr float DEFAULT_SHUNT_RESISTANCE = 49.0f;
// Bounds of the shunt resistance, outside of which the calibration is ignored
constexpr float SHUNT_RESISTANCE_LOWER_BOUND = 46.0f;
constexpr float SHUNT_RESISTANCE_UPPER_BOUND = 51.0f;

constexpr auto CALIBRATE_SAMPLE_COUNT         = 10;
constexpr auto CALIBRATE_WAIT_BETWEEN_SAMPLES = 100ms;

// Current at 0 bar relative to atmospheric pressure
constexpr float MIN_CURRENT = 4.0f;  // [mA]
// Current at MAX_PRESSURE bar relative to atmospheric pressure
constexpr float MAX_CURRENT = 20.0f;  // [mA]

constexpr float PRZ_VESSEL_1_MAX_PRESSURE = 400.0f;  // bar
constexpr float PRZ_VESSEL_2_MAX_PRESSURE = 400.0f;  // bar
constexpr float PRZ_FILLING_MAX_PRESSURE  = 250.0f;  // bar
constexpr float OX_VESSEL_MAX_PRESSURE    = 100.0f;  // bar
constexpr float OX_FILLING_MAX_PRESSURE   = 100.0f;  // bar

constexpr float PRZ_TANK_MAX_PRESSURE        = 400.0f;  // bar
constexpr float REG_OUT_MAX_PRESSURE         = 250.0f;  // bar
constexpr float OX_TANK_MAX_PRESSURE         = 100.0f;  // bar
constexpr float FUEL_TANK_MAX_PRESSURE       = 100.0f;  // bar
constexpr float IGNITER_CHAMBER_MAX_PRESSURE = 40.0f;   // bar
constexpr float MAIN_CHAMBER_MAX_PRESSURE    = 100.0f;  // bar

}  // namespace Trafag

namespace Encoder
{
// Default shunt resistance
constexpr float DEFAULT_SHUNT_RESISTANCE = 49.5f;

constexpr float FULLSCALE_VOLTAGE = 4.7f;

constexpr int SENSOR_RESISTANCE = 31240;  // Ohm
constexpr int CURRENT_GAIN      = 100;

constexpr float MAX_ANGLE = 360.0f;  // degrees

}  // namespace Encoder

namespace LoadCell
{
constexpr auto CALIBRATE_SAMPLE_COUNT  = 10;
constexpr auto CALIBRATE_SAMPLE_PERIOD = 40ms;

/*
// Rocket ramp loadcell calibration data
constexpr float ROCKET_P0_VOLTAGE = -0.0004273;
constexpr float ROCKET_P0_MASS    = 10.005;
constexpr float ROCKET_P1_VOLTAGE = -0.0018125;
constexpr float ROCKET_P1_MASS    = 40.290;
*/

// Static fire teststand loadcell calibration data
// - 0 kg       V: -0.000275
// - 5.072 kg   V: -0.0029
// - 15.249 kg  V: -0.00816
// - 25.300 kg  V: -0.013327
// constexpr float ROCKET_P0_VOLTAGE = -0.0029;
// constexpr float ROCKET_P0_MASS    = 5.072;
// constexpr float ROCKET_P1_VOLTAGE = -0.013327;
// constexpr float ROCKET_P1_MASS    = 25.300;

// New Rocket ramp loadcell calibration data
// Points obtained from measurement on the ramp, through regression. Check
// https://docs.google.com/spreadsheets/d/1nQse2GMNQPRd9KqRvvtfn-WiyyxnLysz/edit?gid=1597496666#gid=1597496666
constexpr float ROCKET_P0_VOLTAGE = -0.001689;
constexpr float ROCKET_P0_MASS    = 10.307;
constexpr float ROCKET_P1_VOLTAGE = -0.007356;
constexpr float ROCKET_P1_MASS    = 45.768;

/* OLD CALIBRATION DATA (before 07/09/2024, before new flipping)
// LC Vessel sensor calibration data
// - 1.866kg V: 0.00027
// - 5.050kg V: 0.00073
// - 6.916kg V: 0.00100000941
constexpr float VESSEL_P0_VOLTAGE = 0.00027;
constexpr float VESSEL_P0_MASS    = 1.866;
constexpr float VESSEL_P1_VOLTAGE = 0.0010;
constexpr float VESSEL_P1_MASS    = 6.916;
*/

constexpr float VESSEL_SCALE  = 1517.689958f;
constexpr float VESSEL_OFFSET = -259.2304283f;

// LC Vessel sensor calibration data (post 07/09/2024)
// - 0kg      V: 0.000630177
// - 4.985kg  V: 0.000470017 (−31125.124875125 kg/v)
// - 10.177kg V: 0.00030895  (−31681,645689808 kg/v)
constexpr float VESSEL_P0_VOLTAGE = 0.000470017;
constexpr float VESSEL_P0_MASS    = 4.985;
constexpr float VESSEL_P1_VOLTAGE = 0.00030895;
constexpr float VESSEL_P1_MASS    = 10.177;

/*
// LC Vessel sensor calibration data (post 08/11/2024, old flipping)
// - 0kg      V: -0.009553
// - 8.720kg  V: -0.010958 (−31125.124875125 kg/v)
constexpr float VESSEL_P0_VOLTAGE = -0.009553;
constexpr float VESSEL_P0_MASS    = 0.0;
constexpr float VESSEL_P1_VOLTAGE = -0.010958;
constexpr float VESSEL_P1_MASS    = 8.720;
*/
}  // namespace LoadCell

}  // namespace Sensors
}  // namespace Config
}  // namespace RIGv3
