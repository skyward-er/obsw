/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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
#include <sensors/MAX31856/MAX31856.h>
#include <units/Frequency.h>

#include <chrono>
#include <cstdint>

namespace RIGv2
{

namespace Config
{

namespace Sensors
{

/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace ADS131M08
{
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

constexpr Hertz PERIOD = 100_hz;

// Servo current sensor calibration data
// - A: 0.0 V: 2.520
// - A: 0.5 V: 2.513
// - A: 1.0 V: 2.505
// - A: 1.5 V: 2.498
// - A: 2.0 V: 2.490
// - A: 2.5 V: 2.484
// - A: 5.2 V: 2.441
constexpr float SERVO_CURRENT_SCALE = 4.5466;
constexpr float SERVO_CURRENT_ZERO  = 2.520 / SERVO_CURRENT_SCALE;
}  // namespace ADS131M08

namespace ADC_1
{
constexpr bool ENABLED = true;

using Channel = Boardcore::ADS131M08Defs::Channel;

constexpr auto OX_VESSEL_PT_CHANNEL   = Channel::CHANNEL_0;
constexpr auto OX_FILLING_PT_CHANNEL  = Channel::CHANNEL_1;
constexpr auto N2_VESSEL_1_PT_CHANNEL = Channel::CHANNEL_2;
constexpr auto N2_VESSEL_2_PT_CHANNEL = Channel::CHANNEL_3;
constexpr auto SERVO_CURRENT_CHANNEL  = Channel::CHANNEL_4;
constexpr auto OX_VESSEL_LC_CHANNEL   = Channel::CHANNEL_5;
constexpr auto ROCKET_LC_CHANNEL      = Channel::CHANNEL_6;
constexpr auto N2_FILLING_PT_CHANNEL  = Channel::CHANNEL_7;

constexpr float CH0_SHUNT_RESISTANCE = 29.283f;
constexpr float CH1_SHUNT_RESISTANCE = 29.233f;
constexpr float CH2_SHUNT_RESISTANCE = 29.268f;
constexpr float CH3_SHUNT_RESISTANCE = 29.645f;
constexpr float CH7_SHUNT_RESISTANCE = 29.708f;
}  // namespace ADC_1

namespace ADC_2
{
constexpr bool ENABLED = true;

using Channel = Boardcore::ADS131M08Defs::Channel;

constexpr auto OX_TANK_PT_CHANNEL = Channel::CHANNEL_0;
constexpr auto N2_TANK_PT_CHANNEL = Channel::CHANNEL_1;

constexpr float CH0_SHUNT_RESISTANCE = 29.685f;
constexpr float CH1_SHUNT_RESISTANCE = 29.625f;
}  // namespace ADC_2

namespace MAX31856
{
constexpr Hertz PERIOD = 10_hz;
constexpr bool ENABLED = true;
}  // namespace MAX31856

namespace Trafag
{
constexpr float OX_VESSEL_SHUNT_RESISTANCE  = ADC_1::CH0_SHUNT_RESISTANCE;
constexpr float OX_FILLING_SHUNT_RESISTANCE = ADC_1::CH1_SHUNT_RESISTANCE;
constexpr float N2_VESSEL1_SHUNT_RESISTANCE = ADC_1::CH2_SHUNT_RESISTANCE;
constexpr float N2_VESSEL2_SHUNT_RESISTANCE = ADC_1::CH3_SHUNT_RESISTANCE;
constexpr float N2_FILLING_SHUNT_RESISTANCE = ADC_1::CH7_SHUNT_RESISTANCE;
constexpr float OX_TANK_SHUNT_RESISTANCE    = ADC_2::CH0_SHUNT_RESISTANCE;
constexpr float N2_TANK_SHUNT_RESISTANCE    = ADC_2::CH1_SHUNT_RESISTANCE;

constexpr float MIN_CURRENT = 4;
constexpr float MAX_CURRENT = 20;

constexpr float OX_VESSEL_MAX_PRESSURE  = 100;  // bar
constexpr float OX_FILLING_MAX_PRESSURE = 100;  // bar
constexpr float N2_VESSEL1_MAX_PRESSURE = 400;  // bar
constexpr float N2_VESSEL2_MAX_PRESSURE = 400;  // bar
constexpr float N2_FILLING_MAX_PRESSURE = 400;  // bar
constexpr float OX_TANK_MAX_PRESSURE    = 100;  // bar
constexpr float N2_TANK_MAX_PRESSURE    = 400;  // bar
}  // namespace Trafag

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
constexpr float ROCKET_P0_VOLTAGE = -0.0029;
constexpr float ROCKET_P0_MASS    = 5.072;
constexpr float ROCKET_P1_VOLTAGE = -0.013327;
constexpr float ROCKET_P1_MASS    = 25.300;

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

namespace InternalADC
{
constexpr auto BATTERY_VOLTAGE_CHANNEL = Boardcore::InternalADC::CH14;

constexpr float BATTERY_VOLTAGE_SCALE = 4.7917;
constexpr Hertz PERIOD                = 10_hz;
constexpr bool ENABLED                = true;
}  // namespace InternalADC

}  // namespace Sensors

}  // namespace Config

}  // namespace RIGv2
