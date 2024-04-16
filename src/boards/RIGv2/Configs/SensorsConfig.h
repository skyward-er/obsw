/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor
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

#include <cstdint>

namespace RIGv2
{

namespace Config
{

namespace Sensors
{

namespace ADS131M08
{

static constexpr float CH1_SHUNT_RESISTANCE = 29.4048;
static constexpr float CH2_SHUNT_RESISTANCE = 29.5830;
static constexpr float CH3_SHUNT_RESISTANCE = 29.4973;
static constexpr float CH4_SHUNT_RESISTANCE = 29.8849;

// ADC channels definitions for various sensors
static constexpr Boardcore::ADS131M08Defs::Channel VESSEL_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_0;
static constexpr Boardcore::ADS131M08Defs::Channel FILLING_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_1;
static constexpr Boardcore::ADS131M08Defs::Channel BOTTOM_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_2;
static constexpr Boardcore::ADS131M08Defs::Channel TOP_PT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_3;
static constexpr Boardcore::ADS131M08Defs::Channel SERVO_CURRENT_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_4;
static constexpr Boardcore::ADS131M08Defs::Channel VESSEL_LC_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_5;
static constexpr Boardcore::ADS131M08Defs::Channel TANK_LC_CHANNEL =
    Boardcore::ADS131M08Defs::Channel::CHANNEL_6;

// Servo current sensor calibration data
// - A: 0.0 V: 2.520
// - A: 0.5 V: 2.513
// - A: 1.0 V: 2.505
// - A: 1.5 V: 2.498
// - A: 2.0 V: 2.490
// - A: 2.5 V: 2.484
// - A: 5.2 V: 2.441
static constexpr float SERVO_CURRENT_SCALE = 4.5466;
static constexpr float SERVO_CURRENT_ZERO  = 2.520 / SERVO_CURRENT_SCALE;

static constexpr uint32_t SAMPLE_PERIOD = 10;
static constexpr bool ENABLED           = true;
}  // namespace ADS131M08

namespace MAX31856
{
static constexpr uint32_t SAMPLE_PERIOD = 100;
static constexpr bool ENABLED           = true;
}  // namespace MAX31856

namespace Trafag
{
static constexpr float FILLING_SHUNT_RESISTANCE =
    ADS131M08::CH2_SHUNT_RESISTANCE;
static constexpr float TANK_TOP_SHUNT_RESISTANCE =
    ADS131M08::CH4_SHUNT_RESISTANCE;
static constexpr float TANK_BOTTOM_SHUNT_RESISTANCE =
    ADS131M08::CH3_SHUNT_RESISTANCE;
static constexpr float VESSEL_SHUNT_RESISTANCE =
    ADS131M08::CH1_SHUNT_RESISTANCE;

static constexpr float MIN_CURRENT = 4;
static constexpr float MAX_CURRENT = 20;

static constexpr float FILLING_MAX_PRESSURE     = 100;  // bar
static constexpr float TANK_TOP_MAX_PRESSURE    = 100;  // bar
static constexpr float TANK_BOTTOM_MAX_PRESSURE = 100;  // bar
static constexpr float VESSEL_MAX_PRESSURE      = 400;  // bar
}  // namespace Trafag

namespace LoadCell
{
static constexpr unsigned int CALIBRATE_SAMPLE_COUNT  = 10;
static constexpr unsigned int CALIBRATE_SAMPLE_PERIOD = 40;

// LC Tank sensor calibration data
static constexpr float TANK_P0_VOLTAGE = 0.000941;
static constexpr float TANK_P0_MASS    = 1.866;
static constexpr float TANK_P1_VOLTAGE = 0.003559;
static constexpr float TANK_P1_MASS    = 6.916;

// LC Vessel sensor calibration data
// - 1.866kg V: 0.00027
// - 5.050kg V: 0.00073
// - 6.916kg V: 0.00100
static constexpr float VESSEL_P0_VOLTAGE = 0.00027;
static constexpr float VESSEL_P0_MASS    = 1.866;
static constexpr float VESSEL_P1_VOLTAGE = 0.0010;
static constexpr float VESSEL_P1_MASS    = 6.916;
}  // namespace LoadCell

namespace InternalADC
{
static constexpr Boardcore::InternalADC::Channel BATTERY_VOLTAGE_CHANNEL =
    Boardcore::InternalADC::CH14;

static constexpr float BATTERY_VOLTAGE_SCALE = 4.7917;
static constexpr uint32_t SAMPLE_PERIOD      = 100;
static constexpr bool ENABLED                = true;
}  // namespace InternalADC

}  // namespace Sensors

}  // namespace Config

}  // namespace RIGv2