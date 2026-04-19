/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto, Fabrizio Monti, Riccardo Sironi
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
#include <units/Units.h>

#include <chrono>

namespace Motor
{

namespace Config
{

namespace Sensors
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

namespace ADC_1
{
// TODO: DA verificare l'OSR
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

using namespace Boardcore::ADS131M08Defs;

constexpr auto CC_PT_CHANNEL                 = Channel::CHANNEL_0;
constexpr auto FUEL_TANK_PT_CHANNEL          = Channel::CHANNEL_1;
constexpr auto PRZ_TANK_PT_CHANNEL           = Channel::CHANNEL_2;
constexpr auto OX_TANK_PT_CHANNEL            = Channel::CHANNEL_3;
constexpr auto REGULATOR_OUT_FUEL_PT_CHANNEL = Channel::CHANNEL_4;
constexpr auto REGULATOR_OUT_OX_PT_CHANNEL   = Channel::CHANNEL_5;
constexpr auto IGNITER_PT_CHANNEL            = Channel::CHANNEL_6;
// constexpr auto EXTRA_PT                      = Channel::CHANNEL_7;

constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADC_1

namespace ADC_2
{
// TODO: DA verificare l'OSR
constexpr auto OSR = Boardcore::ADS131M08Defs::OversamplingRatio::OSR_8192;
constexpr bool GLOBAL_CHOP_MODE_EN = true;

using namespace Boardcore::ADS131M08Defs;

constexpr auto FUEL_MAIN_EN_CHANNEL    = Channel::CHANNEL_1;
constexpr auto PRZ_OX_EN_CHANNEL       = Channel::CHANNEL_2;
constexpr auto PRZ_FUEL_EN_CHANNEL     = Channel::CHANNEL_3;
constexpr auto OX_VENTING_EN_CHANNEL   = Channel::CHANNEL_4;
constexpr auto FUEL_VENTING_EN_CHANNEL = Channel::CHANNEL_5;
// constexpr auto EXTRA_TOP_EN_CHANNEL    = Channel::CHANNEL_6;
// constexpr auto EXTRA_BOTTOM_EN_CHANNEL = Channel::CHANNEL_7;

constexpr auto RATE    = 100_hz;
constexpr bool ENABLED = true;
}  // namespace ADC_2

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

constexpr float FUEL_TANK_MAX_PRESSURE          = 400;  // bar
constexpr float PRZ_TANK_MAX_PRESSURE           = 400;  // bar
constexpr float REGULATOR_OUT_FUEL_MAX_PRESSURE = 100;  // bar
constexpr float REGULATOR_OUT_OX_MAX_PRESSURE   = 100;  // bar
constexpr float OX_TANK_MAX_PRESSURE            = 100;  // bar
constexpr float CC_MAX_PRESSURE                 = 40;   // bar
constexpr float IGNITER_MAX_PRESSURE            = 40;   // bar

}  // namespace Trafag

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

namespace OxTankOverpressure
{
constexpr auto CHECK_RATE = 10_hz;

constexpr float PRESSURE_THRESHOLD = 69.0;  // bar
constexpr auto HYSTERESIS          = 1s;
constexpr auto VENTING_DURATION    = 1s;
}  // namespace OxTankOverpressure

}  // namespace Sensors

}  // namespace Config

}  // namespace Motor
