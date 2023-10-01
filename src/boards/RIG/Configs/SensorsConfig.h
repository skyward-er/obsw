/* Copyright (c) 2023 Skyward Experimental Rocketry
 * Authors: Matteo Pignataro
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
#include <stdint.h>

namespace RIG
{
namespace Config
{
namespace Sensors
{
constexpr uint16_t ADC_SAMPLE_PERIOD          = 10;
constexpr uint16_t LOAD_CELL_SAMPLE_PERIOD    = 13;
constexpr uint16_t THERMOCOUPLE_SAMPLE_PERIOD = 100;

constexpr float ADC1_CH1_SHUNT_RESISTANCE = 7.3;
constexpr float ADC1_CH2_SHUNT_RESISTANCE = 7.3;
constexpr float ADC1_CH3_SHUNT_RESISTANCE = 7.3;
constexpr float ADC1_CH4_SHUNT_RESISTANCE = 7.3;

constexpr float ADC2_CH1_SHUNT_RESISTANCE = 7.3;

constexpr float FILLING_MAX_PRESSURE     = 100;  // bar
constexpr float TANK_TOP_MAX_PRESSURE    = 100;  // bar
constexpr float TANK_BOTTOM_MAX_PRESSURE = 100;  // bar
constexpr float VESSEL_MAX_PRESSURE      = 400;  // bar

constexpr float SENSOR_MIN_CURRENT = 4;   // mA
constexpr float SENSOR_MAX_CURRENT = 20;  // mA

constexpr float LOAD_CELL2_OFFSET = -38150;
constexpr float LOAD_CELL1_OFFSET = -93539.8f;

constexpr float LOAD_CELL2_SCALE = -5.6188e-5f;
constexpr float LOAD_CELL1_SCALE = -2.251307e-5f;

constexpr float VOLTAGE_CONVERSION_FACTOR         = 13.58f;
constexpr float CURRENT_CONVERSION_FACTOR         = 13.645f;
constexpr float VOLTAGE_CURRENT_CONVERSION_FACTOR = 0.020f;  // 40 mV/A
}  // namespace Sensors
}  // namespace Config
}  // namespace RIG