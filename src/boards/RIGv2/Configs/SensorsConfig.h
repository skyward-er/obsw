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

static constexpr uint32_t ADC_SAMPLE_PERIOD = 10;
static constexpr uint32_t TC_SAMPLE_PERIOD = 100;

static constexpr float ADC1_CH1_SHUNT_RESISTANCE = 30;
static constexpr float ADC1_CH2_SHUNT_RESISTANCE = 30;
static constexpr float ADC1_CH3_SHUNT_RESISTANCE = 30;
static constexpr float ADC1_CH4_SHUNT_RESISTANCE = 30;

// ADC channels definitions for various sensors
static constexpr int ADC1_VESSEL_PT_CHANNEL = 0;
static constexpr int ADC1_FILLING_PT_CHANNEL = 1;
static constexpr int ADC1_BOTTOM_PT_CHANNEL = 2;
static constexpr int ADC1_TOP_PT_CHANNEL = 3;
static constexpr int ADC1_VESSEL_LC_CHANNEL = 6;
static constexpr int ADC1_TANK_LC_CHANNEL = 7;

static constexpr float PT_MIN_CURRENT = 4;
static constexpr float PT_MAX_CURRENT = 20;

static constexpr float FILLING_MAX_PRESSURE     = 100;  // bar
static constexpr float TANK_TOP_MAX_PRESSURE    = 100;  // bar
static constexpr float TANK_BOTTOM_MAX_PRESSURE = 100;  // bar
static constexpr float VESSEL_MAX_PRESSURE      = 400;  // bar

static constexpr unsigned int LC_CALIBRATE_SAMPLE_COUNT = 10;
static constexpr unsigned int LC_CALIBRATE_SAMPLE_PERIOD = 40;

}

}  // namespace Config

}  // namespace RIGv2