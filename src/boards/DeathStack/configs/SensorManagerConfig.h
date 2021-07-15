/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <drivers/adc/ADS1118/ADS1118.h>
#include <interfaces-impl/hwmapping.h>

using miosix::Gpio;

namespace DeathStackBoard
{

namespace SensorConfigs
{
static constexpr ADS1118::ADS1118Mux ADC_CH_STATIC_PORT = ADS1118::MUX_AIN0_GND;
static constexpr ADS1118::ADS1118Mux ADC_CH_PITOT_PORT  = ADS1118::MUX_AIN1_GND;
static constexpr ADS1118::ADS1118Mux ADC_CH_DPL_PORT    = ADS1118::MUX_AIN2_GND;
static constexpr ADS1118::ADS1118Mux ADC_CH_VREF        = ADS1118::MUX_AIN3_GND;

static constexpr ADS1118::ADS1118DataRate ADC_DR_STATIC_PORT = ADS1118::DR_860;
static constexpr ADS1118::ADS1118DataRate ADC_DR_PITOT_PORT  = ADS1118::DR_860;
static constexpr ADS1118::ADS1118DataRate ADC_DR_DPL_PORT    = ADS1118::DR_860;
static constexpr ADS1118::ADS1118DataRate ADC_DR_VREF        = ADS1118::DR_860;

static constexpr ADS1118::ADS1118Pga ADC_PGA_STATIC_PORT = ADS1118::FSR_6_144;
static constexpr ADS1118::ADS1118Pga ADC_PGA_PITOT_PORT  = ADS1118::FSR_6_144;
static constexpr ADS1118::ADS1118Pga ADC_PGA_DPL_PORT    = ADS1118::FSR_6_144;
static constexpr ADS1118::ADS1118Pga ADC_PGA_VREF        = ADS1118::FSR_6_144;

// Sampling periods
static constexpr unsigned int SP_ADC = 3;

static constexpr unsigned int SP_DIGITAL_PRESS = 1000 / 50;
static constexpr unsigned int SP_PITOT_PRESS   = SP_ADC * 4;
static constexpr unsigned int SP_DPL_PRESS     = SP_ADC * 4;
static constexpr unsigned int SP_STATIC_PRESS  = SP_ADC * 4;

static constexpr float REFERENCE_VOLTAGE = 4.8;  // TODO: Measure it
}  // namespace SensorConfigs

}  // namespace DeathStackBoard
