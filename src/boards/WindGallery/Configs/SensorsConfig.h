/* Copyright (c) 2015-2021 Skyward Experimental Rocketry
 * Authors: Luca Erbetta, Luca Conterio, Alberto Nidasio
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

#include <sensors/ADS1118/ADS1118.h>

namespace WindGallery
{

namespace SensorsConfigs
{

static constexpr Boardcore::ADS1118::ADS1118Mux ADC_CH_PITOT_PORT =
    Boardcore::ADS1118::MUX_AIN1_GND;
static constexpr Boardcore::ADS1118::ADS1118DataRate ADC_DR_PITOT_PORT =
    Boardcore::ADS1118::DR_860;
static constexpr Boardcore::ADS1118::ADS1118Pga ADC_PGA_PITOT_PORT =
    Boardcore::ADS1118::FSR_6_144;
static constexpr float ADC_REFERENCE_VOLTAGE = 5.0;

// Sampling periods in milliseconds
static constexpr unsigned int SAMPLE_PERIOD_ADC_ADS1118 = 6;

}  // namespace SensorsConfigs

}  // namespace WindGallery
