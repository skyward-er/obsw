/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano, Giulia Facchi
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

#include <units/Frequency.h>
#include <units/Units.h>

#include <chrono>

#include "SensorsConfig.h"

/* linter off */ using namespace Boardcore::Units::Frequency;

namespace Motor
{
namespace Config
{
namespace HIL
{

constexpr bool ENABLE_HW = true;

/** Frequency of the simulation */
constexpr auto SIMULATION_RATE = 10_hz;

/** sampling periods of sensors [ms] */
constexpr int N_DATA_BARO_CHAMBER = 10;

static_assert(N_DATA_BARO_CHAMBER * SIMULATION_RATE >=
                  Sensors::ADS131M08::PERIOD,
              "N_DATA_BARO_CHAMBER not enough");

}  // namespace HIL
}  // namespace Config
}  // namespace Motor
