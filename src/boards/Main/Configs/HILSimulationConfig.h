/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Emilio Corigliano
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

namespace Main
{
namespace Config
{
namespace HIL
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr bool IS_FULL_HIL = false;
constexpr bool ENABLE_HW   = false;

// Period of simulation [ms]
constexpr auto SIMULATION_RATE = 10_hz;

// Period of "sampling" of can sensors for non-FullHIL simulations
constexpr auto BARO_CHAMBER_RATE = 50_hz;
constexpr auto BARO_PITOT_RATE   = 20_hz;

// Number of samples per sensor at each simulator iteration
constexpr int N_DATA_ACCEL        = 10;  // #samples
constexpr int N_DATA_GYRO         = 10;  // #samples
constexpr int N_DATA_MAGNETO      = 10;  // #samples
constexpr int N_DATA_GPS          = 1;   // #samples
constexpr int N_DATA_BARO_STATIC  = 10;  // #samples
constexpr int N_DATA_BARO_CHAMBER = 5;   // #samples
constexpr int N_DATA_PITOT        = 10;  // #samples
constexpr int N_DATA_TEMP         = 1;   // #samples

// Checking if the data coming from simulator is enough
static_assert(N_DATA_ACCEL * SIMULATION_RATE >= Sensors::LSM6DSRX::RATE,
              "N_DATA_ACCEL not enough");
static_assert(N_DATA_GYRO * SIMULATION_RATE >= Sensors::LSM6DSRX::RATE,
              "N_DATA_GYRO not enough");
static_assert(N_DATA_MAGNETO * SIMULATION_RATE >= Sensors::LIS2MDL::RATE,
              "N_DATA_MAGNETO not enough");
static_assert(N_DATA_GPS * SIMULATION_RATE >= Sensors::UBXGPS::RATE,
              "N_DATA_GPS not enough");
static_assert(N_DATA_BARO_STATIC * SIMULATION_RATE >= Sensors::ADS131M08::RATE,
              "N_DATA_BARO_STATIC not enough");
static_assert(N_DATA_BARO_STATIC * SIMULATION_RATE >= Sensors::LPS22DF::RATE,
              "N_DATA_BARO_STATIC not enough");
static_assert(N_DATA_BARO_STATIC * SIMULATION_RATE >= Sensors::LPS28DFW::RATE,
              "N_DATA_BARO_STATIC not enough");
static_assert(N_DATA_BARO_CHAMBER * SIMULATION_RATE >= BARO_CHAMBER_RATE,
              "N_DATA_BARO_CHAMBER not enough");
static_assert(N_DATA_PITOT * SIMULATION_RATE >= BARO_PITOT_RATE,
              "N_DATA_PITOT not enough");

}  // namespace HIL
}  // namespace Config
}  // namespace Main