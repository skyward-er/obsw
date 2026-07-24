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

namespace Payload
{
namespace Config
{
namespace HIL
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr bool ENABLE_HW = false;

// Period of simulation [ms]
constexpr auto SIMULATION_RATE = 10_hz;

// Number of samples per sensor at each simulator iteration
constexpr int N_DATA_ACCEL       = 10;  // #samples
constexpr int N_DATA_GYRO        = 10;  // #samples
constexpr int N_DATA_MAGNETO     = 10;  // #samples
constexpr int N_DATA_GPS         = 1;   // #samples
constexpr int N_DATA_BARO_STATIC = 10;  // #samples
constexpr int N_DATA_BARO_PITOT  = 10;  // #samples
constexpr int N_DATA_TEMP        = 1;   // #samples

// clang-format off
// Checking if the data coming from simulator is enough
static_assert(N_DATA_ACCEL * SIMULATION_RATE >= Sensors::LSM6DSRX_0::RATE,
              "N_DATA_ACCEL not enough");
static_assert(N_DATA_GYRO * SIMULATION_RATE >= Sensors::LSM6DSRX_0::RATE,
              "N_DATA_GYRO not enough");
static_assert(N_DATA_ACCEL * SIMULATION_RATE >= Sensors::LSM6DSRX_1::RATE,
              "N_DATA_ACCEL not enough");
static_assert(N_DATA_GYRO * SIMULATION_RATE >= Sensors::LSM6DSRX_1::RATE,
              "N_DATA_GYRO not enough");
static_assert(N_DATA_MAGNETO * SIMULATION_RATE >= Sensors::LIS2MDL::RATE,
              "N_DATA_MAGNETO not enough");
static_assert(N_DATA_GPS * SIMULATION_RATE >= Sensors::UBXGPS::RATE,
              "N_DATA_GPS not enough");
static_assert(N_DATA_BARO_STATIC * SIMULATION_RATE >= Sensors::LPS22DF::RATE,
              "N_DATA_BARO_STATIC not enough");
static_assert(N_DATA_BARO_PITOT * SIMULATION_RATE >= Sensors::ND015A::RATE,
              "N_DATA_BARO_PITOT not enough");
static_assert(N_DATA_BARO_PITOT * SIMULATION_RATE >= Sensors::ND030D::RATE,
              "N_DATA_BARO_PITOT not enough");
// clang-format on

}  // namespace HIL
}  // namespace Config
}  // namespace Payload
