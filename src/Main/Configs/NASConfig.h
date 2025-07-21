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

#include <algorithms/NAS/NASConfig.h>
#include <common/ReferenceConfig.h>
#include <units/Frequency.h>

namespace Main
{

namespace Config
{

namespace NAS
{

/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr Hertz UPDATE_RATE         = 50_hz;
constexpr float UPDATE_RATE_SECONDS = 0.02;  // [s]

constexpr int CALIBRATION_SAMPLES_COUNT       = 20;
constexpr unsigned int CALIBRATION_SLEEP_TIME = 100;  // [ms]

static const Boardcore::NASConfig CONFIG = {
    .T                   = UPDATE_RATE_SECONDS,
    .SIGMA_BETA          = 0.0001,
    .SIGMA_W             = 0.0019,
    .SIGMA_ACC           = 0.202,
    .SIGMA_MAG           = 0.0047,
    .SIGMA_GPS           = {0.0447f, 0.0447f, 1.0f / 30.0f, 1.0f / 30.0f},
    .SIGMA_BAR           = 400.0f,
    .SIGMA_POS           = 2.0,
    .SIGMA_VEL           = 1.0,
    .SIGMA_PITOT_STATIC  = 75.0,
    .SIGMA_PITOT_DYNAMIC = 75.0,
    .P_POS               = 0.0,
    .P_POS_VERTICAL      = 0.0,
    .P_VEL               = 0.0,
    .P_VEL_VERTICAL      = 0.0,
    .P_ATT               = 0.1,
    .P_BIAS              = 0.01,
    .SATS_NUM            = 6.0,
    .NED_MAG             = Common::ReferenceConfig::nedMag};

// Only use one out of every 50 samples (1 Hz)
constexpr int MAGNETOMETER_DECIMATE = 50;

// Maximum allowed acceleration to correct with GPS
constexpr float DISABLE_GPS_ACCELERATION = 34.0f;  // [m/s^2]

// How much confidence (in m/s^2) to apply to the accelerometer to check if it
// is 1g
constexpr float ACCELERATION_1G_CONFIDENCE = 0.5;
// How many samples will determine that we are in fact measuring gravity
// acceleration
constexpr int ACCELERATION_1G_SAMPLES = 20;

// Altitude under which we make the pitot correction
constexpr float PITOT_ALTITUDE_THRESHOLD = 2000;  // [m]

// Vertical speed over which we make the pitot correction
constexpr float PITOT_SPEED_THRESHOLD = 70;  // [m/s]

}  // namespace NAS

}  // namespace Config

}  // namespace Main
