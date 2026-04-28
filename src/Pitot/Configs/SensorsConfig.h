/* Copyright (c) 2026 Skyward Experimental Rocketry
 * Author: Leonardo Montecchi
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
#include <sensors/H3LIS331DL/H3LIS331DL.h>
#include <sensors/LIS2MDL/LIS2MDL.h>
#include <sensors/LPS22DF/LPS22DF.h>
#include <sensors/LSM6DSRX/LSM6DSRX.h>
#include <sensors/ND015X/ND015A.h>
#include <sensors/ND030X/ND030A.h>
#include <units/Frequency.h>

#include <chrono>
#include <string>

namespace Pitot
{

namespace Config
{

namespace Sensors
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

constexpr auto CALIBRATION_SAMPLES_COUNT = 20;
constexpr auto CALIBRATION_SLEEP_TIME    = 100ms;

namespace ND015A
{
constexpr auto IOW = Boardcore::ND015A::IOWatchdogEnable::DISABLED;
constexpr auto BWL = Boardcore::ND015A::BWLimitFilter::BWL_200;
constexpr auto NTC = Boardcore::ND015A::NotchEnable::DISABLED;

constexpr uint8_t ODR = 0x00;  // Auto select based on BW

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace ND015A

namespace ND030A
{
constexpr auto IOW = Boardcore::ND030A::IOWatchdogEnable::DISABLED;
constexpr auto BWL = Boardcore::ND030A::BWLimitFilter::BWL_200;
constexpr auto NTC = Boardcore::ND030A::NotchEnable::DISABLED;

constexpr uint8_t ODR = 0x00;  // Auto select based on BW

constexpr auto RATE    = 100_hz;
constexpr auto ENABLED = true;
}  // namespace ND030A

namespace HeatingPadNTC
{
    constexpr auto ENABLED = true;
    constexpr auto CH  = Boardcore::InternalADC::Channel::CH9;
    constexpr float REF_VOLTAGE = V_DDA_VOLTAGE;
    constexpr float REF_RESISTANCE = 10000.0f; // ohms
    constexpr float REF_TEMPERATURE = 298.15f; // K, 25°C
    constexpr float BETA  = 3435.0f;
} // namespace HeatingPadNTC


}  // namespace Sensors
}  // namespace Config
}  // namespace Pitot
