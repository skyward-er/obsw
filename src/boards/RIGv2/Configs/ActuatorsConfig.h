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

#include <interfaces-impl/hwmapping.h>

namespace RIGv2
{
namespace Config
{

namespace Servos
{
static constexpr unsigned int MIN_PULSE = 500;
static constexpr unsigned int MAX_PULSE     = 2460;
static constexpr unsigned int FREQUENCY     = 333;

static constexpr uint32_t SERVO_TIMINGS_CHECK_PERIOD = 100;
static constexpr long long SERVO_CONFIDENCE_TIME     = 500;   // 0.5s
static constexpr float SERVO_CONFIDENCE              = 0.02;  // 2%

static constexpr uint32_t DEFAULT_FILLING_OPENING_TIME    = 15000;  // 15s
static constexpr uint32_t DEFAULT_VENTING_OPENING_TIME    = 15000;  // 15s
static constexpr uint32_t DEFAULT_MAIN_OPENING_TIME       = 6000;   // 6s
static constexpr uint32_t DEFAULT_RELEASE_OPENING_TIME    = 10000;  // 10s
static constexpr uint32_t DEFAULT_DISCONNECT_OPENING_TIME = 10000;  // 10s

static constexpr float DEFAULT_FILLING_MAX_APERTURE    = 1.00f;
static constexpr float DEFAULT_VENTING_MAX_APERTURE    = 1.00f;
static constexpr float DEFAULT_MAIN_MAX_APERTURE       = 1.00f;
static constexpr float DEFAULT_RELEASE_MAX_APERTURE    = 1.00f;
static constexpr float DEFAULT_DISCONNECT_MAX_APERTURE = 1.00f;

static constexpr float FILLING_LIMIT = 0.90f;
static constexpr float RELEASE_LIMIT = 0.50f;
static constexpr float VENTING_LIMIT = 0.90f;
static constexpr float MAIN_LIMIT    = 0.90f;

static constexpr bool FILLING_FLIPPED    = true;
static constexpr bool VENTING_FLIPPED    = true;
static constexpr bool MAIN_FLIPPED       = true;
static constexpr bool RELEASE_FLIPPED    = true;
static constexpr bool DISCONNECT_FLIPPED = false;

}  // namespace Servos

namespace Actuators
{
static constexpr uint32_t NITROGEN_OPENING_TIME = 2000;  // 2s
}

}  // namespace Config
}  // namespace RIGv2