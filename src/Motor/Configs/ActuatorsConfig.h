/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Authors: Davide Mor, Niccolò Betto
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
#include <units/Frequency.h>

#include <chrono>

namespace Motor
{
namespace Config
{
namespace Servos
{
/* linter off */ using namespace std::chrono;
/* linter off */ using namespace Boardcore::Units::Frequency;

// Pulse width for normal (74 kg) servos
constexpr unsigned int MIN_PULSE = 500;
constexpr unsigned int MAX_PULSE = 2440;

// Pulse width for small (16 kg) servos
constexpr unsigned int SMALL_MIN_PULSE = 900;
constexpr unsigned int SMALL_MAX_PULSE = 2100;

constexpr unsigned int FREQUENCY = 333;

constexpr auto SERVO_BACKSTEP_DELAY   = 500ms;
constexpr float SERVO_BACKSTEP_AMOUNT = 0.02;  // 2%

constexpr auto ANIMATION_UPDATE_PERIOD = 10ms;

constexpr auto MOVE_SERVO_TIMEOUT = 10000ms;

constexpr auto SAFETY_VENTING_TIMEOUT  = 45min;
constexpr auto SAFETY_VENTING_DURATION = 10min;  // How long to vent

constexpr uint32_t DEFAULT_OX_VEN_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_FUEL_VEN_OPENING_TIME  = 15000;
constexpr uint32_t DEFAULT_MAIN_OX_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_MAIN_FUEL_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_PRZ_OX_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_PRZ_FUEL_OPENING_TIME  = 15000;
constexpr uint32_t DEFAULT_IGN_OX_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_IGN_FUEL_OPENING_TIME  = 15000;

constexpr float DEFAULT_OX_VEN_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_FUEL_VEN_MAX_APERTURE  = 1.0;
constexpr float DEFAULT_MAIN_OX_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_MAIN_FUEL_MAX_APERTURE = 1.0;
constexpr float DEFAULT_PRZ_OX_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_PRZ_FUEL_MAX_APERTURE  = 1.0;
constexpr float DEFAULT_IGN_OX_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_IGN_FUEL_MAX_APERTURE  = 1.0;

constexpr float OX_VEN_LIMIT    = 0.9f;
constexpr float FUEL_VEN_LIMIT  = 0.9f;
constexpr float MAIN_OX_LIMIT   = 0.9f;
constexpr float MAIN_FUEL_LIMIT = 0.9f;
constexpr float PRZ_OX_LIMIT    = 0.9f;
constexpr float PRZ_FUEL_LIMIT  = 0.85f;
constexpr float IGN_FUEL_LIMIT  = 1.0;
constexpr float IGN_OX_LIMIT    = 1.0;

constexpr bool OX_VEN_FLIPPED    = true;
constexpr bool FUEL_VEN_FLIPPED  = true;
constexpr bool MAIN_OX_FLIPPED   = true;
constexpr bool MAIN_FUEL_FLIPPED = true;
constexpr bool PRZ_OX_FLIPPED    = true;
constexpr bool PRZ_FUEL_FLIPPED  = true;
constexpr bool IGN_OX_FLIPPED    = false;
constexpr bool IGN_FUEL_FLIPPED  = false;

}  // namespace Servos
}  // namespace Config
}  // namespace Motor
