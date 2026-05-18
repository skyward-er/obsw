/* Copyright (c) 2024 Skyward Experimental Rocketry
 * Author: Davide Mor
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
#include <units/Angle.h>
#include <units/Frequency.h>

namespace Main
{
namespace Config
{
namespace Actuators
{

/* linter off */ using namespace std::chrono_literals;
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace Boardcore::Units::Angle;
/* linter off */ using namespace Boardcore::Units::Time;

enum class ServoDirection
{
    CW,
    CCW
};

namespace PrfServo
{
constexpr auto MIN_PULSE               = 500_us;
constexpr auto MAX_PULSE               = 2440_us;
constexpr auto HERTZ                   = 333_hz;
constexpr auto SCHMITT_THRESHOLD_LOW   = 10_deg;
constexpr auto SCHMITT_THRESHOLD_HIGH  = 10_deg;
constexpr auto HIGH_THRESHOLD_VELOCITY = 1.f;
constexpr auto LOW_THRESHOLD_VELOCITY  = 0.f;
constexpr auto STOP_THRESHOLD_VELOCITY = 0.5f;
constexpr auto INITIAL_ANGLE           = 0.0_deg;
constexpr auto LEFT_SERVO_DIRECTION    = ServoDirection::CW;
constexpr auto RIGHT_SERVO_DIRECTION   = ServoDirection::CCW;
constexpr auto MAX_ANGLE               = 1080_deg;
constexpr auto LEFT_MIN_ANGLE          = 45_deg;
constexpr auto RIGHT_MIN_ANGLE         = 45_deg;

constexpr auto UPDATE_RATE = 50_hz;
}  // namespace PrfServo

constexpr auto SERVO_TWIRL_RADIUS = 0.5f;  // [%]

constexpr unsigned int ABK_MIN_PULSE = 1950;
constexpr unsigned int ABK_MAX_PULSE = 1390;

// Inverted to invert the servo logic
constexpr unsigned int EXP_MIN_PULSE = 900;
constexpr unsigned int EXP_MAX_PULSE = 2000;

// Buzzer configs
constexpr uint16_t BUZZER_FREQUENCY = 500;
constexpr float BUZZER_DUTY_CYCLE   = 0.5;

constexpr Hertz BUZZER_UPDATE_RATE  = 10_hz;
constexpr uint32_t BUZZER_ARM_RATE  = 5;   // 5 * 100ms = 500ms
constexpr uint32_t BUZZER_LAND_RATE = 10;  // 10 * 100ms = 1000ms

// Status configs
constexpr Hertz STATUS_UPDATE_RATE = 10_hz;
constexpr uint32_t STATUS_OK_RATE  = 10;  // 10 * 100ms = 1000ms
constexpr uint32_t STATUS_ERR_RATE = 1;   // 1 * 100ms = 100ms

}  // namespace Actuators
}  // namespace Config
}  // namespace Main
