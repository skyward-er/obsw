/* Copyright (c) 2025 Skyward Experimental Rocketry
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

#include <drivers/i2c/I2CDriver.h>
#include <units/Frequency.h>

#include <chrono>

namespace RIGv3
{
namespace Config
{

constexpr uint32_t SPARK_PLUG_TIMEOUT = 1000;  // ms

namespace Expanders
{
constexpr Boardcore::I2CDriver::I2CSlaveConfig I2CExpander0Config{
    .slaveAddress = MIOSIX_SERVO_EXPANDER_0_ADDR,
    .addressing   = Boardcore::I2CDriver::Addressing::BIT7,
    .speed        = Boardcore::I2CDriver::Speed::STANDARD,
    .MSBFirst     = false};

constexpr Boardcore::I2CDriver::I2CSlaveConfig I2CExpander1Config{
    .slaveAddress = MIOSIX_SERVO_EXPANDER_1_ADDR,
    .addressing   = Boardcore::I2CDriver::Addressing::BIT7,
    .speed        = Boardcore::I2CDriver::Speed::STANDARD,
    .MSBFirst     = false};

}  // namespace Expanders

namespace Servos
{
/* linter off */ using namespace Boardcore::Units::Frequency;
/* linter off */ using namespace std::chrono;

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

constexpr uint32_t DEFAULT_OX_FIL_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_OX_REL_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_PRZ_FUEL_OPENING_TIME  = 15000;
constexpr uint32_t DEFAULT_PRZ_FIL_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_PRZ_REL_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_MAIN_FUEL_OPENING_TIME = 15000;
constexpr uint32_t DEFAULT_PRZ_OX_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_OX_VEN_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_PRZ_QUE_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_MAIN_OX_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_FUEL_VEN_OPENING_TIME  = 15000;
constexpr uint32_t DEFAULT_IGN_OX_OPENING_TIME    = 5000;
constexpr uint32_t DEFAULT_IGN_FUEL_OPENING_TIME  = 5000;
constexpr uint32_t DEFAULT_PRZ_DET_OPENING_TIME   = 15000;
constexpr uint32_t DEFAULT_OX_DET_OPENING_TIME    = 15000;
constexpr uint32_t DEFAULT_PURGE_OPENING_TIME     = 5000;

constexpr float DEFAULT_OX_FIL_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_OX_REL_MAX_APERTURE    = 0.55;
constexpr float DEFAULT_PRZ_FUEL_MAX_APERTURE  = 1.0;
constexpr float DEFAULT_PRZ_FIL_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_PRZ_REL_MAX_APERTURE   = 0.55;
constexpr float DEFAULT_MAIN_FUEL_MAX_APERTURE = 1.0;
constexpr float DEFAULT_PRZ_OX_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_OX_VEN_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_PRZ_QUE_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_MAIN_OX_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_FUEL_VEN_MAX_APERTURE  = 1.0;
constexpr float DEFAULT_IGN_OX_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_IGN_FUEL_MAX_APERTURE  = 1.0;
constexpr float DEFAULT_PRZ_DET_MAX_APERTURE   = 1.0;
constexpr float DEFAULT_OX_DET_MAX_APERTURE    = 1.0;
constexpr float DEFAULT_PURGE_MAX_APERTURE     = 1.0;

constexpr float OX_FIL_LIMIT    = 0.9;
constexpr float OX_REL_LIMIT    = 0.9;
constexpr float PRZ_FUEL_LIMIT  = 0.85;
constexpr float PRZ_3W_LIMIT    = 1.0;
constexpr float PRZ_FIL_LIMIT   = 1.0;
constexpr float PRZ_REL_LIMIT   = 0.9;
constexpr float MAIN_FUEL_LIMIT = 1.0;
constexpr float PRZ_OX_LIMIT    = 0.9;
constexpr float OX_VEN_LIMIT    = 0.9;
constexpr float PRZ_QUE_LIMIT   = 0.9;
constexpr float MAIN_OX_LIMIT   = 1.0;
constexpr float FUEL_VEN_LIMIT  = 0.9;
constexpr float IGN_FUEL_LIMIT  = 1.0;
constexpr float IGN_OX_LIMIT    = 1.0;
constexpr float PRZ_DET_LIMIT   = 1.0;
constexpr float OX_DET_LIMIT    = 1.0;
constexpr float PURGE_LIMIT     = 1.0;

constexpr bool OX_FIL_FLIPPED    = true;
constexpr bool OX_REL_FLIPPED    = true;
constexpr bool PRZ_FUEL_FLIPPED  = true;
constexpr bool PRZ_OX_FLIPPED    = true;
constexpr bool PRZ_3W_FLIPPED    = true;
constexpr bool PRZ_FIL_FLIPPED   = false;
constexpr bool PRZ_REL_FLIPPED   = false;
constexpr bool MAIN_FUEL_FLIPPED = true;
constexpr bool NITR_FLIPPED      = true;
constexpr bool OX_VEN_FLIPPED    = true;
constexpr bool PRZ_QUE_FLIPPED   = false;
constexpr bool MAIN_OX_FLIPPED   = true;
constexpr bool FUEL_VEN_FLIPPED  = true;
constexpr bool IGN_OX_FLIPPED    = false;
constexpr bool IGN_FUEL_FLIPPED  = false;
constexpr bool PRZ_DET_FLIPPED   = false;
constexpr bool OX_DET_FLIPPED    = false;
constexpr bool PURGE_FLIPPED     = false;

}  // namespace Servos
}  // namespace Config
}  // namespace RIGv3
