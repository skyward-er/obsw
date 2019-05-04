/* Copyright (c) 2015-2018 Skyward Experimental Rocketry
 * Authors: Luca Erbetta
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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#pragma once

#include <Common.h>
#include <drivers/BusTemplate.h>
#include <drivers/stm32f2_f4_i2c.h>
#include <interfaces-impl/hwmapping.h>

using miosix::Gpio;

namespace DeathStackBoard
{

// I2C 1
typedef ProtocolI2C<miosix::I2C1Driver> i2c1;

// SPI1
typedef BusSPI<1, miosix::interfaces::spi1::mosi,
               miosix::interfaces::spi1::miso, miosix::interfaces::spi1::sck>
    busSPI1;

// Spi protocol defs
typedef ProtocolSPI<busSPI1, miosix::sensors::mpu9250::cs> spiMPU9250;
typedef ProtocolSPI<busSPI1, miosix::sensors::adis16405::cs> spiADIS16405;
typedef ProtocolSPI<busSPI1, miosix::sensors::ms5803::cs> spiMS5803;

typedef miosix::sensors::ad7994::ab ad7994_busy_pin;
typedef miosix::sensors::ad7994::nconvst ad7994_nconvst;

static constexpr uint8_t AD7994_NXP_BARO_CHANNEL = 1;
static constexpr uint8_t AD7994_HONEYWELL_BARO_CHANNEL = 3;


static constexpr uint8_t INTERNAL_ADC_NUM = 3;
static constexpr uint8_t ADC_CURRENT_SENSE_1_CHANNEL = 6;
static constexpr uint8_t ADC_CURRENT_SENSE_2_CHANNEL = 4;
static constexpr uint8_t ADC_BATTERY_VOLTAGE_CHANNEL = 5;

}  // namespace DeathStackBoard

