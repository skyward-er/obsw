/* Copyright (c) 2025 Skyward Experimental Rocketry
 * Author: Niccolò Betto
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

#include <miosix.h>

#define MIOSIX_SERVOS_MAIN_OX_TIM TIM3
#define MIOSIX_SERVOS_MAIN_FUEL_TIM TIM4

#define MIOSIX_SERVOS_MAIN_OX_CHANNEL CHANNEL_2
#define MIOSIX_SERVOS_MAIN_FUEL_CHANNEL CHANNEL_2

#define MAIN_FUEL_LED MainFuelLed
#define MAIN_OX_LED MainOxLed

#define IRQ_HANDLER __attribute__((used)) void

#define DEWESOFT_INTERRUPT_1 EXTI2_IRQHandlerImpl
#define DEWESOFT_INTERRUPT_2 EXTI4_IRQHandlerImpl
#define DEWESOFT_INTERRUPT_3 EXTI6_IRQHandlerImpl

namespace hwmapping
{
using StatusLed = miosix::Gpio<GPIOG_BASE, 13>;
using ActionLed = miosix::Gpio<GPIOG_BASE, 14>;

using MainOxTimer   = miosix::Gpio<GPIOA_BASE, 7>;   // TIM3_CH2
using MainFuelTimer = miosix::Gpio<GPIOD_BASE, 13>;  // TIM4_CH2

using DewesoftInterrupt1 = miosix::Gpio<GPIOE_BASE, 2>;
using DewesoftInterrupt2 = miosix::Gpio<GPIOE_BASE, 4>;
using DewesoftInterrupt3 = miosix::Gpio<GPIOE_BASE, 6>;

inline void init()
{
    StatusLed::mode(miosix::Mode::OUTPUT);
    StatusLed::low();
    ActionLed::mode(miosix::Mode::OUTPUT);
    ActionLed::low();

    MainOxTimer::alternateFunction(2);
    MainOxTimer::mode(miosix::Mode::ALTERNATE);
    MainFuelTimer::alternateFunction(2);
    MainFuelTimer::mode(miosix::Mode::ALTERNATE);

    DewesoftInterrupt1::mode(miosix::Mode::INPUT_PULL_DOWN);
    DewesoftInterrupt2::mode(miosix::Mode::INPUT_PULL_DOWN);
    DewesoftInterrupt3::mode(miosix::Mode::INPUT_PULL_DOWN);
}
}  // namespace hwmapping
