/* Copyright (c) 2023 Skyward Experimental Rocketry
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

#include <drivers/usart/USART.h>

#include <utils/ModuleManager/ModuleManager.hpp>

namespace Antennas
{

miosix::GpioPin usart2_tx = miosix::GpioPin(GPIOA_BASE, 2);
miosix::GpioPin usart2_rx = miosix::GpioPin(GPIOA_BASE, 3);
miosix::GpioPin uart4_tx  = miosix::GpioPin(GPIOA_BASE, 0);
miosix::GpioPin uart4_rx  = miosix::GpioPin(GPIOA_BASE, 1);

class Buses : public Boardcore::Module
{
public:
    Boardcore::USART usart2;
    Boardcore::USART uart4;

    Buses() : usart2(USART2, 115200), uart4(UART4, 115200)
    {
        usart2_tx.mode(miosix::Mode::ALTERNATE);
        usart2_tx.alternateFunction(7);
        usart2_rx.mode(miosix::Mode::ALTERNATE);
        usart2_rx.alternateFunction(7);

        uart4_tx.mode(miosix::Mode::ALTERNATE);
        uart4_tx.alternateFunction(8);
        uart4_rx.mode(miosix::Mode::ALTERNATE);
        uart4_rx.alternateFunction(8);
    }
};
}  // namespace Antennas